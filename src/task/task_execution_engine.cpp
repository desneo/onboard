#include "task/task_execution_engine.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <magic_enum/magic_enum.hpp>

#include "task/nodes/arm_node.h"
#include "task/nodes/detect_node.h"
#include "task/nodes/guide_node.h"
#include "task/nodes/midcourse_node.h"
#include "task/nodes/takeoff_node.h"
#include "task/nodes/track_node.h"
#include "task/task_events.h"

namespace plane::task {

TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::core::SystemParamApi& system_param,
                                         plane::perception::PerceptionApi& perception,
                                         plane::target::TargetStoreApi& target_store)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store),
      system_param_(system_param),
      perception_(perception),
      target_store_(target_store) {}

TaskExecutionEngine::~TaskExecutionEngine() { halt_tree(); }

std::string TaskExecutionEngine::name() const { return "Task Execution Engine"; }

bool TaskExecutionEngine::init() { return true; }

bool TaskExecutionEngine::start() { return true; }

void TaskExecutionEngine::stop() { abort(); }

void TaskExecutionEngine::release() {}

BT::Tree TaskExecutionEngine::build_anti_uav() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_, &system_param_);
    factory.registerNodeType<MidcourseNode>("Midcourse", &flight_control_, &state_store_, &target_store_, &config_,
                                            &suicide_context_);
    factory.registerNodeType<DetectNode>("Detect", &flight_control_, &state_store_, &perception_, &target_store_,
                                         &config_, &suicide_context_);
    factory.registerNodeType<TrackNode>("Track", &flight_control_, &state_store_, &perception_, &target_store_,
                                        &config_, &suicide_context_);
    factory.registerNodeType<GuideNode>("Guide", &flight_control_, &state_store_, &perception_, &target_store_,
                                        &config_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "anti-uav";
    subtree->instance_name = "anti-uav";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;

    auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
    auto arm = factory.instantiateTreeNode("Arm", "Arm", node_config);
    auto takeoff = factory.instantiateTreeNode("Takeoff", "Takeoff", node_config);
    auto midcourse = factory.instantiateTreeNode("Midcourse", "Midcourse", node_config);
    auto detect = factory.instantiateTreeNode("Detect", "Detect", node_config);
    auto track = factory.instantiateTreeNode("Track", "Track", node_config);
    auto guide = factory.instantiateTreeNode("Guide", "Guide", node_config);

    auto* control = dynamic_cast<BT::ControlNode*>(root.get());
    control->addChild(arm.get());
    control->addChild(takeoff.get());
    control->addChild(midcourse.get());
    control->addChild(detect.get());
    control->addChild(track.get());
    control->addChild(guide.get());

    subtree->nodes.push_back(std::move(root));
    subtree->nodes.push_back(std::move(arm));
    subtree->nodes.push_back(std::move(takeoff));
    subtree->nodes.push_back(std::move(midcourse));
    subtree->nodes.push_back(std::move(detect));
    subtree->nodes.push_back(std::move(track));
    subtree->nodes.push_back(std::move(guide));

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
}

bool TaskExecutionEngine::start(const plane::model::TaskInfo& task, std::string& error) {
    abort();
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        suicide_context_.targetTrackId = task.targetTrackId;
        tree_ = build_anti_uav();
    } catch (const std::exception& e) {
        error = std::string("行为树创建失败: ") + e.what();
        return false;
    }
    current_tree_name_ = "anti-uav";
    tree_ready_ = true;
    current_node_.clear();
    ticking_ = true;
    tick_thread_ = std::thread([this]() { tick_loop(); });
    SPDLOG_INFO("行为树已创建: {}, taskId={}, taskCode={}", current_tree_name_, task.taskId,
                std::string(magic_enum::enum_name(task.taskCode)));
    return true;
}

void TaskExecutionEngine::abort() {
    halt_tree();
    cleanup_control();
}

void TaskExecutionEngine::halt_tree() {
    ticking_ = false;
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    bool had_tree = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        had_tree = tree_ready_;
        if (tree_ready_) {
            tree_.haltTree();
        }
        tree_ready_ = false;
        current_node_.clear();
    }
    if (had_tree) {
        SPDLOG_INFO("任务中止，行为树已停止");
    }
}

void TaskExecutionEngine::tick_loop() {
    while (ticking_) {
        BT::NodeStatus status = BT::NodeStatus::IDLE;
        bool changed = false;
        bool exception_thrown = false;
        std::string exception_text;
        std::string node;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!tree_ready_) {
                break;
            }
            try {
                status = tree_.tickOnce();
            } catch (const std::exception& e) {
                exception_thrown = true;
                exception_text = e.what();
            }
            if (!exception_thrown && !tree_.subtrees.empty() && tree_.subtrees.front()->blackboard) {
                const bool has_node = tree_.subtrees.front()->blackboard->get("current_node", node);
                if (has_node && !node.empty() && node != current_node_) {
                    current_node_ = node;
                    changed = true;
                }
            }
        }
        if (exception_thrown) {
            finish(plane::model::TaskStatus::Failed, "行为树异常: " + exception_text);
            return;
        }
        if (changed) {
            publish_progress(node);
        }
        if (status == BT::NodeStatus::SUCCESS) {
            finish(plane::model::TaskStatus::Completed, "");
            return;
        }
        if (status == BT::NodeStatus::FAILURE) {
            finish(plane::model::TaskStatus::Failed, "节点执行失败: " + node);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.bt_tick_interval_ms));
    }
}

void TaskExecutionEngine::publish_progress(const std::string& node) {
    TaskProgress progress;
    progress.currentBehaviorTree = current_tree_name_;
    progress.currentNode = node;
    event_bus_.publish(events::kExecutionProgress, progress);
    SPDLOG_INFO("任务执行进度: {} / {}", current_tree_name_, node);
}

void TaskExecutionEngine::finish(plane::model::TaskStatus status, const std::string& detail) {
    ticking_ = false;
    cleanup_control();
    TaskEngineResult result;
    result.status = status;
    result.detail = detail;
    event_bus_.publish(events::kEngineResult, result);
    SPDLOG_INFO("行为树结束: {}, status={}, detail={}", current_tree_name_, std::string(magic_enum::enum_name(status)),
                detail);
}

void TaskExecutionEngine::cleanup_control() {
    flight_control_.clear_ned_velocity();
    perception_.stop_detection();
    perception_.stop_tracking();
    perception_.stop_guidance();
}

}  // namespace plane::task
