#include "task/task_execution_engine.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <magic_enum/magic_enum.hpp>

#include "task/nodes/arm_node.h"
#include "task/nodes/hover_node.h"
#include "task/nodes/land_node.h"
#include "task/nodes/rtl_node.h"
#include "task/nodes/stop_motor_node.h"
#include "task/nodes/takeoff_node.h"
#include "task/task_events.h"

namespace plane::task {

TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::core::SystemParamApi& system_param)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store),
      system_param_(system_param) {}

TaskExecutionEngine::~TaskExecutionEngine() { stop(); }

std::string TaskExecutionEngine::name() const { return "Task Execution Engine"; }

bool TaskExecutionEngine::init() { return true; }

bool TaskExecutionEngine::start() { return true; }

void TaskExecutionEngine::stop() { abort(); }

void TaskExecutionEngine::release() {}

BT::Tree TaskExecutionEngine::build_basic_flight() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_, &system_param_);
    factory.registerNodeType<LandNode>("Land", &flight_control_, &state_store_, &config_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "basic_flight";
    subtree->instance_name = "basic_flight";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;

    auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
    auto arm = factory.instantiateTreeNode("Arm", "Arm", node_config);
    auto takeoff = factory.instantiateTreeNode("Takeoff", "Takeoff", node_config);
    auto land = factory.instantiateTreeNode("Land", "Land", node_config);

    auto* control = dynamic_cast<BT::ControlNode*>(root.get());
    control->addChild(arm.get());
    control->addChild(takeoff.get());
    control->addChild(land.get());

    subtree->nodes.push_back(std::move(root));
    subtree->nodes.push_back(std::move(arm));
    subtree->nodes.push_back(std::move(takeoff));
    subtree->nodes.push_back(std::move(land));

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
}

bool TaskExecutionEngine::start(const plane::model::TaskInfo& task, std::string& error) {
    if (task.taskCode != "basic_flight") {
        error = "未知任务类型: " + task.taskCode;
        return false;
    }
    abort();
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        tree_ = build_basic_flight();
    } catch (const std::exception& e) {
        error = std::string("行为树创建失败: ") + e.what();
        return false;
    }
    tree_ready_ = true;
    current_node_.clear();
    ticking_ = true;
    tick_thread_ = std::thread([this]() { tick_loop(); });
    spdlog::info("行为树已创建: basic_flight, taskId={}", task.taskId);
    return true;
}

void TaskExecutionEngine::abort() {
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
        spdlog::info("任务中止，行为树已停止");
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
    progress.currentBehaviorTree = "basic_flight";
    progress.currentNode = node;
    event_bus_.publish(events::kExecutionProgress, progress);
    spdlog::info("任务执行进度: basic_flight / {}", node);
}

void TaskExecutionEngine::finish(plane::model::TaskStatus status, const std::string& detail) {
    ticking_ = false;
    TaskEngineResult result;
    result.status = status;
    result.detail = detail;
    event_bus_.publish(events::kEngineResult, result);
    spdlog::info("行为树结束: status={}, detail={}", std::string(magic_enum::enum_name(status)), detail);
}

}  // namespace plane::task
