#include "task/task_execution_engine.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <magic_enum/magic_enum.hpp>
#include <string>
#include <utility>
#include <vector>

#include "task/command_tree_plan.h"
#include "task/nodes/adjust_height_node.h"
#include "task/nodes/arm_node.h"
#include "task/nodes/disarm_node.h"
#include "task/nodes/hover_node.h"
#include "task/nodes/land_node.h"
#include "task/nodes/midcourse_node.h"
#include "task/nodes/mission_node.h"
#include "task/nodes/rtl_node.h"
#include "task/nodes/stop_motor_node.h"
#include "task/nodes/takeoff_node.h"
#include "task/task_events.h"

namespace plane::task {
namespace {

std::string sequence_structure(const std::vector<std::string>& node_names) {
    std::string structure = "顺序执行[";
    for (std::size_t index = 0; index < node_names.size(); ++index) {
        if (index > 0) {
            structure += " -> ";
        }
        structure += node_names[index];
    }
    structure += "]";
    return structure;
}

}  // namespace

TaskExecutionEngine::TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::flight_control::FlightControlApi& flight_control,
                                         plane::state::StateStoreApi& state_store,
                                         plane::core::SystemParamApi& system_param,
                                         plane::target::TargetStoreApi& target_store)
    : event_bus_(event_bus),
      config_(config),
      flight_control_(flight_control),
      state_store_(state_store),
      system_param_(system_param),
      target_store_(target_store),
      midcourse_controller_(event_bus, config, flight_control, state_store, target_store) {}

TaskExecutionEngine::~TaskExecutionEngine() { halt_tree(); }

std::string TaskExecutionEngine::name() const { return "任务执行引擎"; }

bool TaskExecutionEngine::init() { return true; }

bool TaskExecutionEngine::start() {
    midcourse_controller_.start();
    return true;
}

void TaskExecutionEngine::stop() { abort(); }

void TaskExecutionEngine::release() {}

void TaskExecutionEngine::launch_tree(BT::Tree tree, const std::string& tree_name, const std::string& tree_structure) {
    tree_ = std::move(tree);
    current_tree_name_ = tree_name;
    tree_ready_ = true;
    current_node_.clear();
    SPDLOG_INFO("行为树结构: currentBehaviorTree={} tree_structure={}", current_tree_name_, tree_structure);
    ticking_ = true;
    tick_thread_ = std::thread([this]() { tick_loop(); });
}

bool TaskExecutionEngine::start(const plane::model::TaskInfo& task, std::string& error) {
    abort();
    if (task.taskCode != plane::model::TaskCode::AntiUav) {
        error = "不支持的任务类型";
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    task_context_.targetTrackId = task.targetTrackId;
    command_context_.takeoffHeight_m = static_cast<float>(system_param_.current().safeHeight_m);

    const plane::flight_control::FlightStatus flight_status = flight_control_.flight_status();
    const std::vector<std::string> node_names = anti_uav_node_names(flight_status);

    BT::Tree tree;
    try {
        tree = build_task_tree(node_names);
    } catch (const std::exception& e) {
        error = std::string("AntiUav 行为树创建失败: ") + e.what();
        return false;
    }
    is_task_tree_ = true;
    launch_tree(std::move(tree), "AntiUav", sequence_structure(node_names));
    SPDLOG_INFO("AntiUav 中制导任务已创建: taskId={} targetTrackId={}", task.taskId,
                task.targetTrackId.empty() ? "<自动>" : task.targetTrackId);
    return true;
}

bool TaskExecutionEngine::start_command(const plane::model::Command& command, std::string& error) {
    abort();
    std::lock_guard<std::mutex> lock(mutex_);
    command_context_.commandId = command.commandId;
    command_context_.type = command.type;
    command_context_.waypoints = command.waypoints;
    float height = command.type == plane::model::CommandType::Takeoff ? command.takeoffHeight_m : 0.0F;
    if (height <= 0.0F) {
        height = static_cast<float>(system_param_.current().safeHeight_m);
        SPDLOG_INFO("起飞指令未指定高度，使用安全高度: {} 米", height);
    }
    command_context_.takeoffHeight_m = height;

    const plane::flight_control::FlightStatus flight_status = flight_control_.flight_status();
    const std::vector<std::string> node_names = command_node_names(command_context_.type, flight_status);
    BT::Tree tree;
    try {
        tree = build_command_tree(node_names);
    } catch (const std::exception& e) {
        error = std::string("指令行为树创建失败: ") + e.what();
        return false;
    }
    is_task_tree_ = false;
    launch_tree(std::move(tree), std::string("cmd-") + std::string(magic_enum::enum_name(command.type)),
                sequence_structure(node_names));
    SPDLOG_INFO("指令行为树已创建: currentBehaviorTree={} commandId={}", current_tree_name_, command.commandId);
    return true;
}

BT::Tree TaskExecutionEngine::build_command_tree(const std::vector<std::string>& node_names) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<DisarmNode>("Disarm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_, &system_param_,
                                          &command_context_);
    factory.registerNodeType<LandNode>("Land", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<RtlNode>("RTL", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<HoverNode>("Hover", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<StopMotorNode>("StopMotor", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<AdjustHeightNode>("AdjustHeight", &flight_control_, &state_store_, &config_,
                                               &command_context_);
    factory.registerNodeType<MissionNode>("Mission", &flight_control_, &state_store_, &config_, &command_context_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "command";
    subtree->instance_name = "command";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;

    // 统一以 Sequence 为根，保证根为控制节点；单指令仅含 1 个动作节点。
    auto make_sequence = [&](const std::vector<std::string>& node_names) {
        auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
        auto* control = dynamic_cast<BT::ControlNode*>(root.get());
        subtree->nodes.push_back(std::move(root));
        for (const std::string& node_name : node_names) {
            auto node = factory.instantiateTreeNode(node_name, node_name, node_config);
            control->addChild(node.get());
            subtree->nodes.push_back(std::move(node));
        }
    };

    make_sequence(node_names);

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
}

BT::Tree TaskExecutionEngine::build_task_tree(const std::vector<std::string>& node_names) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ArmNode>("Arm", &flight_control_, &state_store_, &config_);
    factory.registerNodeType<TakeoffNode>("Takeoff", &flight_control_, &state_store_, &config_, &system_param_,
                                          &command_context_);
    factory.registerNodeType<MidcourseNode>("Midcourse", &flight_control_, &state_store_, &config_, &task_context_,
                                            &midcourse_controller_);

    auto subtree = std::make_shared<BT::Tree::Subtree>();
    subtree->blackboard = BT::Blackboard::create();
    subtree->tree_ID = "anti-uav";
    subtree->instance_name = "anti-uav";

    BT::NodeConfig node_config;
    node_config.blackboard = subtree->blackboard;
    auto root = factory.instantiateTreeNode("Sequence", "Sequence", node_config);
    auto* control = dynamic_cast<BT::ControlNode*>(root.get());
    subtree->nodes.push_back(std::move(root));
    for (const std::string& node_name : node_names) {
        auto node = factory.instantiateTreeNode(node_name, node_name, node_config);
        control->addChild(node.get());
        subtree->nodes.push_back(std::move(node));
    }

    BT::Tree tree;
    tree.subtrees.push_back(subtree);
    tree.initialize();
    return tree;
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
        if (changed && is_task_tree_) {
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
    SPDLOG_INFO("任务执行进度: currentBehaviorTree={} currentNode={}", current_tree_name_, node);
}

void TaskExecutionEngine::finish(plane::model::TaskStatus status, const std::string& detail) {
    ticking_ = false;
    cleanup_control();
    TaskEngineResult result;
    result.status = status;
    result.detail = detail;
    event_bus_.publish(events::kEngineResult, result);
    SPDLOG_INFO("行为树结束: currentBehaviorTree={} status={} detail={}", current_tree_name_,
                std::string(magic_enum::enum_name(status)), detail);
}

void TaskExecutionEngine::cleanup_control() {
    midcourse_controller_.stop();
    flight_control_.clear_offboard();
}

}  // namespace plane::task
