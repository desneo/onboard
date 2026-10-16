#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/control_node.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "core/system_param_api.h"
#include "flight_control/flight_control_api.h"
#include "perception/perception_api.h"
#include "state/state_store_api.h"
#include "target/target_store_api.h"
#include "task/command_context.h"
#include "task/suicide_task_context.h"
#include "task/task_execution_engine_api.h"

namespace plane::task {

/// 任务执行引擎：taskCode → 内置 BehaviorTree，tick 执行并上报进度/结果（ARCHITECTURE §9）。
class TaskExecutionEngine final : public plane::core::IModule, public TaskExecutionEngineApi {
   public:
    TaskExecutionEngine(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::flight_control::FlightControlApi& flight_control,
                        plane::state::StateStoreApi& state_store, plane::core::SystemParamApi& system_param,
                        plane::perception::PerceptionApi& perception, plane::target::TargetStoreApi& target_store);
    ~TaskExecutionEngine() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool start(const plane::model::TaskInfo& task, std::string& error) override;
    bool start_command(const plane::model::Command& command, std::string& error) override;
    void abort() override;

   private:
    BT::Tree build_anti_uav();
    BT::Tree build_command_tree(const std::vector<std::string>& node_names);
    /// 在持锁状态下装载并启动一棵树（供任务/指令共用）。
    void launch_tree(BT::Tree tree, const std::string& tree_name, const std::string& tree_structure);
    void tick_loop();
    void publish_progress(const std::string& node);
    void finish(plane::model::TaskStatus status, const std::string& detail);
    void halt_tree();
    void cleanup_control();

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::flight_control::FlightControlApi& flight_control_;
    plane::state::StateStoreApi& state_store_;
    plane::core::SystemParamApi& system_param_;
    plane::perception::PerceptionApi& perception_;
    plane::target::TargetStoreApi& target_store_;

    std::mutex mutex_;
    BT::Tree tree_;
    bool tree_ready_ = false;
    bool is_task_tree_ = false;  ///< 当前树是否为任务树（任务才发布进度）
    std::string current_node_;
    std::string current_tree_name_;
    SuicideTaskContext suicide_context_;
    CommandContext command_context_;
    std::thread tick_thread_;
    std::atomic<bool> ticking_{false};
};

}  // namespace plane::task
