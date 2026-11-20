#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "communication/communication_api.h"
#include "communication/zenoh_transport.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "state/state_store_api.h"
#include "task/task_state_provider.h"

namespace plane::camera {
class CameraControlApi;
}

namespace plane::communication {

/// 通信模块：Zenoh 收发、信封编解码、事件分发、Ack 与可靠重试。
class CommunicationModule final : public plane::core::IModule, public CommunicationApi {
   public:
    CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::state::StateStoreApi& state_store, plane::task::TaskStateProvider& task_state_provider,
                        plane::camera::CameraControlApi* camera = nullptr);
    ~CommunicationModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool sendToStation(const OutgoingMessage& message) override;
    bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) override;
    bool sendToAll(const OutgoingMessage& message) override;

    bool queryAgent(const std::string& agent_id, plane::model::Message& reply, std::string& error,
                    int timeout_ms = 2000) override;

   private:
    struct PendingMessage {
        std::string wire;
        std::string topic;
        std::int64_t deadline_ms = 0;
        int attempts = 0;
    };

    struct IncomingMessage {
        std::string wire;
        bool unicast;
        std::chrono::steady_clock::time_point received_at;
    };

    void enqueue_wire(std::string raw, bool unicast);
    bool enqueue_alert(const plane::model::Alert& alert);
    plane::model::AgentInfo agent_info() const;
    bool update_agent_liveliness(const std::string& key_expr, bool online);
    void on_peer_liveliness(const std::string& key_expr, bool online);
    void worker_loop();
    void on_wire_message(const IncomingMessage& incoming);
    void handle_envelope(const plane::model::Message& envelope, const IncomingMessage& incoming);
    bool remember_flight_control(const plane::model::Message& envelope);
    void handle_pod_command(const plane::model::Message& envelope, const IncomingMessage& incoming);
    void handle_ack(const plane::model::Message& envelope);
    void report_states();
    void check_pending();
    bool send_message(const std::string& topic, const OutgoingMessage& message);
    bool send_ack(const std::string& message_id);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::state::StateStoreApi& state_store_;
    plane::task::TaskStateProvider& task_state_provider_;
    plane::camera::CameraControlApi* camera_;
    std::int64_t last_report_ms_ = 0;
    ZenohTransport transport_;
    std::mutex presence_mutex_;
    bool presence_active_ = false;
    std::unordered_set<std::string> online_agents_;
    std::atomic<bool> agent_info_pending_{false};
    std::mutex rx_mutex_;
    std::condition_variable rx_cv_;
    std::deque<IncomingMessage> rx_queue_;
    // 与接收队列共用唤醒机制；初始化后接收启动早期诊断，停止后拒绝入队。
    std::deque<plane::model::Alert> alert_queue_;
    bool accepting_alerts_ = false;
    bool diagnostic_subscribed_ = false;
    std::deque<std::pair<std::string, std::string>> recent_pod_commands_;
    std::deque<std::pair<std::string, std::string>> recent_flight_controls_;
    std::mutex pending_mutex_;
    std::unordered_map<std::string, PendingMessage> pending_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace plane::communication
