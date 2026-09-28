#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "communication/communication_api.h"
#include "communication/zenoh_transport.h"
#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "state/state_store_api.h"
#include "task/task_state_provider.h"

namespace plane::communication {

/// 通信模块：Zenoh 收发、信封编解码、事件分发、Ack 与可靠重试。
class CommunicationModule final : public plane::core::IModule, public CommunicationApi {
   public:
    CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                        plane::state::StateStoreApi& state_store, plane::task::TaskStateProvider& task_state_provider);
    ~CommunicationModule() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    bool sendToStation(const OutgoingMessage& message) override;
    bool sendToPlane(const std::string& plane_id, const OutgoingMessage& message) override;
    bool sendToAll(const OutgoingMessage& message) override;

   private:
    struct PendingMessage {
        std::string wire;
        std::string topic;
        std::int64_t deadline_ms = 0;
        int attempts = 0;
    };

    void enqueue_wire(std::string raw);
    void worker_loop();
    void on_wire_message(const std::string& raw);
    void handle_envelope(const plane::model::Message& envelope);
    void handle_ack(const plane::model::Message& envelope);
    void report_states();
    void check_pending();
    bool send_message(const std::string& topic, const OutgoingMessage& message);
    bool send_ack(const plane::model::Message& envelope);
    void report_error(const std::string& detail) const;

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    plane::state::StateStoreApi& state_store_;
    plane::task::TaskStateProvider& task_state_provider_;
    std::int64_t last_report_ms_ = 0;
    ZenohTransport transport_;
    std::mutex rx_mutex_;
    std::condition_variable rx_cv_;
    std::deque<std::string> rx_queue_;
    std::mutex pending_mutex_;
    std::unordered_map<std::string, PendingMessage> pending_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

}  // namespace plane::communication
