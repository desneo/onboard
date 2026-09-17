#include "communication/communication_module.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstddef>
#include <utility>
#include <vector>

#include "communication/communication_events.h"
#include "communication/serialization.h"

namespace plane::communication {
namespace {

constexpr int kWorkerTickMs = 50;

bool is_reliable(plane::model::MessageType type) {
    return type == plane::model::MessageType::Alert || type == plane::model::MessageType::Ack;
}

}  // namespace

CommunicationModule::CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config)
    : event_bus_(event_bus), config_(config) {}

CommunicationModule::~CommunicationModule() { stop(); }

std::string CommunicationModule::name() const { return "Communication"; }

bool CommunicationModule::init() { return true; }

bool CommunicationModule::start() {
    ZenohEndpoints endpoints;
    endpoints.config_file = config_.zenoh_config_file;
    endpoints.connect = config_.zenoh_connect;
    endpoints.listen = config_.zenoh_listen;

    std::string error;
    if (!transport_.open(endpoints, error)) {
        report_error("Zenoh 打开失败: " + error);
        return false;
    }
    const auto rx_handler = [this](const std::string& payload) { enqueue_wire(payload); };
    if (!transport_.subscribe("sendToPlane/" + config_.plane_id, rx_handler, error)) {
        report_error("订阅 sendToPlane 失败: " + error);
        return false;
    }
    if (!transport_.subscribe(std::string(plane::model::topic_sendToAll), rx_handler, error)) {
        report_error("订阅 sendToAll 失败: " + error);
        return false;
    }

    running_ = true;
    worker_ = std::thread([this]() { worker_loop(); });
    spdlog::info("Communication 已启动: plane_id={}, station_id={}", config_.plane_id, config_.station_id);
    return true;
}

void CommunicationModule::stop() {
    const bool was_running = running_.exchange(false);
    if (was_running) {
        rx_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }
    transport_.close();
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.clear();
}

void CommunicationModule::release() {}

bool CommunicationModule::sendToStation(const OutgoingMessage& message) {
    return send_message(std::string(plane::model::topic_sendToStation), message);
}

bool CommunicationModule::sendToPlane(const std::string& plane_id, const OutgoingMessage& message) {
    return send_message("sendToPlane/" + plane_id, message);
}

bool CommunicationModule::sendToAll(const OutgoingMessage& message) {
    return send_message(std::string(plane::model::topic_sendToAll), message);
}

void CommunicationModule::enqueue_wire(std::string raw) {
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        const auto limit = static_cast<std::size_t>(config_.comm_rx_queue_max);
        if (rx_queue_.size() >= limit) {
            rx_queue_.pop_front();
            spdlog::warn("通信接收队列已满，丢弃最旧消息");
        }
        rx_queue_.push_back(std::move(raw));
    }
    rx_cv_.notify_one();
}

void CommunicationModule::worker_loop() {
    while (running_) {
        std::deque<std::string> batch;
        {
            std::unique_lock<std::mutex> lock(rx_mutex_);
            rx_cv_.wait_for(lock, std::chrono::milliseconds(kWorkerTickMs),
                            [this]() { return !rx_queue_.empty() || !running_; });
            batch.swap(rx_queue_);
        }
        for (const auto& raw : batch) {
            on_wire_message(raw);
        }
        check_pending();
    }
}

void CommunicationModule::on_wire_message(const std::string& raw) {
    plane::model::Message envelope;
    std::string error;
    if (!decode_envelope(raw, envelope, error)) {
        report_error("报文解析失败: " + error);
        return;
    }
    if (envelope.senderId == config_.plane_id) {
        return;
    }
    handle_envelope(envelope);
}

void CommunicationModule::handle_envelope(const plane::model::Message& envelope) {
    std::string error;
    switch (envelope.type) {
        case plane::model::MessageType::TaskInfo: {
            plane::model::TaskInfo value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("TaskInfo 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kTaskInfoReceived, value);
            send_ack(envelope);
            break;
        }
        case plane::model::MessageType::Command: {
            plane::model::Command value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("Command 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kCommandReceived, value);
            send_ack(envelope);
            break;
        }
        case plane::model::MessageType::SystemParam: {
            plane::model::SystemParam value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("SystemParam 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kSystemParamReceived, value);
            break;
        }
        case plane::model::MessageType::GcsTarget: {
            plane::model::GcsTarget value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("GcsTarget 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kGcsTargetReceived, value);
            break;
        }
        case plane::model::MessageType::Ack: {
            handle_ack(envelope);
            break;
        }
        default: {
            spdlog::warn("忽略未处理的消息类型: {}", message_type_to_string(envelope.type));
            break;
        }
    }
}

void CommunicationModule::handle_ack(const plane::model::Message& envelope) {
    plane::model::Ack ack;
    std::string error;
    if (!from_payload(envelope.payload, ack, error)) {
        report_error("Ack 解码失败: " + error);
        return;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.erase(ack.ackId) > 0) {
        spdlog::debug("收到 Ack: {}", ack.ackId);
    }
}

bool CommunicationModule::send_message(const std::string& topic, const OutgoingMessage& message) {
    plane::model::Message envelope;
    std::string error;
    if (!make_message(config_.plane_id, message, envelope, error)) {
        report_error("消息编码失败: " + error);
        return false;
    }
    const std::string wire = encode_envelope(envelope);
    if (!transport_.put(topic, wire, error)) {
        report_error("发送失败: " + error);
        return false;
    }
    if (is_reliable(envelope.type)) {
        PendingMessage pending;
        pending.wire = wire;
        pending.topic = topic;
        pending.deadline_ms = now_ms() + config_.comm_ack_timeout_ms;
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_[envelope.messageId] = std::move(pending);
    }
    return true;
}

bool CommunicationModule::send_ack(const plane::model::Message& envelope) {
    plane::model::Ack ack;
    ack.ackId = envelope.messageId;
    const std::string topic = envelope.senderId == config_.station_id ? std::string(plane::model::topic_sendToStation)
                                                                      : "sendToPlane/" + envelope.senderId;
    return send_message(topic, OutgoingMessage{ack});
}

void CommunicationModule::check_pending() {
    const std::int64_t now = now_ms();
    std::vector<PendingMessage> resend;
    std::vector<std::string> expired;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto it = pending_.begin(); it != pending_.end();) {
            if (now < it->second.deadline_ms) {
                ++it;
                continue;
            }
            if (it->second.attempts < config_.comm_ack_max_retries) {
                it->second.attempts += 1;
                it->second.deadline_ms = now + config_.comm_ack_timeout_ms;
                resend.push_back(it->second);
                ++it;
            } else {
                expired.push_back(it->first + " -> " + it->second.topic);
                it = pending_.erase(it);
            }
        }
    }
    for (const auto& pending : resend) {
        std::string error;
        if (transport_.put(pending.topic, pending.wire, error)) {
            spdlog::warn("可靠消息重试: topic={}, attempts={}", pending.topic, pending.attempts);
        } else {
            report_error("重试发送失败: " + error);
        }
    }
    for (const auto& item : expired) {
        report_error("可靠消息确认超时: " + item);
    }
}

void CommunicationModule::report_error(const std::string& detail) const {
    CommunicationError event;
    event.detail = detail;
    event_bus_.publish(events::kError, event);
}

std::int64_t CommunicationModule::now_ms() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace plane::communication
