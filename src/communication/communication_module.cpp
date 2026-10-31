#include "communication/communication_module.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstddef>
#include <string_view>
#include <utility>
#include <vector>

#include "communication/communication_events.h"
#include "communication/plane_query.h"
#include "communication/serialization.h"
#include "core/time_utils.h"

namespace plane::communication {

using plane::core::now_ms;

namespace {

constexpr int kWorkerTickMs = 50;
constexpr std::string_view kLivelinessPrefix = "plane/liveliness/";

bool is_reliable(plane::model::MessageType type) {
    return type == plane::model::MessageType::Alert || type == plane::model::MessageType::Ack;
}

}  // namespace

CommunicationModule::CommunicationModule(plane::core::EventBus& event_bus, const plane::core::AppConfig& config,
                                         plane::state::StateStoreApi& state_store,
                                         plane::task::TaskStateProvider& task_state_provider)
    : event_bus_(event_bus), config_(config), state_store_(state_store), task_state_provider_(task_state_provider) {}

CommunicationModule::~CommunicationModule() { stop(); }

std::string CommunicationModule::name() const { return "通信"; }

bool CommunicationModule::init() {
    SPDLOG_INFO("模块初始化完成: {}", name());
    return true;
}

bool CommunicationModule::start() {
    ZenohEndpoints endpoints;
    endpoints.config_file = config_.zenoh_config_file;
    endpoints.connect = config_.zenoh_connect;
    endpoints.listen = config_.zenoh_listen;
    endpoints.debug_log = config_.debug_zenoh_log;

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

    {
        std::lock_guard<std::mutex> lock(presence_mutex_);
        presence_active_ = true;
        station_online_ = false;
        vehicle_info_pending_ = true;
    }
    if (!transport_.subscribe_liveliness(
            std::string(kLivelinessPrefix) + "*",
            [this](const std::string& key_expr, bool online) { on_peer_liveliness(key_expr, online); }, error) ||
        !transport_.declare_liveliness_token(std::string(kLivelinessPrefix) + config_.plane_id, error)) {
        report_error("飞机存活性声明失败: " + error);
        return false;
    }

    if (!transport_.subscribe_liveliness(
            station_liveliness_key(config_.station_id),
            [this](const std::string&, bool online) { on_station_liveliness(online); }, error) ||
        !transport_.declare_queryable(
            plane_query_selector(config_.plane_id),
            [this](const std::string& key, std::string& reply, std::string& error) {
                if (key != plane_query_key(config_.plane_id, plane::model::MessageType::VehicleInfo)) {
                    error = "不支持的查询类型，当前仅支持 VehicleInfo";
                    return false;
                }
                plane::model::Message message;
                if (!make_message(config_.plane_id, vehicle_info(), message, error)) return false;
                reply = encode_envelope(message);
                return true;
            },
            error)) {
        report_error("地面站存活性或查询接口声明失败: " + error);
        return false;
    }

    running_ = true;
    worker_ = std::thread([this]() { worker_loop(); });
    SPDLOG_INFO("通信模块已启动: plane_id={} station_id={}", config_.plane_id, config_.station_id);
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void CommunicationModule::stop() {
    {
        std::lock_guard<std::mutex> lock(presence_mutex_);
        presence_active_ = false;
        station_online_ = false;
        vehicle_info_pending_ = false;
        online_peers_.clear();
    }
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

bool CommunicationModule::queryPlane(const std::string& plane_id, plane::model::MessageType type,
                                     plane::model::Message& reply, std::string& error, int timeout_ms) {
    return plane::communication::queryPlane(transport_, plane_id, type, reply, error, timeout_ms);
}

plane::model::VehicleInfo CommunicationModule::vehicle_info() const {
    return {config_.plane_id, config_.board_ip, config_.pod_video_url, config_.is_sim};
}

void CommunicationModule::on_station_liveliness(bool online) {
    std::lock_guard<std::mutex> lock(presence_mutex_);
    if (!presence_active_ || station_online_ == online) return;
    station_online_ = online;
    if (online) {
        vehicle_info_pending_ = true;
        rx_cv_.notify_one();
    }
}

void CommunicationModule::enqueue_wire(std::string raw) {
    {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        const auto limit = static_cast<std::size_t>(config_.comm_rx_queue_max);
        if (rx_queue_.size() >= limit) {
            rx_queue_.pop_front();
            SPDLOG_WARN("通信接收队列已满，丢弃最旧消息");
        }
        rx_queue_.push_back(std::move(raw));
    }
    rx_cv_.notify_one();
}

void CommunicationModule::on_peer_liveliness(const std::string& key_expr, bool online) {
    if (key_expr.compare(0, kLivelinessPrefix.size(), kLivelinessPrefix) != 0) {
        return;
    }
    const std::string plane_id = key_expr.substr(kLivelinessPrefix.size());
    if (plane_id.empty() || plane_id == config_.plane_id) {
        return;
    }
    std::lock_guard<std::mutex> lock(presence_mutex_);
    if (!presence_active_) {
        return;
    }
    if (online) {
        if (online_peers_.insert(plane_id).second) {
            SPDLOG_INFO("他机上线: {}", plane_id);
        }
    } else if (online_peers_.erase(plane_id) != 0) {
        SPDLOG_INFO("他机下线: {}", plane_id);
    }
}

void CommunicationModule::worker_loop() {
    while (running_) {
        std::deque<std::string> batch;
        {
            std::unique_lock<std::mutex> lock(rx_mutex_);
            rx_cv_.wait_for(lock, std::chrono::milliseconds(kWorkerTickMs),
                            [this]() { return !rx_queue_.empty() || vehicle_info_pending_ || !running_; });
            batch.swap(rx_queue_);
        }
        for (const auto& raw : batch) {
            on_wire_message(raw);
        }
        if (vehicle_info_pending_.exchange(false)) {
            if (sendToStation(vehicle_info())) {
                SPDLOG_INFO("已上报飞机信息: plane_id={}", config_.plane_id);
            }
        }
        report_states();
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
    if (envelope.senderId == config_.station_id) {
        state_store_.notify_station_seen();
    }
    if ((envelope.type == plane::model::MessageType::TaskInfo || envelope.type == plane::model::MessageType::Command) &&
        envelope.senderId != config_.station_id) {
        SPDLOG_WARN("拒绝非地面站控制消息: senderId={}", envelope.senderId);
        return;
    }
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
            plane::target::GcsTarget value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("GcsTarget 解码失败: " + error);
                return;
            }
            event_bus_.publish(events::kGcsTargetReceived, value);
            break;
        }
        case plane::model::MessageType::VehicleState: {
            if (envelope.senderId == config_.station_id) {
                break;
            }
            plane::model::VehicleState value;
            if (!from_payload(envelope.payload, value, error)) {
                report_error("VehicleState 解码失败: " + error);
                return;
            }
            state_store_.update_peer_state(value);
            break;
        }
        case plane::model::MessageType::Ack: {
            handle_ack(envelope);
            break;
        }
        default: {
            SPDLOG_WARN("忽略未处理的消息类型: {}", message_type_to_string(envelope.type));
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
        SPDLOG_DEBUG("收到确认消息: ackId={}", ack.ackId);
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
    // 发往地面站的消息一律尽力发送，不做可靠重试；其余链路按消息类型决定。
    const bool reliable = topic != std::string(plane::model::topic_sendToStation) && is_reliable(envelope.type);
    if (reliable) {
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

void CommunicationModule::report_states() {
    const std::int64_t now = now_ms();
    if (now - last_report_ms_ < config_.state_report_period_ms) {
        return;
    }
    last_report_ms_ = now;
    sendToAll(state_store_.own_state());
    sendToStation(task_state_provider_.current_task_state());
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
            SPDLOG_WARN("可靠消息重试: topic={} attempts={}", pending.topic, pending.attempts);
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

}  // namespace plane::communication
