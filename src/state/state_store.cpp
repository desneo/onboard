#include "state/state_store.h"

#include <spdlog/spdlog.h>

#include <chrono>
#include <utility>

#include "core/time_utils.h"
#include "state/state_events.h"

namespace plane::state {

using plane::core::now_ms;

namespace {

constexpr int kTickIntervalMs = 200;

}  // namespace

StateStore::StateStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config)
    : event_bus_(event_bus), config_(config) {
    own_.planeId = config.plane_id;
}

StateStore::~StateStore() { stop(); }

std::string StateStore::name() const { return "无人机状态管理"; }

bool StateStore::init() { return true; }

bool StateStore::start() {
    running_ = true;
    tick_ = std::thread([this]() { tick_loop(); });
    return true;
}

void StateStore::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (tick_.joinable()) {
        tick_.join();
    }
}

void StateStore::release() {}

plane::model::VehicleState StateStore::own_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return own_;
}

std::vector<plane::model::VehicleState> StateStore::peer_states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<plane::model::VehicleState> result;
    result.reserve(peers_.size());
    for (const auto& item : peers_) {
        result.push_back(item.second);
    }
    return result;
}

bool StateStore::peer_state(const std::string& plane_id, plane::model::VehicleState& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = peers_.find(plane_id);
    if (it == peers_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

GcsLinkState StateStore::gcs_link() const {
    std::lock_guard<std::mutex> lock(mutex_);
    GcsLinkState link;
    link.connected = gcs_connected_;
    link.last_seen_ms = gcs_last_seen_ms_;
    return link;
}

void StateStore::update_own_state(const plane::model::VehicleState& state) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        own_ = state;
    }
    event_bus_.publish(events::kOwnStateChanged, state);
}

void StateStore::update_peer_state(const plane::model::VehicleState& state) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        peers_[state.planeId] = state;
    }
    event_bus_.publish(events::kPeerStateChanged, state);
}

void StateStore::notify_station_seen() {
    bool became_connected = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gcs_last_seen_ms_ = now_ms();
        if (!gcs_connected_) {
            gcs_connected_ = true;
            became_connected = true;
        }
    }
    if (became_connected) {
        GcsLinkState link;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            link.connected = gcs_connected_;
            link.last_seen_ms = gcs_last_seen_ms_;
        }
        SPDLOG_INFO("地面站链路状态: 已连接");
        event_bus_.publish(events::kGcsLinkChanged, link);
    }
}

void StateStore::tick_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTickIntervalMs));
        bool became_disconnected = false;
        GcsLinkState link;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (gcs_connected_ && now_ms() - gcs_last_seen_ms_ > config_.gcs_link_timeout_ms) {
                gcs_connected_ = false;
                became_disconnected = true;
            }
            link.connected = gcs_connected_;
            link.last_seen_ms = gcs_last_seen_ms_;
        }
        if (became_disconnected) {
            SPDLOG_INFO("地面站链路状态: 已断开");
            event_bus_.publish(events::kGcsLinkChanged, link);
        }
    }
}

}  // namespace plane::state
