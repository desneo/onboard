#include "target/target_store.h"

#include <spdlog/spdlog.h>

#include <cstddef>
#include <string>
#include <utility>

#include "communication/communication_events.h"

namespace plane::target {
namespace {

constexpr std::size_t kGcsTargetHistoryDepth = 20;

}  // namespace

TargetStore::TargetStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config)
    : event_bus_(event_bus), config_(config) {}

TargetStore::~TargetStore() { stop(); }

std::string TargetStore::name() const { return "目标管理"; }

bool TargetStore::init() {
    SPDLOG_INFO("模块初始化完成: {}", name());
    return true;
}

bool TargetStore::start() {
    event_bus_.subscribe<GcsTarget>(plane::communication::events::kGcsTargetReceived,
                                    [this](const GcsTarget& target) { on_gcs_target(target); });
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void TargetStore::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    gcs_latest_.clear();
    gcs_history_.clear();
}

void TargetStore::release() {}

std::vector<GcsTarget> TargetStore::gcs_targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<GcsTarget> result;
    result.reserve(gcs_latest_.size());
    for (const auto& item : gcs_latest_) {
        result.push_back(item.second);
    }
    return result;
}

bool TargetStore::gcs_target(const std::string& track_id, GcsTarget& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_latest_.find(track_id);
    if (it == gcs_latest_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

std::vector<GcsTarget> TargetStore::gcs_target_history(const std::string& track_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_history_.find(track_id);
    if (it == gcs_history_.end()) {
        return {};
    }
    return {it->second.begin(), it->second.end()};
}

void TargetStore::on_gcs_target(const GcsTarget& target) {
    if (target.trackId.empty() || target.timestamp_ms <= 0) {
        SPDLOG_WARN("忽略非法 GCS 目标: trackId={} timestamp_ms={}", target.trackId, target.timestamp_ms);
        return;
    }
    std::size_t history_size = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto latest = gcs_latest_.find(target.trackId);
        if (latest != gcs_latest_.end() && target.timestamp_ms <= latest->second.timestamp_ms) {
            SPDLOG_WARN("忽略乱序或重复 GCS 目标: trackId={} timestamp_ms={} latest.timestamp_ms={}", target.trackId,
                        target.timestamp_ms, latest->second.timestamp_ms);
            return;
        }
        gcs_latest_[target.trackId] = target;
        auto& history = gcs_history_[target.trackId];
        history.push_back(target);
        if (history.size() > kGcsTargetHistoryDepth) {
            history.pop_front();
        }
        history_size = history.size();
    }
    SPDLOG_INFO("GCS 目标更新: trackId={} timestamp_ms={} history_size={}", target.trackId, target.timestamp_ms,
                history_size);
}

}  // namespace plane::target
