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

std::string TargetStore::name() const { return "Target Management"; }

bool TargetStore::init() { return true; }

bool TargetStore::start() {
    event_bus_.subscribe<plane::model::GcsTarget>(
        plane::communication::events::kGcsTargetReceived,
        [this](const plane::model::GcsTarget& target) { on_gcs_target(target); });
    return true;
}

void TargetStore::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    gcs_latest_.clear();
    gcs_history_.clear();
}

void TargetStore::release() {}

std::vector<plane::model::GcsTarget> TargetStore::gcs_targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<plane::model::GcsTarget> result;
    result.reserve(gcs_latest_.size());
    for (const auto& item : gcs_latest_) {
        result.push_back(item.second);
    }
    return result;
}

bool TargetStore::gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_latest_.find(track_id);
    if (it == gcs_latest_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

std::vector<plane::model::GcsTarget> TargetStore::gcs_target_history(const std::string& track_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = gcs_history_.find(track_id);
    if (it == gcs_history_.end()) {
        return {};
    }
    return {it->second.begin(), it->second.end()};
}

void TargetStore::on_gcs_target(const plane::model::GcsTarget& target) {
    std::size_t history_size = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gcs_latest_[target.trackId] = target;
        auto& history = gcs_history_[target.trackId];
        history.push_back(target);
        if (history.size() > kGcsTargetHistoryDepth) {
            history.pop_front();
        }
        history_size = history.size();
    }
    SPDLOG_INFO("GCS 目标更新: trackId={}, 历史={}", target.trackId, history_size);
}

}  // namespace plane::target
