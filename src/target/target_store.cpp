#include "target/target_store.h"

#include <spdlog/spdlog.h>

#include <cstddef>
#include <string>
#include <utility>

#include "communication/communication_events.h"
#include "perception/perception_events.h"

namespace plane::target {
namespace {

constexpr std::size_t kGcsTargetHistoryDepth = 20;

}  // namespace

TargetStore::TargetStore(plane::core::EventBus& event_bus) : event_bus_(event_bus) {}

TargetStore::~TargetStore() { stop(); }

std::string TargetStore::name() const { return "Target Management"; }

bool TargetStore::init() { return true; }

bool TargetStore::start() {
    event_bus_.subscribe<plane::model::GcsTarget>(
        plane::communication::events::kGcsTargetReceived,
        [this](const plane::model::GcsTarget& target) { on_gcs_target(target); });
    event_bus_.subscribe<plane::perception::DetectionResult>(
        plane::perception::events::kDetection,
        [this](const plane::perception::DetectionResult& result) { on_detection(result); });
    event_bus_.subscribe<plane::perception::TrackingResult>(
        plane::perception::events::kTracking,
        [this](const plane::perception::TrackingResult& result) { on_tracking(result); });
    return true;
}

void TargetStore::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    vision_targets_.clear();
    tracked_target_.reset();
    gcs_latest_.clear();
    gcs_history_.clear();
}

void TargetStore::release() {}

std::vector<plane::model::DetectTarget> TargetStore::vision_targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vision_targets_;
}

bool TargetStore::tracked_target(plane::model::DetectTarget& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!tracked_target_.has_value()) {
        return false;
    }
    out = *tracked_target_;
    return true;
}

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
    spdlog::info("GCS 目标更新: trackId={}, 历史={}", target.trackId, history_size);
}

void TargetStore::on_detection(const plane::perception::DetectionResult& result) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vision_targets_ = result.targets;
    }
    spdlog::info("视觉目标更新: {} 个", result.targets.size());
}

void TargetStore::on_tracking(const plane::perception::TrackingResult& result) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tracked_target_ = result.target;
    }
    spdlog::info("跟踪目标更新: type={}, lost={}", result.target.type, result.lost ? 1 : 0);
}

}  // namespace plane::target
