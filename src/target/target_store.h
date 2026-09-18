#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/event_bus.h"
#include "core/module.h"
#include "perception/perception_api.h"
#include "target/target_store_api.h"

namespace plane::target {

/// 目标管理：机载感知目标（最新）与 GCS 目标（最新 + 历史）（ARCHITECTURE §16）。
class TargetStore final : public plane::core::IModule, public TargetStoreApi {
   public:
    explicit TargetStore(plane::core::EventBus& event_bus);
    ~TargetStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    std::vector<plane::model::DetectTarget> vision_targets() const override;
    bool tracked_target(plane::model::DetectTarget& out) const override;
    TrackingState tracking_state() const override;
    std::vector<plane::model::GcsTarget> gcs_targets() const override;
    bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const override;
    std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const override;

   private:
    void on_gcs_target(const plane::model::GcsTarget& target);
    void on_detection(const plane::perception::DetectionResult& result);
    void on_tracking(const plane::perception::TrackingResult& result);

    plane::core::EventBus& event_bus_;
    mutable std::mutex mutex_;
    std::vector<plane::model::DetectTarget> vision_targets_;
    std::optional<plane::model::DetectTarget> tracked_target_;
    bool tracking_valid_ = false;
    bool tracking_lost_ = false;
    std::unordered_map<std::string, plane::model::GcsTarget> gcs_latest_;
    std::unordered_map<std::string, std::deque<plane::model::GcsTarget>> gcs_history_;
};

}  // namespace plane::target
