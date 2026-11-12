#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "target/target_store_api.h"

namespace plane::target {

/// 目标管理：GCS 目标（最新 + 历史）（ARCHITECTURE §16）。
class TargetStore final : public plane::core::IModule, public TargetStoreApi {
   public:
    TargetStore(plane::core::EventBus& event_bus, const plane::core::AppConfig& config);
    ~TargetStore() override;

    std::string name() const override;
    bool init() override;
    bool start() override;
    void stop() override;
    void release() override;

    std::vector<plane::model::GcsTarget> gcs_targets() const override;
    bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const override;
    std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const override;

   private:
    void on_gcs_target(const plane::model::GcsTarget& target);

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, plane::model::GcsTarget> gcs_latest_;
    std::unordered_map<std::string, std::deque<plane::model::GcsTarget>> gcs_history_;
};

}  // namespace plane::target
