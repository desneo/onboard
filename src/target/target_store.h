#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/config.h"
#include "core/event_bus.h"
#include "core/module.h"
#include "target/target_store_api.h"

namespace plane::target {

/// 目标管理：GCS 最新值、历史和当前目标需求的接收监测。
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
    std::optional<std::int64_t> gcs_target_last_received_ms(const std::string& track_id) const override;
    std::uint64_t gcs_target_revision() const override;
    GcsTargetUpdates gcs_target_updates_after(const std::string& track_id,
                                              std::uint64_t after_revision) const override;
    void set_required_gcs_target(std::optional<std::string> track_id) override;
    void lock_required_gcs_target(const std::string& track_id) override;

   private:
    friend struct TargetStoreTestAccess;

    struct Reception {
        std::int64_t received_ms = 0;
        std::chrono::system_clock::time_point received_time;
        std::int64_t last_summary_ms = 0;
        std::size_t pending_updates = 0;
    };

    struct TargetHistoryItem {
        plane::model::GcsTarget target;
        std::uint64_t revision = 0;
        std::int64_t received_ms = 0;
    };

    struct RequiredTarget {
        std::string track_id;
        std::int64_t started_ms = 0;
        std::optional<std::int64_t> last_notice_ms;
    };

    struct MissingNotice {
        std::string track_id;
        std::optional<std::chrono::system_clock::time_point> received_time;
        std::optional<std::int64_t> timestamp_ms;
        std::int64_t age_ms = 0;
    };

    void on_gcs_target(const plane::model::GcsTarget& target);
    void monitor_loop();
    std::optional<std::int64_t> required_target_deadline_locked() const;
    std::optional<MissingNotice> take_missing_notice_locked(std::int64_t now);
    void report_required_target();

    plane::core::EventBus& event_bus_;
    const plane::core::AppConfig& config_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, plane::model::GcsTarget> gcs_latest_;
    std::unordered_map<std::string, std::deque<TargetHistoryItem>> gcs_history_;
    std::unordered_map<std::string, Reception> receptions_;
    std::uint64_t gcs_target_revision_ = 0;
    std::optional<RequiredTarget> required_target_;
    // 输出与取消需求串行，避免取消返回后仍输出旧需求的提示。
    std::mutex notice_mutex_;
    std::condition_variable monitor_cv_;
    std::thread monitor_thread_;
    bool running_ = false;
    bool subscribed_ = false;
};

}  // namespace plane::target
