#include "target/target_store.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <string>
#include <utility>

#include "communication/communication_events.h"
#include "core/time_utils.h"

namespace plane::target {
namespace {

constexpr std::size_t kGcsTargetHistoryDepth = 20;
constexpr std::int64_t kTargetSummaryIntervalMs = 20000;
constexpr std::int64_t kMissingTargetIntervalMs = 2000;

std::string reception_time_text(std::chrono::system_clock::time_point received_time) {
    const auto unix_ms = std::chrono::duration_cast<std::chrono::milliseconds>(received_time.time_since_epoch());
    const std::time_t seconds = std::chrono::system_clock::to_time_t(received_time);
    std::tm local_time{};
    localtime_r(&seconds, &local_time);
    char date[32]{};
    char offset[8]{};
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local_time);
    std::strftime(offset, sizeof(offset), "%z", &local_time);
    return fmt::format("{}.{:03d}{}", date, unix_ms.count() % 1000, offset);
}

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
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return true;
    if (!subscribed_) {
        event_bus_.subscribe<plane::model::GcsTarget>(
            plane::communication::events::kGcsTargetReceived,
            [this](const plane::model::GcsTarget& target) { on_gcs_target(target); });
        subscribed_ = true;
    }
    running_ = true;
    monitor_thread_ = std::thread([this]() { monitor_loop(); });
    SPDLOG_INFO("模块启动完成: {}", name());
    return true;
}

void TargetStore::stop() {
    {
        std::lock_guard<std::mutex> notice_lock(notice_mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        required_target_.reset();
    }
    monitor_cv_.notify_all();
    if (monitor_thread_.joinable()) monitor_thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    gcs_latest_.clear();
    gcs_history_.clear();
    receptions_.clear();
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
    std::vector<plane::model::GcsTarget> result;
    result.reserve(it->second.size());
    for (const auto& item : it->second) {
        result.push_back(item.target);
    }
    return result;
}

std::optional<std::int64_t> TargetStore::gcs_target_last_received_ms(const std::string& track_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto reception = receptions_.find(track_id);
    if (reception == receptions_.end()) return std::nullopt;
    return reception->second.received_ms;
}

std::uint64_t TargetStore::gcs_target_revision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gcs_target_revision_;
}

GcsTargetUpdates TargetStore::gcs_target_updates_after(const std::string& track_id,
                                                       std::uint64_t after_revision) const {
    std::lock_guard<std::mutex> lock(mutex_);
    GcsTargetUpdates result;
    const auto history = gcs_history_.find(track_id);
    if (history == gcs_history_.end()) {
        return result;
    }
    result.history.reserve(history->second.size());
    for (const auto& item : history->second) {
        if (item.revision <= after_revision) {
            continue;
        }
        result.history.push_back(item.target);
        result.last_received_ms = item.received_ms;
    }
    return result;
}

void TargetStore::set_required_gcs_target(std::optional<std::string> track_id) {
    {
        std::lock_guard<std::mutex> notice_lock(notice_mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!track_id) {
            required_target_.reset();
        } else if (running_ && (!required_target_ || required_target_->track_id != *track_id)) {
            required_target_ = RequiredTarget{std::move(*track_id), plane::core::now_ms(), std::nullopt};
        }
    }
    monitor_cv_.notify_all();
}

void TargetStore::lock_required_gcs_target(const std::string& track_id) {
    {
        std::lock_guard<std::mutex> notice_lock(notice_mutex_);
        std::lock_guard<std::mutex> lock(mutex_);
        if (required_target_ && required_target_->track_id.empty() && !track_id.empty()) {
            required_target_->track_id = track_id;
        }
    }
    monitor_cv_.notify_all();
}

std::optional<std::int64_t> TargetStore::required_target_deadline_locked() const {
    if (!required_target_) return std::nullopt;
    std::string track_id = required_target_->track_id;
    if (track_id.empty()) {
        if (gcs_latest_.size() > 1) return std::nullopt;
        if (!gcs_latest_.empty()) track_id = gcs_latest_.begin()->first;
    }
    std::int64_t deadline = required_target_->started_ms + kMissingTargetIntervalMs;
    const auto reception = receptions_.find(track_id);
    if (reception != receptions_.end()) {
        deadline = std::max(deadline, reception->second.received_ms + kMissingTargetIntervalMs);
    }
    if (required_target_->last_notice_ms) {
        deadline = std::max(deadline, *required_target_->last_notice_ms + kMissingTargetIntervalMs);
    }
    return deadline;
}

std::optional<TargetStore::MissingNotice> TargetStore::take_missing_notice_locked(std::int64_t now) {
    const auto deadline = required_target_deadline_locked();
    if (!running_ || !deadline || now < *deadline) return std::nullopt;
    MissingNotice notice;
    notice.track_id = required_target_->track_id;
    if (notice.track_id.empty() && !gcs_latest_.empty()) notice.track_id = gcs_latest_.begin()->first;
    const auto reception = receptions_.find(notice.track_id);
    if (reception == receptions_.end()) {
        notice.age_ms = now - required_target_->started_ms;
    } else {
        notice.received_time = reception->second.received_time;
        notice.timestamp_ms = gcs_latest_.at(notice.track_id).timestamp_ms;
        notice.age_ms = now - reception->second.received_ms;
    }
    if (notice.track_id.empty()) notice.track_id = "<自动>";
    required_target_->last_notice_ms = now;
    return notice;
}

void TargetStore::report_required_target() {
    std::lock_guard<std::mutex> notice_lock(notice_mutex_);
    std::optional<MissingNotice> notice;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        notice = take_missing_notice_locked(plane::core::now_ms());
    }
    if (!notice) return;
    SPDLOG_INFO("等待所需目标更新: trackId={} 上次接收时间={} timestamp_ms={} 未更新时长={}毫秒", notice->track_id,
                notice->received_time ? reception_time_text(*notice->received_time) : "从未收到",
                notice->timestamp_ms ? std::to_string(*notice->timestamp_ms) : "<无>", notice->age_ms);
}

void TargetStore::monitor_loop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (running_) {
        const auto deadline = required_target_deadline_locked();
        if (!deadline) {
            monitor_cv_.wait(lock);
            continue;
        }
        const auto remaining_ms = *deadline - plane::core::now_ms();
        if (remaining_ms > 0) {
            monitor_cv_.wait_for(lock, std::chrono::milliseconds(remaining_ms));
            continue;
        }
        lock.unlock();
        report_required_target();
        lock.lock();
    }
}

void TargetStore::on_gcs_target(const plane::model::GcsTarget& target) {
    if (target.trackId.empty() || target.timestamp_ms <= 0) {
        SPDLOG_WARN("忽略非法 GCS 目标: trackId={} timestamp_ms={}", target.trackId, target.timestamp_ms);
        return;
    }
    std::size_t history_size = 0;
    std::size_t pending_updates = 0;
    bool first_received = false;
    bool summarize = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        const auto latest = gcs_latest_.find(target.trackId);
        if (latest != gcs_latest_.end() && target.timestamp_ms <= latest->second.timestamp_ms) {
            SPDLOG_WARN("忽略乱序或重复 GCS 目标: trackId={} timestamp_ms={} latest.timestamp_ms={}", target.trackId,
                        target.timestamp_ms, latest->second.timestamp_ms);
            return;
        }
        const auto now = plane::core::now_ms();
        auto [reception, inserted] = receptions_.try_emplace(target.trackId);
        auto& state = reception->second;
        first_received = inserted;
        state.received_ms = now;
        state.received_time = std::chrono::system_clock::now();
        if (inserted) {
            state.last_summary_ms = now;
        } else {
            ++state.pending_updates;
            if (now - state.last_summary_ms >= kTargetSummaryIntervalMs) {
                summarize = true;
                pending_updates = state.pending_updates;
                state.pending_updates = 0;
                state.last_summary_ms = now;
            }
        }
        gcs_latest_[target.trackId] = target;
        auto& history = gcs_history_[target.trackId];
        history.push_back(TargetHistoryItem{target, ++gcs_target_revision_, now});
        if (history.size() > kGcsTargetHistoryDepth) {
            history.pop_front();
        }
        history_size = history.size();
    }
    monitor_cv_.notify_all();
    if (first_received) {
        SPDLOG_INFO("GCS 目标更新: trackId={} timestamp_ms={} history_size={}", target.trackId, target.timestamp_ms,
                    history_size);
    } else if (summarize) {
        SPDLOG_INFO("GCS 目标更新汇总: trackId={} timestamp_ms={} history_size={} 更新数量={}", target.trackId,
                    target.timestamp_ms, history_size, pending_updates);
    }
}

}  // namespace plane::target
