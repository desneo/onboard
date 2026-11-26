#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "communication/communication_events.h"
#include "target/target_store.h"

namespace {

std::atomic<std::int64_t> test_now_ms{100000};

class CaptureSink final : public spdlog::sinks::base_sink<std::mutex> {
   public:
    std::vector<std::string> matching(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        return matching_locked(text);
    }

    bool wait_for_count(const std::string& text, std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, std::chrono::milliseconds(1000),
                            [&]() { return matching_locked(text).size() == count; });
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& message) override {
        messages_.emplace_back(message.payload.data(), message.payload.size());
        cv_.notify_all();
    }
    void flush_() override {}

   private:
    std::vector<std::string> matching_locked(const std::string& text) const {
        std::vector<std::string> result;
        for (const auto& message : messages_) {
            if (message.find(text) != std::string::npos) result.push_back(message);
        }
        return result;
    }

    std::condition_variable cv_;
    std::vector<std::string> messages_;
};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

plane::model::GcsTarget target(const std::string& track_id, std::int64_t timestamp_ms) {
    plane::model::GcsTarget value;
    value.trackId = track_id;
    value.timestamp_ms = timestamp_ms;
    return value;
}

}  // namespace

namespace plane::core {
std::int64_t now_ms() { return test_now_ms.load(); }
}  // namespace plane::core

namespace plane::target {

// 直接触发生产监测路径，以可控单调时间验证边界，无需等待真实的 2 秒和 20 秒。
struct TargetStoreTestAccess {
    static void check(TargetStore& store) { store.report_required_target(); }
};

}  // namespace plane::target

namespace {

struct Fixture {
    explicit Fixture(int target_history_size = 20) : store(events, config), sink(std::make_shared<CaptureSink>()) {
        test_now_ms = 100000;
        config.midcourse_target_history_size = target_history_size;
        auto logger = std::make_shared<spdlog::logger>("目标日志测试", sink);
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
        require(store.start(), "目标管理启动失败");
    }

    void send(std::int64_t now, const std::string& track_id, std::int64_t timestamp_ms) {
        test_now_ms = now;
        events.publish(plane::communication::events::kGcsTargetReceived, target(track_id, timestamp_ms));
    }

    void check(std::int64_t now) {
        test_now_ms = now;
        plane::target::TargetStoreTestAccess::check(store);
    }

    std::vector<std::string> notices() { return sink->matching("等待所需目标更新:"); }

    plane::core::EventBus events;
    plane::core::AppConfig config;
    plane::target::TargetStore store;
    std::shared_ptr<CaptureSink> sink;
};

void test_summary_and_history() {
    Fixture fixture;
    require(!fixture.store.gcs_target_last_received_ms("A"), "从未收到的目标不应有接收时间");
    fixture.send(100000, "A", 1000);
    for (int index = 1; index < 20; ++index) fixture.send(100000 + index * 1000, "A", 1000 + index);
    fixture.send(119999, "A", 1020);
    require(fixture.sink->matching("GCS 目标更新:").size() == 1, "稳定更新不应逐帧打印");
    require(fixture.sink->matching("GCS 目标更新汇总:").empty(), "20 秒前不应汇总");
    fixture.send(120000, "A", 1021);
    auto summaries = fixture.sink->matching("GCS 目标更新汇总:");
    require(summaries.size() == 1 &&
                summaries[0].find("timestamp_ms=1021 history_size=20 更新数量=21") != std::string::npos,
            "汇总应计入触发帧，排除已打印首帧，并保留真实历史帧数");
    const auto history = fixture.store.gcs_target_history("A");
    require(fixture.store.gcs_target_last_received_ms("A") == 120000,
            "查询应返回最后有效帧的本机单调接收时间");
    require(history.size() == 20 && history.front().timestamp_ms == 1002 && history.back().timestamp_ms == 1021,
            "日志降频不得影响逐帧缓存和最近 20 帧历史");

    fixture.send(120100, "B", 2000);
    fixture.send(139999, "A", 1022);
    fixture.send(140000, "B", 2001);
    require(fixture.sink->matching("GCS 目标更新汇总:").size() == 1, "各航迹应独立计时");
    fixture.send(140000, "A", 1023);
    fixture.send(140100, "B", 2002);
    summaries = fixture.sink->matching("GCS 目标更新汇总:");
    require(summaries.size() == 3 && summaries[1].find("trackId=A") != std::string::npos &&
                summaries[1].find("更新数量=2") != std::string::npos &&
                summaries[2].find("trackId=B") != std::string::npos &&
                summaries[2].find("更新数量=2") != std::string::npos,
            "汇总后应重新计数，多航迹统计应相互独立");
    fixture.check(180000);
    require(fixture.notices().empty() && fixture.sink->matching("GCS 目标更新汇总:").size() == 3,
            "无需求不提示，无数据不产生汇总");
}

void test_revision_updates() {
    Fixture fixture;
    require(fixture.store.gcs_target_revision() == 0, "新目标缓存版本应从 0 开始");

    fixture.send(100000, "A", 1000);
    const auto baseline = fixture.store.gcs_target_revision();
    require(baseline == 1, "首个有效目标应递增全局版本");
    fixture.send(100100, "A", 1001);
    fixture.send(100200, "B", 2000);

    const auto a_updates = fixture.store.gcs_target_updates_after("A", baseline);
    require(a_updates.history.size() == 1 && a_updates.history.front().timestamp_ms == 1001 &&
                a_updates.last_received_ms == 100100,
            "航迹更新快照应只返回基线后的数据及其接收时间");
    const auto b_updates = fixture.store.gcs_target_updates_after("B", baseline);
    require(b_updates.history.size() == 1 && b_updates.history.front().timestamp_ms == 2000 &&
                b_updates.last_received_ms == 100200,
            "全局版本过滤后各航迹数据应保持隔离");

    const auto accepted_revision = fixture.store.gcs_target_revision();
    fixture.send(100300, "A", 1001);
    fixture.send(100400, "A", 999);
    require(fixture.store.gcs_target_revision() == accepted_revision,
            "重复或乱序目标不得递增全局版本");
    const auto rejected_updates = fixture.store.gcs_target_updates_after("A", accepted_revision);
    require(rejected_updates.history.empty() && !rejected_updates.last_received_ms,
            "重复或乱序目标不得出现在增量快照中");

    for (std::int64_t index = 2; index <= 25; ++index) {
        fixture.send(101000 + index, "A", 1000 + index);
    }
    const auto retained = fixture.store.gcs_target_updates_after("A", 0);
    require(retained.history.size() == 20 && retained.history.front().timestamp_ms == 1006 &&
                retained.history.back().timestamp_ms == 1025 && retained.last_received_ms == 101025,
            "增量快照应与航迹历史一致，最多保留最近 20 帧");
    require(fixture.store.gcs_target_updates_after("B", 0).history.size() == 1,
            "其他航迹的 20 帧裁剪不得影响已保留的目标");
}

void test_configured_history_depth() {
    Fixture fixture(30);
    for (std::int64_t index = 0; index < 35; ++index) {
        fixture.send(100000 + index, "A", 1000 + index);
    }
    const auto history = fixture.store.gcs_target_history("A");
    require(history.size() == 30 && history.front().timestamp_ms == 1005 && history.back().timestamp_ms == 1034,
            "target_history_size=30 时应保留每航迹最近 30 帧");
    const auto updates = fixture.store.gcs_target_updates_after("A", 0);
    require(updates.history.size() == 30 && updates.history.front().timestamp_ms == 1005 &&
                updates.history.back().timestamp_ms == 1034,
            "增量快照应采用配置的目标历史缓存深度");

    for (std::int64_t index = 0; index < 31; ++index) {
        fixture.send(101000 + index, "B", 2000 + index);
    }
    const auto other_history = fixture.store.gcs_target_history("B");
    require(other_history.size() == 30 && other_history.front().timestamp_ms == 2001 &&
                fixture.store.gcs_target_history("A").size() == 30,
            "配置缓存深度应逐航迹独立生效");
}

void test_never_received_and_cancel() {
    Fixture fixture;
    fixture.store.set_required_gcs_target("wanted");
    fixture.check(101999);
    require(fixture.notices().empty(), "需求开始未满 2 秒不应提示");
    fixture.check(102000);
    auto notices = fixture.notices();
    require(notices.size() == 1 &&
                notices[0].find("trackId=wanted 上次接收时间=从未收到 timestamp_ms=<无>") != std::string::npos &&
                notices[0].find("未更新时长=2000毫秒") != std::string::npos,
            "首次未接收提示应准确标注时间与等待时长");
    fixture.check(103999);
    require(fixture.notices().size() == 1, "重复提示间隔不得少于 2 秒");
    fixture.check(104000);
    require(fixture.notices().size() == 2, "持续缺失应每 2 秒提示");
    fixture.send(104500, "other", 2000);
    fixture.check(106000);
    notices = fixture.notices();
    require(notices.size() == 3 && notices.back().find("未更新时长=6000毫秒") != std::string::npos,
            "其他航迹不得刷新指定航迹的等待时间");
    fixture.store.set_required_gcs_target(std::nullopt);
    fixture.store.lock_required_gcs_target("wanted");
    fixture.check(108000);
    require(fixture.notices().size() == 3, "取消后迟到的锁定不得重新启用提示");
}

void test_invalid_frames_and_recovery() {
    Fixture fixture;
    fixture.send(100000, "A", 1000);
    fixture.store.set_required_gcs_target("A");
    fixture.send(101000, "A", 1000);
    fixture.send(101100, "A", 999);
    fixture.send(101200, "A", 0);
    fixture.send(101500, "other", 2000);
    fixture.check(101999);
    require(fixture.notices().empty(), "首次接收满 2 秒前不应提示");
    fixture.check(102000);
    auto notices = fixture.notices();
    require(notices.size() == 1 && notices[0].find("timestamp_ms=1000 未更新时长=2000毫秒") != std::string::npos,
            "重复、乱序、非法帧不得刷新接收时间");
    require(fixture.store.gcs_target_history("A").size() == 1 &&
                fixture.sink->matching("忽略乱序或重复 GCS 目标:").size() == 2 &&
                fixture.sink->matching("忽略非法 GCS 目标:").size() == 1,
            "异常数据应继续拒绝并保留告警");
    fixture.send(102500, "A", 1001);
    fixture.check(104499);
    require(fixture.notices().size() == 1, "恢复后应从新的接收时间重新等待 2 秒");
    fixture.check(104500);
    notices = fixture.notices();
    require(notices.size() == 2 && notices.back().find("timestamp_ms=1001 未更新时长=2000毫秒") != std::string::npos,
            "再次中断应按新的报文及接收时间提示");
}

void test_old_cache_and_readable_time() {
    Fixture fixture;
    fixture.send(100000, "A", 1000);
    test_now_ms = 110000;
    fixture.store.set_required_gcs_target("A");
    fixture.check(111999);
    require(fixture.notices().empty(), "任务前陈旧缓存不应导致任务开始立即提示");
    fixture.check(112000);
    const auto notices = fixture.notices();
    require(notices.size() == 1 && notices[0].find("未更新时长=12000毫秒") != std::string::npos &&
                std::regex_search(notices[0], std::regex("上次接收时间=[0-9]{4}-[0-9]{2}-[0-9]{2} "
                                                        "[0-9]{2}:[0-9]{2}:[0-9]{2}\\.[0-9]{3}[+-][0-9]{4}")),
            "陈旧缓存应显示真实接收龄期和带时区的可读时间");
}

void test_automatic_and_locked_target() {
    Fixture fixture;
    fixture.store.set_required_gcs_target("");
    fixture.check(102000);
    require(fixture.notices().size() == 1 && fixture.notices()[0].find("trackId=<自动>") != std::string::npos,
            "自动模式无候选时应明确提示");
    fixture.send(102500, "A", 1000);
    fixture.check(104499);
    require(fixture.notices().size() == 1, "自动模式应观察唯一候选的有效接收时间");
    fixture.check(104500);
    require(fixture.notices().size() == 2 && fixture.notices().back().find("trackId=A") != std::string::npos,
            "自动模式唯一候选中断应提示该航迹");
    fixture.send(105000, "B", 2000);
    fixture.check(107000);
    require(fixture.notices().size() == 2, "自动模式多候选不得误报未收到");
    fixture.store.lock_required_gcs_target("A");
    fixture.check(107000);
    require(fixture.notices().size() == 3 && fixture.notices().back().find("trackId=A") != std::string::npos,
            "实际锁定不应重置等待计时");
    fixture.send(108500, "B", 2001);
    fixture.check(109000);
    require(fixture.notices().size() == 4 &&
                fixture.notices().back().find("timestamp_ms=1000 未更新时长=6500毫秒") != std::string::npos,
            "锁定后其他航迹不得刷新所需航迹接收时间");
    fixture.store.set_required_gcs_target(std::nullopt);
    test_now_ms = 111000;
    fixture.store.set_required_gcs_target("B");
    fixture.store.lock_required_gcs_target("A");
    fixture.check(113000);
    require(fixture.notices().size() == 5 && fixture.notices().back().find("trackId=B") != std::string::npos,
            "自动锁定接口不得覆盖显式指定航迹");
}

void test_worker_wakeup_and_restart() {
    Fixture fixture;
    fixture.store.set_required_gcs_target("A");
    test_now_ms = 102000;
    fixture.store.set_required_gcs_target("A");
    require(fixture.sink->wait_for_count("等待所需目标更新:", 1), "监测线程应独立唤醒，相同需求不重置计时");
    fixture.store.stop();
    fixture.send(104000, "A", 1000);
    fixture.check(106000);
    require(fixture.notices().size() == 1 && fixture.store.gcs_targets().empty(), "停止后不得接收数据或继续提示");
    fixture.store.stop();
    require(fixture.store.start(), "目标管理重启失败");
    fixture.send(106000, "A", 1000);
    require(fixture.store.gcs_target_history("A").size() == 1 &&
                fixture.sink->matching("GCS 目标更新:").size() == 1,
            "重启应清理统计且不得重复订阅");
}

}  // namespace

int main() {
    try {
        test_summary_and_history();
        test_revision_updates();
        test_configured_history_depth();
        test_never_received_and_cancel();
        test_invalid_frames_and_recovery();
        test_old_cache_and_readable_time();
        test_automatic_and_locked_target();
        test_worker_wakeup_and_restart();
    } catch (const std::exception& error) {
        spdlog::stderr_color_mt("目标日志测试失败")->error("{}", error.what());
        return 1;
    }
    return 0;
}
