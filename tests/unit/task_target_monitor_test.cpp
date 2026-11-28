#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "communication/communication_events.h"
#include "core/system_param_api.h"
#include "state/state_store_api.h"
#include "target/target_store.h"
#include "task/task_events.h"
#include "task/task_execution_engine.h"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}

class CapturedLogs final : public spdlog::sinks::base_sink<std::mutex> {
   public:
    struct Entry {
        std::string text;
        Clock::time_point time;
    };

    std::vector<Entry> prompts(const std::string& track_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        return prompts_locked(track_id);
    }

    bool wait_for_prompts(const std::string& track_id, std::size_t count,
                          Clock::duration timeout = 6500ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, timeout, [&] { return prompts_locked(track_id).size() >= count; });
    }

    std::size_t total_prompts() {
        std::lock_guard<std::mutex> lock(mutex_);
        return total_prompts_locked();
    }

    bool wait_for_total_prompts(std::size_t count, Clock::duration timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, timeout, [&] { return total_prompts_locked() >= count; });
    }

    bool contains(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& entry : entries_) {
            if (entry.text.find(text) != std::string::npos) return true;
        }
        return false;
    }

   protected:
    void sink_it_(const spdlog::details::log_msg& message) override {
        entries_.push_back({std::string(message.payload.data(), message.payload.size()), Clock::now()});
        changed_.notify_all();
    }

    void flush_() override {}

   private:
    std::size_t total_prompts_locked() const {
        std::size_t count = 0;
        for (const auto& entry : entries_) {
            if (entry.text.find("等待所需目标更新:") != std::string::npos) ++count;
        }
        return count;
    }

    std::vector<Entry> prompts_locked(const std::string& track_id) const {
        std::vector<Entry> result;
        const std::string field = "trackId=" + track_id + " ";
        for (const auto& entry : entries_) {
            if (entry.text.find("等待所需目标更新:") != std::string::npos &&
                entry.text.find(field) != std::string::npos) {
                result.push_back(entry);
            }
        }
        return result;
    }

    std::condition_variable changed_;
    std::vector<Entry> entries_;
};

class BlockedFlightControl final : public plane::flight_control::FlightControlApi {
   public:
    explicit BlockedFlightControl(bool armed, bool in_air = false) {
        status_.armed = armed;
        status_.connected = true;
        status_.in_air = in_air;
        status_.on_ground = !in_air;
    }

    void prepare() {
        std::lock_guard<std::mutex> lock(mutex_);
        entered_ = false;
        released_ = false;
        action_succeeds_ = false;
        takeoff_observed_ = false;
    }

    bool wait_until_entered() {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, 2s, [&] { return entered_; });
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        changed_.notify_all();
    }

    void release_takeoff_successfully() {
        std::lock_guard<std::mutex> lock(mutex_);
        action_succeeds_ = true;
        status_.flight_mode = "Takeoff";
        status_.in_air = true;
        status_.on_ground = false;
        status_.landed_in_air = true;
        released_ = true;
        changed_.notify_all();
    }

    bool wait_until_takeoff_observed() {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, 2s, [&] { return takeoff_observed_; });
    }

    void finish_takeoff() {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.flight_mode = "Hold";
    }

    bool arm(std::string& error) override { return wait_for_release(error); }
    bool takeoff(float, std::string& error) override { return wait_for_release(error); }
    bool disarm(std::string&) override { return true; }
    bool land(std::string&) override { return true; }
    bool return_to_launch(std::string&) override { return true; }
    bool hold(std::string&) override { return true; }
    bool stop_motor(std::string&) override { return true; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override { return true; }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return true; }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override {
        ++setpoint_calls;
        return true;
    }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand&, double,
                                       std::string&) override {
        ++setpoint_calls;
        return true;
    }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint&, std::string&) override {
        ++setpoint_calls;
        return true;
    }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams&, std::string&) const override {
        return false;
    }
    void clear_offboard() override { ++clear_calls; }
    std::string link_state() const override { return "connected"; }
    plane::flight_control::FlightStatus flight_status() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status_.flight_mode == "Takeoff") {
            takeoff_observed_ = true;
            changed_.notify_all();
        }
        return status_;
    }

    std::atomic<int> setpoint_calls{0};
    std::atomic<int> clear_calls{0};

   private:
    bool wait_for_release(std::string& error) {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        changed_.wait(lock, [&] { return released_; });
        if (!action_succeeds_) {
            error = "模拟飞控回执失败";
        }
        return action_succeeds_;
    }

    plane::flight_control::FlightStatus status_;
    mutable std::mutex mutex_;
    mutable std::condition_variable changed_;
    bool entered_ = false;
    bool released_ = false;
    bool action_succeeds_ = false;
    mutable bool takeoff_observed_ = false;
};

class FakeStateStore final : public plane::state::StateStoreApi {
   public:
    FakeStateStore() {
        value.geoPos.latitude = 30.0;
        value.geoPos.longitude = 120.0;
        value.geoPos.altitude = 50.0F;
        value.startPos = value.geoPos;
        value.yaw_deg = 0.0F;
    }

    plane::model::VehicleState own_state() const override { return value; }
    std::vector<plane::model::VehicleState> peer_states() const override { return {}; }
    bool peer_state(const std::string&, plane::model::VehicleState&) const override { return false; }
    plane::state::GcsLinkState gcs_link() const override { return {}; }
    void update_own_state(const plane::model::VehicleState& state) override { value = state; }
    void update_peer_state(const plane::model::VehicleState&) override {}
    void notify_station_seen() override {}

    plane::model::VehicleState value;
};

class FakeSystemParam final : public plane::core::SystemParamApi {
   public:
    plane::model::SystemParam current() const override {
        plane::model::SystemParam result;
        result.safeHeight_m = 20.0;
        return result;
    }
};

struct Fixture {
    explicit Fixture(bool armed, bool in_air = false)
        : flight(armed, in_air), targets(events, config), engine(events, config, flight, states, params, targets) {
        config.bt_update_rate_hz = 100;
        config.midcourse_target_history_size = 20;
        config.midcourse_target_timeout_ms = 5000;
        config.midcourse_algorithm.update_dt_s = 0.05;
        events.subscribe<plane::task::TaskEngineResult>(
            plane::task::events::kEngineResult, [this](const plane::task::TaskEngineResult& result) {
                {
                    std::lock_guard<std::mutex> lock(results_mutex);
                    results.push_back(result);
                }
                last_result.store(result.status);
                results_changed.notify_all();
            });
        require(targets.start() && engine.start(), "目标管理或执行引擎启动失败");
    }

    ~Fixture() {
        // 先释放模拟飞控，保证断言异常时也能回收行为树线程。
        flight.release();
        engine.stop();
        targets.stop();
    }

    void start_task(std::uint64_t execution_id, const std::string& track_id) {
        plane::model::TaskInfo task;
        task.taskId = "test-task-" + std::to_string(execution_id);
        task.taskCode = plane::model::TaskCode::AntiUav;
        task.targetTrackId = track_id;
        std::string error;
        require(engine.start(execution_id, task, error), "任务创建失败: " + error);
        require(flight.wait_until_entered(), "行为树未进入模拟飞控等待");
    }

    bool wait_for_result() const {
        const auto deadline = Clock::now() + 2s;
        while (last_result.load() == plane::model::TaskStatus::None && Clock::now() < deadline) {
            std::this_thread::sleep_for(5ms);
        }
        return last_result.load() != plane::model::TaskStatus::None;
    }

    bool wait_for_results(std::size_t count) {
        std::unique_lock<std::mutex> lock(results_mutex);
        return results_changed.wait_for(lock, 2s, [&] { return results.size() >= count; });
    }

    std::vector<plane::task::TaskEngineResult> result_snapshot() {
        std::lock_guard<std::mutex> lock(results_mutex);
        return results;
    }

    plane::core::AppConfig config;
    plane::core::EventBus events;
    BlockedFlightControl flight;
    FakeStateStore states;
    FakeSystemParam params;
    plane::target::TargetStore targets;
    plane::task::TaskExecutionEngine engine;
    std::atomic<plane::model::TaskStatus> last_result{plane::model::TaskStatus::None};
    std::mutex results_mutex;
    std::condition_variable results_changed;
    std::vector<plane::task::TaskEngineResult> results;
};

class PendingAbort {
   public:
    explicit PendingAbort(Fixture& fixture)
        : fixture_(fixture), result_(promise_.get_future()), worker_([this] {
              try {
                  promise_.set_value(fixture_.engine.abort());
              } catch (...) {
                  promise_.set_exception(std::current_exception());
              }
          }) {}

    ~PendingAbort() {
        fixture_.flight.release();
        if (worker_.joinable()) worker_.join();
    }

    bool waiting() { return result_.wait_for(0ms) == std::future_status::timeout; }

    void finish() {
        fixture_.flight.release();
        require(result_.wait_for(2s) == std::future_status::ready, "释放飞控后中止未完成");
        require(result_.get(), "任务中止失败");
        worker_.join();
    }

   private:
    Fixture& fixture_;
    std::promise<bool> promise_;
    std::future<bool> result_;
    std::thread worker_;
};

void require_periodic_prompts(CapturedLogs& logs, const std::string& track_id) {
    require(logs.wait_for_prompts(track_id, 2), "飞控阻塞期间未持续打印两次目标缺失提示");
    const auto entries = logs.prompts(track_id);
    const auto spacing = entries[1].time - entries[0].time;
    require(spacing >= 1800ms && spacing <= 2800ms, "目标缺失提示没有保持两秒间隔");
}

void arm_wait_and_failure(CapturedLogs& logs) {
    Fixture fixture(false);
    const std::string track_id = "blocked-arm-target";
    const auto started_at = Clock::now();
    fixture.start_task(1, track_id);
    require_periodic_prompts(logs, track_id);
    require(Clock::now() - started_at >= 3900ms, "解锁等待未覆盖两个完整提示周期");
    fixture.flight.release();
    require(fixture.wait_for_result(), "模拟解锁失败未结束任务");
    require(fixture.last_result.load() == plane::model::TaskStatus::Failed, "解锁失败未上报任务失败");
    const auto count = logs.prompts(track_id).size();
    require(!logs.wait_for_prompts(track_id, count + 1, 2200ms), "任务失败后仍打印目标缺失提示");
}

void takeoff_wait_abort_and_replacement(CapturedLogs& logs) {
    Fixture fixture(true);
    const std::string old_track_id = "blocked-takeoff-target";
    const auto started_at = Clock::now();
    fixture.start_task(2, old_track_id);
    require_periodic_prompts(logs, old_track_id);
    require(Clock::now() - started_at >= 3900ms, "起飞等待未覆盖两个完整提示周期");

    {
        PendingAbort abort(fixture);
        // 给中止调用执行取消监测的机会，飞控回执仍保持阻塞。
        std::this_thread::sleep_for(50ms);
        const auto count = logs.prompts(old_track_id).size();
        require(abort.waiting(), "飞控尚未返回，中止应仍等待行为树退出");
        require(!logs.wait_for_prompts(old_track_id, count + 1, 2200ms),
                "等待行为树退出期间未提前取消目标监测");
        require(abort.waiting(), "模拟飞控等待被意外解除");
        abort.finish();
    }

    const auto old_count = logs.prompts(old_track_id).size();
    const std::string new_track_id = "replacement-target";
    fixture.flight.prepare();
    fixture.start_task(3, new_track_id);
    require(logs.wait_for_prompts(new_track_id, 1, 3200ms), "替换任务未开始监测新航迹");
    require(logs.prompts(old_track_id).size() == old_count, "替换任务后仍监测旧航迹");
    fixture.flight.release();
    require(fixture.engine.abort(), "替换任务中止失败");

    fixture.flight.prepare();
    plane::model::Command command;
    command.commandId = "independent-takeoff";
    command.type = plane::model::CommandType::Takeoff;
    command.takeoffHeight_m = 20.0F;
    std::string error;
    require(fixture.engine.start_command(4, command, error), "独立指令创建失败: " + error);
    require(fixture.flight.wait_until_entered(), "独立指令未进入模拟飞控等待");
    const auto prompt_count = logs.total_prompts();
    require(!logs.wait_for_total_prompts(prompt_count + 1, 2200ms), "独立指令仍保留任务目标监测");
    require(logs.prompts(old_track_id).size() == old_count, "独立指令重新启用了旧航迹监测");
    fixture.flight.release();
    require(fixture.engine.abort(), "独立指令中止失败");
}

void takeoff_history_is_ready_on_midcourse_entry() {
    Fixture fixture(true);
    const std::string track_id = "takeoff-history-target";
    fixture.start_task(5, track_id);
    for (int index = 0; index < 20; ++index) {
        plane::model::GcsTarget target;
        target.trackId = track_id;
        target.timestamp_ms = 2000 + index;
        target.geoPos.latitude = 30.01 + static_cast<double>(index) * 0.00001;
        target.geoPos.longitude = 120.0;
        target.geoPos.altitude = 50.0F;
        fixture.events.publish(plane::communication::events::kGcsTargetReceived, target);
    }

    fixture.flight.release_takeoff_successfully();
    require(fixture.flight.wait_until_takeoff_observed(), "起飞成功后未观测到 Takeoff 模式");
    fixture.flight.finish_takeoff();
    const auto deadline = Clock::now() + 2s;
    while (fixture.flight.setpoint_calls.load() == 0 && Clock::now() < deadline) {
        std::this_thread::sleep_for(5ms);
    }
    require(fixture.flight.setpoint_calls.load() > 0,
            "起飞期间已收齐 20 帧，但进入 Midcourse 后未立即初始化并下发控制");
    require(!fixture.wait_for_result(), "进入中制导并下发控制后任务不应自行结束");
    require(fixture.engine.abort(), "起飞历史就绪场景无法中止任务");
}

void midcourse_missing_target_failure_and_restart(CapturedLogs& logs) {
    Fixture fixture(true, true);
    fixture.config.midcourse_target_timeout_ms = 100;
    const std::string track_id = "missing-midcourse-target";
    plane::model::TaskInfo task;
    task.taskId = "midcourse-timeout-task";
    task.taskCode = plane::model::TaskCode::AntiUav;
    task.targetTrackId = track_id;

    std::string error;
    const auto started_at = Clock::now();
    const auto clear_calls_before = fixture.flight.clear_calls.load();
    require(fixture.engine.start(10, task, error), "中制导超时任务创建失败: " + error);
    require(fixture.wait_for_results(1), "中制导未收到目标时未按配置超时结束");
    require(Clock::now() - started_at >= 100ms, "中制导目标超时早于配置阈值");
    const auto results = fixture.result_snapshot();
    require(results.size() == 1 && results[0].execution_id == 10 &&
                results[0].status == plane::model::TaskStatus::Failed &&
                results[0].currentBehaviorTree == "AntiUav" && results[0].currentNode == "Midcourse",
            "目标超时未上报一次关联当前执行和节点的任务失败");
    require(results[0].detail.find("中制导目标雷达信息超时") != std::string::npos &&
                results[0].detail.find("trackId=" + track_id) != std::string::npos &&
                results[0].detail.find("target_timeout_ms=100") != std::string::npos &&
                results[0].detail.find("从未收到") != std::string::npos,
            "任务失败详情遗漏目标、配置阈值或从未收到的原因");
    require(logs.contains("中段制导失败: " + results[0].detail), "目标超时缺少显式失败日志");
    require(fixture.flight.clear_calls.load() == clear_calls_before + 1 &&
                fixture.flight.setpoint_calls.load() == 0,
            "初始化前目标超时未清理 Offboard 或仍下发了控制设定值");

    const auto prompt_count = logs.prompts(track_id).size();
    require(!logs.wait_for_prompts(track_id, prompt_count + 1, 2200ms),
            "目标超时失败后未取消目标需求，仍输出缺失提示");
    for (int index = 0; index < 20; ++index) {
        plane::model::GcsTarget target;
        target.trackId = track_id;
        target.timestamp_ms = 1000 + index;
        target.geoPos.latitude = 30.0;
        target.geoPos.longitude = 120.0;
        target.geoPos.altitude = 50.0F;
        fixture.events.publish(plane::communication::events::kGcsTargetReceived, target);
    }
    require(fixture.targets.gcs_target_history(track_id).size() == 20, "迟到目标未被真实目标缓存接受");
    std::this_thread::sleep_for(150ms);
    require(fixture.result_snapshot().size() == 1 && fixture.flight.setpoint_calls.load() == 0 &&
                fixture.flight.clear_calls.load() == clear_calls_before + 1,
            "迟到目标恢复了已结束的任务或重复上报失败");

    // 与调度器相同，先回收结束的执行线程，再接收新任务。
    require(fixture.engine.abort(), "目标超时后无法回收旧任务");
    task.taskId = "midcourse-after-timeout";
    task.targetTrackId = "new-midcourse-target";
    const auto restarted_at = Clock::now();
    require(fixture.engine.start(11, task, error), "目标超时后不能启动新任务: " + error);
    require(fixture.wait_for_results(2), "新任务未开始独立的目标超时监测");
    const auto restarted_results = fixture.result_snapshot();
    require(Clock::now() - restarted_at >= 100ms && restarted_results.size() == 2 &&
                restarted_results[1].execution_id == 11 &&
                restarted_results[1].status == plane::model::TaskStatus::Failed &&
                restarted_results[1].currentBehaviorTree == "AntiUav" &&
                restarted_results[1].currentNode == "Midcourse" &&
                restarted_results[1].detail.find("trackId=new-midcourse-target") != std::string::npos,
            "新任务沿用了旧任务的超时结果、目标或计时起点");
}

}  // namespace

int main() {
    const auto logs = std::make_shared<CapturedLogs>();
    const auto logger = std::make_shared<spdlog::logger>("task-target-monitor-test", logs);
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    try {
        arm_wait_and_failure(*logs);
        takeoff_wait_abort_and_replacement(*logs);
        takeoff_history_is_ready_on_midcourse_entry();
        midcourse_missing_target_failure_and_restart(*logs);
    } catch (const std::exception& error) {
        spdlog::set_default_logger(spdlog::stderr_color_mt("task-target-monitor-error"));
        SPDLOG_ERROR("任务目标监测测试失败: {}", error.what());
        return 1;
    }
    return 0;
}
