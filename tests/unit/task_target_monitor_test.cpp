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
    explicit BlockedFlightControl(bool armed) {
        status_.armed = armed;
        status_.connected = true;
        status_.on_ground = true;
    }

    void prepare() {
        std::lock_guard<std::mutex> lock(mutex_);
        entered_ = false;
        released_ = false;
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

    bool arm(std::string& error) override { return wait_for_release(error); }
    bool takeoff(float, std::string& error) override { return wait_for_release(error); }
    bool disarm(std::string&) override { return true; }
    bool land(std::string&) override { return true; }
    bool return_to_launch(std::string&) override { return true; }
    bool hold(std::string&) override { return true; }
    bool stop_motor(std::string&) override { return true; }
    bool goto_waypoint(const plane::model::GeoPoint&, std::string&) override { return true; }
    bool execute_route(const std::vector<plane::model::GeoPoint>&, std::string&) override { return true; }
    bool set_ned_velocity(const plane::model::NedVelocity&, std::string&) override { return true; }
    bool set_ned_velocity_acceleration(const plane::algorithm::VelocityAccelerationCommand&, double,
                                       std::string&) override { return true; }
    bool set_attitude(const plane::flight_control::AttitudeSetpoint&, std::string&) override { return true; }
    bool get_cached_flight_control_params(plane::flight_control::FlightControlParams&, std::string&) const override {
        return false;
    }
    void clear_offboard() override {}
    std::string link_state() const override { return "connected"; }
    plane::flight_control::FlightStatus flight_status() const override { return status_; }

   private:
    bool wait_for_release(std::string& error) {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        changed_.notify_all();
        changed_.wait(lock, [&] { return released_; });
        error = "模拟飞控回执失败";
        return false;
    }

    plane::flight_control::FlightStatus status_;
    std::mutex mutex_;
    std::condition_variable changed_;
    bool entered_ = false;
    bool released_ = false;
};

class FakeStateStore final : public plane::state::StateStoreApi {
   public:
    plane::model::VehicleState own_state() const override { return {}; }
    std::vector<plane::model::VehicleState> peer_states() const override { return {}; }
    bool peer_state(const std::string&, plane::model::VehicleState&) const override { return false; }
    plane::state::GcsLinkState gcs_link() const override { return {}; }
    void update_own_state(const plane::model::VehicleState&) override {}
    void update_peer_state(const plane::model::VehicleState&) override {}
    void notify_station_seen() override {}
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
    explicit Fixture(bool armed)
        : flight(armed), targets(events, config), engine(events, config, flight, states, params, targets) {
        config.bt_tick_interval_ms = 10;
        events.subscribe<plane::task::TaskEngineResult>(
            plane::task::events::kEngineResult, [this](const plane::task::TaskEngineResult& result) {
                last_result.store(result.status);
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

    plane::core::AppConfig config;
    plane::core::EventBus events;
    BlockedFlightControl flight;
    FakeStateStore states;
    FakeSystemParam params;
    plane::target::TargetStore targets;
    plane::task::TaskExecutionEngine engine;
    std::atomic<plane::model::TaskStatus> last_result{plane::model::TaskStatus::None};
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

}  // namespace

int main() {
    const auto logs = std::make_shared<CapturedLogs>();
    const auto logger = std::make_shared<spdlog::logger>("task-target-monitor-test", logs);
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    try {
        arm_wait_and_failure(*logs);
        takeoff_wait_abort_and_replacement(*logs);
    } catch (const std::exception& error) {
        spdlog::set_default_logger(spdlog::stderr_color_mt("task-target-monitor-error"));
        SPDLOG_ERROR("任务目标监测测试失败: {}", error.what());
        return 1;
    }
    return 0;
}
