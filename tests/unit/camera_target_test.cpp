#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camera/camera_control_module.h"
#include "camera/pod_driver.h"
#include "communication/communication_events.h"
#include "target/target_store.h"

namespace {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class FakePod final : public plane::camera::PodDriver {
   public:
    bool can_move() const override { return true; }
    bool start(std::string&) override { return true; }
    std::optional<plane::camera::PodTelemetry> tick(Clock::time_point) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (!telemetry_enabled) return std::nullopt;
        return plane::camera::PodTelemetry{{{"俯仰角（°）", 5.0}}};
    }
    bool apply(const plane::model::PodCommand& command, Clock::time_point, std::string&) override {
        std::lock_guard<std::mutex> lock(mutex);
        if (command.pitchRate_deg_s != 0.0 || command.yawRate_deg_s != 0.0) {
            ++moving;
        } else {
            ++stops;
        }
        return true;
    }
    void stop() override {}

    std::mutex mutex;
    bool telemetry_enabled = true;
    int moving = 0;
    int stops = 0;
};

template <typename Predicate>
bool wait_for(Predicate predicate, Clock::duration timeout = 1500ms) {
    const auto end = Clock::now() + timeout;
    while (!predicate()) {
        if (Clock::now() >= end) return false;
        std::this_thread::sleep_for(5ms);
    }
    return true;
}

}  // namespace

int main() {
    plane::core::EventBus event_bus;
    plane::core::AppConfig config;
    config.midcourse_target_history_size = 20;
    plane::target::TargetStore targets(event_bus, config);
    if (!targets.init() || !targets.start()) return 1;

    plane::model::GcsTarget target;
    target.trackId = "test-target";
    for (int index = 0; index < 21; ++index) {
        target.timestamp_ms = 1000 + index * 1000;
        target.geoPos.altitude = static_cast<float>(index);
        event_bus.publish(plane::communication::events::kGcsTargetReceived, target);
    }
    plane::model::GcsTarget latest;
    if (!targets.gcs_target(target.trackId, latest) || latest.geoPos.altitude != 20.0F) return 2;
    const auto history = targets.gcs_target_history(target.trackId);
    if (history.size() != 20 || history.front().geoPos.altitude != 1.0F ||
        history.back().geoPos.altitude != 20.0F)
        return 3;
    if (targets.gcs_target("missing", latest) || targets.gcs_targets().size() != 1) return 4;

    target.timestamp_ms = latest.timestamp_ms;
    target.geoPos.altitude = 99.0F;
    event_bus.publish(plane::communication::events::kGcsTargetReceived, target);
    if (!targets.gcs_target(target.trackId, latest) || latest.geoPos.altitude != 20.0F ||
        targets.gcs_target_history(target.trackId).size() != 20)
        return 5;

    targets.stop();
    if (!targets.gcs_targets().empty() || !targets.gcs_target_history(target.trackId).empty()) return 10;

    plane::camera::CameraControlModule disabled(config);
    if (!disabled.init() || !disabled.start() || disabled.pod_state()) return 11;
    config.pod_model = "aisheng_asd2";
    auto fake = std::make_unique<FakePod>();
    auto* pod = fake.get();
    plane::camera::CameraControlModule camera(config, std::move(fake));
    if (!camera.init() || !camera.start()) return 12;
    if (!wait_for([&] { return camera.pod_state()->connected; })) return 13;
    const auto connected = camera.pod_state();
    if (!connected->canMove || connected->planeId != config.plane_id || connected->timestamp_ms <= 0 ||
        !connected->fields.count("俯仰角（°）")) {
        return 14;
    }

    plane::model::PodCommand rate;
    rate.pitchRate_deg_s = 5.0;
    rate.yawRate_deg_s = -4.0;
    camera.submit_command(rate, Clock::now() - 1s);
    std::this_thread::sleep_for(40ms);
    {
        std::lock_guard<std::mutex> lock(pod->mutex);
        if (pod->moving || pod->stops) return 15;
    }
    camera.submit_command(rate, Clock::now());
    if (!wait_for([&] {
            std::lock_guard<std::mutex> lock(pod->mutex);
            return pod->moving > 0;
        })) return 16;
    camera.submit_command({}, Clock::now());
    if (!wait_for([&] {
            std::lock_guard<std::mutex> lock(pod->mutex);
            return pod->stops == 1;
        }, 200ms)) return 17;

    camera.submit_command(rate, Clock::now());
    if (!wait_for([&] {
            std::lock_guard<std::mutex> lock(pod->mutex);
            return pod->stops == 2 && pod->moving >= 3;
        }, 800ms)) return 18;
    int movement_count = 0;
    {
        std::lock_guard<std::mutex> lock(pod->mutex);
        pod->telemetry_enabled = false;
        movement_count = pod->moving;
    }
    if (!wait_for([&] { return !camera.pod_state()->connected; }) || !camera.pod_state()->fields.empty()) return 19;
    camera.submit_command(rate, Clock::now());
    if (!wait_for([&] { return camera.pod_state()->fields.count("控制提示") != 0; })) return 20;
    {
        std::lock_guard<std::mutex> lock(pod->mutex);
        pod->telemetry_enabled = true;
    }
    if (!wait_for([&] { return camera.pod_state()->connected; })) return 21;
    camera.stop();
    if (camera.pod_state()->connected || pod->moving != movement_count) return 22;
    return 0;
}
