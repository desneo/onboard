#include <string>

#include "camera/camera_control_events.h"
#include "camera/camera_control_module.h"
#include "communication/communication_events.h"
#include "target/target_store.h"

int main() {
    plane::core::EventBus event_bus;
    plane::core::AppConfig config;
    plane::target::TargetStore targets(event_bus, config);
    if (!targets.init() || !targets.start()) return 1;

    plane::target::GcsTarget target;
    target.trackId = "test-target";
    for (int index = 0; index < 21; ++index) {
        target.timestamp_ms = 1000 + index * 1000;
        target.radial_speed_m_s = static_cast<float>(index);
        event_bus.publish(plane::communication::events::kGcsTargetReceived, target);
    }
    plane::target::GcsTarget latest;
    if (!targets.gcs_target(target.trackId, latest) || latest.radial_speed_m_s != 20.0F) return 2;
    const auto history = targets.gcs_target_history(target.trackId);
    if (history.size() != 20 || history.front().radial_speed_m_s != 1.0F ||
        history.back().radial_speed_m_s != 20.0F)
        return 3;
    if (targets.gcs_target("missing", latest) || targets.gcs_targets().size() != 1) return 4;

    target.timestamp_ms = latest.timestamp_ms;
    target.radial_speed_m_s = 99.0F;
    event_bus.publish(plane::communication::events::kGcsTargetReceived, target);
    if (!targets.gcs_target(target.trackId, latest) || latest.radial_speed_m_s != 20.0F ||
        targets.gcs_target_history(target.trackId).size() != 20)
        return 5;

    int status_events = 0;
    bool connected = false;
    event_bus.subscribe<plane::camera::CameraStatus>(plane::camera::events::kStatusChanged,
                                                     [&](const plane::camera::CameraStatus& status) {
                                                         ++status_events;
                                                         connected = status.connected;
                                                     });
    plane::camera::CameraControlModule camera(event_bus);
    std::string error;
    if (!camera.init() || !camera.start() || !camera.status().connected || !connected) return 6;
    if (!camera.send_command("status", error)) return 7;
    camera.stop();
    if (camera.status().connected || connected || status_events != 2) return 8;
    camera.stop();
    if (status_events != 2) return 9;
    targets.stop();
    if (!targets.gcs_targets().empty() || !targets.gcs_target_history(target.trackId).empty()) return 10;
    return 0;
}
