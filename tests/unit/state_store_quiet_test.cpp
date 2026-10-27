#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <sstream>

#include "state/state_events.h"
#include "state/state_store.h"

int main() {
    std::ostringstream output;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(output);
    auto logger = std::make_shared<spdlog::logger>("test", sink);
    logger->set_level(spdlog::level::trace);
    spdlog::set_default_logger(logger);

    plane::core::EventBus event_bus;
    plane::core::AppConfig config;
    plane::state::StateStore store(event_bus, config);
    int updates = 0;
    event_bus.subscribe<plane::model::VehicleState>(plane::state::events::kPeerStateChanged,
                                                    [&](const plane::model::VehicleState&) { ++updates; });
    plane::model::VehicleState state;
    state.planeId = "test-peer";
    for (int index = 0; index < 100; ++index) {
        state.airspeed_m_s = static_cast<float>(index);
        store.update_peer_state(state);
    }
    plane::model::VehicleState latest;
    if (updates != 100 || !store.peer_state(state.planeId, latest) || latest.airspeed_m_s != 99.0F) return 1;
    logger->flush();
    if (!output.str().empty()) return 2;
    return 0;
}
