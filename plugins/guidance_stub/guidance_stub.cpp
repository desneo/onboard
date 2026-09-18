#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
#include <memory>
#include <string>

#include "functional/guidance_api.h"

namespace plane::algorithm {
namespace {

class GuidanceStub final : public GuidancePlugin {
   public:
    bool init(const GuidanceConfig& config) override {
        config_ = config;
        const char* mode = std::getenv("PLANE_STUB_GUIDE");
        forward_ = (mode != nullptr && std::string(mode) == "forward");
        spdlog::info("GuidanceStub init: desired_distance_m={}, forward={}", config_.desired_distance_m, forward_);
        return true;
    }

    bool guide(const DetectTarget&, const VehicleState&, NedVelocity& out, std::string&) override {
        out = NedVelocity{};
        if (forward_) {
            out.north_m_s = 1.0;
        }
        return true;
    }

   private:
    GuidanceConfig config_;
    bool forward_ = false;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(GuidanceStub, GuidancePlugin);
PLUMA_PROVIDER_SOURCE(GuidancePlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new GuidanceStubProvider());
    return true;
}

}  // namespace plane::algorithm
