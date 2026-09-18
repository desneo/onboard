#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <memory>
#include <string>

#include "functional/guidance_api.h"

namespace plane::algorithm {
namespace {

class GuidanceStub final : public GuidancePlugin {
   public:
    bool init(const GuidanceConfig& config) override {
        config_ = config;
        spdlog::info("GuidanceStub init: desired_distance_m={}", config_.desired_distance_m);
        return true;
    }

    bool guide(const DetectTarget&, const VehicleState&, NedVelocity& out, std::string&) override {
        out = NedVelocity{};
        return true;
    }

   private:
    GuidanceConfig config_;
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
