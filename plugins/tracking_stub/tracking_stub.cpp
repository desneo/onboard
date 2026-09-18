#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

#include "functional/tracking_api.h"

namespace plane::algorithm {
namespace {

class TrackingStub final : public TrackingPlugin {
   public:
    bool init(const TrackConfig& config) override {
        config_ = config;
        spdlog::info("TrackingStub init: model_path={}", config_.model_path);
        return true;
    }

    bool start(const cv::Mat&, const DetectTarget& initTarget, std::string&) override {
        target_ = initTarget;
        return true;
    }

    bool track(const cv::Mat&, DetectTarget& out, std::string&) override {
        out = target_;
        return true;
    }

   private:
    TrackConfig config_;
    DetectTarget target_;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(TrackingStub, TrackingPlugin);
PLUMA_PROVIDER_SOURCE(TrackingPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new TrackingStubProvider());
    return true;
}

}  // namespace plane::algorithm
