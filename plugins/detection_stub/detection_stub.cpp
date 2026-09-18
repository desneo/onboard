#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "functional/detection_api.h"

namespace plane::algorithm {
namespace {

class DetectionStub final : public DetectionPlugin {
   public:
    bool init(const DetectConfig& config) override {
        config_ = config;
        spdlog::info("DetectionStub init: model_path={}", config_.model_path);
        return true;
    }

    bool detect(const cv::Mat&, std::vector<DetectTarget>& out, std::string&) override {
        out.clear();
        return true;
    }

   private:
    DetectConfig config_;
};

}  // namespace

PLUMA_INHERIT_PROVIDER(DetectionStub, DetectionPlugin);
PLUMA_PROVIDER_SOURCE(DetectionPlugin, 1, 1);

PLUMA_CONNECTOR
bool connectToPlumaHost(pluma::Host& host) {
    host.add(new DetectionStubProvider());
    return true;
}

}  // namespace plane::algorithm
