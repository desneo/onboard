#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
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
        const char* mode = std::getenv("PLANE_STUB_DETECT");
        synthetic_center_ = (mode != nullptr && std::string(mode) == "center");
        SPDLOG_INFO("DetectionStub init: model_path={}, synthetic_center={}", config_.model_path, synthetic_center_);
        return true;
    }

    bool detect(const cv::Mat& frame, std::vector<DetectTarget>& out, std::string&) override {
        out.clear();
        if (synthetic_center_ && !frame.empty()) {
            DetectTarget target;
            target.type = "stub";
            target.confidence = 0.9F;
            target.bbox.width = 100;
            target.bbox.height = 100;
            target.bbox.x = frame.cols / 2 - target.bbox.width / 2;
            target.bbox.y = frame.rows / 2 - target.bbox.height / 2;
            out.push_back(target);
        }
        return true;
    }

   private:
    DetectConfig config_;
    bool synthetic_center_ = false;
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
