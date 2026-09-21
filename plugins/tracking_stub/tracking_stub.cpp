#include <spdlog/spdlog.h>

#include <Pluma/Connector.hpp>
#include <Pluma/Pluma.hpp>
#include <cstdlib>
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
        const char* lost_after = std::getenv("PLANE_STUB_TRACK_LOST_AFTER");
        if (lost_after != nullptr) {
            try {
                lost_after_ = std::stoi(lost_after);
            } catch (const std::exception&) {
                lost_after_ = 0;
            }
        }
        SPDLOG_INFO("TrackingStub init: model_path={}, lost_after={}", config_.model_path, lost_after_);
        return true;
    }

    bool start(const cv::Mat&, const DetectTarget& initTarget, std::string&) override {
        target_ = initTarget;
        return true;
    }

    bool track(const cv::Mat&, DetectTarget& out, std::string& error) override {
        ++call_count_;
        out = target_;
        if (lost_after_ > 0 && call_count_ > lost_after_) {
            error = "stub 模拟跟踪丢失";
            return false;
        }
        return true;
    }

   private:
    TrackConfig config_;
    DetectTarget target_;
    int lost_after_ = 0;
    int call_count_ = 0;
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
