#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "camera/pod_transport.h"
#include "message.h"

namespace plane::camera {

struct PodTelemetry {
    std::map<std::string, plane::model::PodStateField> fields;
};

/// 单个型号的吊舱驱动，由相机工作线程串行调用。
class PodDriver {
   public:
    virtual ~PodDriver() = default;

    virtual bool can_move() const = 0;
    virtual bool start(std::string& error) = 0;
    /// 处理输入帧和周期协议操作，只在收到新有效帧时返回测量。
    virtual std::optional<PodTelemetry> tick(std::chrono::steady_clock::time_point now) = 0;
    /// 返回是否提交给驱动。设备实际执行情况由遥测确认。
    virtual bool apply(const plane::model::PodCommand& command, std::chrono::steady_clock::time_point now,
                       std::string& error) = 0;
    virtual void stop() = 0;
};

std::unique_ptr<PodDriver> make_pod_driver(std::string_view model, std::unique_ptr<PodTransport> transport,
                                          std::string& error);

}  // namespace plane::camera
