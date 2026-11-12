#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace plane::camera {

using PodFrame = std::vector<std::uint8_t>;

/// 传输层负责收发候选帧，型号驱动负责验证帧和解释内容。
class PodTransport {
   public:
    virtual ~PodTransport() = default;

    virtual bool start(std::string& error) = 0;
    virtual bool send(const PodFrame& frame, std::string& error) = 0;
    /// 非阻塞读取，没有待处理帧时返回 false。
    virtual bool receive(PodFrame& frame) = 0;
    virtual void stop() = 0;
};

std::unique_ptr<PodTransport> make_pod_transport(std::string_view url, std::string& error);

}  // namespace plane::camera
