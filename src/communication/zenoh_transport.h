#pragma once

#include <functional>
#include <memory>
#include <string>

namespace plane::communication {

/// Zenoh 连接参数（来自 AppConfig）。
struct ZenohEndpoints {
    std::string config_file;
    std::string connect;
    std::string listen;
    bool debug_log = false;
};

/// Zenoh 最小封装：Session 生命周期、订阅与发送。
/// 设计约束：业务模块不直接依赖 Zenoh；Zenoh 头文件仅出现在本类实现中。
class ZenohTransport {
   public:
    using SampleHandler = std::function<void(const std::string& payload)>;
    using LivelinessHandler = std::function<void(const std::string& key_expr, bool online)>;

    ZenohTransport();
    ~ZenohTransport();
    ZenohTransport(const ZenohTransport&) = delete;
    ZenohTransport& operator=(const ZenohTransport&) = delete;

    bool open(const ZenohEndpoints& endpoints, std::string& error);
    void close();
    bool subscribe(const std::string& key_expr, SampleHandler handler, std::string& error);
    bool subscribe_liveliness(const std::string& key_expr, LivelinessHandler handler, std::string& error);
    bool declare_liveliness_token(const std::string& key_expr, std::string& error);
    bool put(const std::string& key_expr, const std::string& payload, std::string& error);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace plane::communication
