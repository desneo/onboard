#include "communication/zenoh_transport.h"

#include <spdlog/spdlog.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <zenoh/api.hxx>

namespace plane::communication {
namespace {

std::string endpoints_json5_array(const std::string& comma_separated) {
    std::string result = "[";
    bool first = true;
    std::size_t begin = 0;
    while (begin <= comma_separated.size()) {
        const std::size_t end = comma_separated.find(',', begin);
        std::string item = comma_separated.substr(begin, end - begin);
        const auto not_space = item.find_first_not_of(" \t");
        const auto last = item.find_last_not_of(" \t");
        item = not_space == std::string::npos ? std::string{} : item.substr(not_space, last - not_space + 1);
        if (!item.empty()) {
            if (!first) {
                result += ",";
            }
            result += "\"" + item + "\"";
            first = false;
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    result += "]";
    return result;
}

std::string canon_key_expr(const std::string& key_expr) {
    // Zenoh 1.x 的 key expression 不允许前导 '/'（规范形式要求首个 chunk 非空）。
    // M1 设计中的 topic 常量以 '/' 开头，这里在 Zenoh 边界统一去掉前导 '/'。
    const std::size_t first = key_expr.find_first_not_of('/');
    return first == std::string::npos ? std::string{} : key_expr.substr(first);
}

std::string join_ids(const std::vector<zenoh::Id>& ids) {
    std::string result;
    for (const auto& id : ids) {
        if (!result.empty()) {
            result += ",";
        }
        result += id.to_string();
    }
    return result;
}

std::string endpoints_or_default(const std::string& endpoints) {
    return endpoints.empty() ? std::string("(未配置, 默认/组播发现)") : endpoints;
}

}  // namespace

struct ZenohTransport::Impl {
    std::optional<zenoh::Session> session;
    std::vector<zenoh::Subscriber<void>> subscribers;
    std::vector<zenoh::LivelinessToken> liveliness_tokens;
    bool debug_log = false;
    std::string remote_endpoints;
};

ZenohTransport::ZenohTransport() = default;
ZenohTransport::~ZenohTransport() = default;

bool ZenohTransport::open(const ZenohEndpoints& endpoints, std::string& error) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    impl_->debug_log = endpoints.debug_log;
    impl_->remote_endpoints = endpoints.connect;
    try {
        zenoh::Config config = endpoints.config_file.empty() ? zenoh::Config::create_default()
                                                             : zenoh::Config::from_file(endpoints.config_file);
        if (!endpoints.connect.empty()) {
            config.insert_json5("connect/endpoints", endpoints_json5_array(endpoints.connect));
        }
        if (!endpoints.listen.empty()) {
            config.insert_json5("listen/endpoints", endpoints_json5_array(endpoints.listen));
        }
        zenoh::ZResult result = Z_OK;
        zenoh::Session session =
            zenoh::Session::open(std::move(config), zenoh::Session::SessionOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "Zenoh Session 打开失败";
            return false;
        }
        impl_->session.emplace(std::move(session));

        std::string routers;
        std::string peers;
        try {
            routers = join_ids(impl_->session->get_routers_z_id());
            peers = join_ids(impl_->session->get_peers_z_id());
        } catch (const zenoh::ZException& e) {
            SPDLOG_WARN("Zenoh 已上线但获取对端信息失败: {}", e.what());
        }
        SPDLOG_INFO(
            "Zenoh 已上线: 远端端点(connect)=[{}], 本地监听(listen)=[{}], 本机 zid={}, routers=[{}], peers=[{}]",
            endpoints_or_default(endpoints.connect), endpoints_or_default(endpoints.listen),
            impl_->session->get_zid().to_string(), routers, peers);
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

void ZenohTransport::close() {
    if (!impl_) {
        return;
    }
    impl_->liveliness_tokens.clear();
    impl_->subscribers.clear();
    impl_->session.reset();
}

bool ZenohTransport::subscribe(const std::string& key_expr, SampleHandler handler, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    try {
        zenoh::ZResult result = Z_OK;
        zenoh::Subscriber<void> subscriber = impl_->session->declare_subscriber(
            zenoh::KeyExpr(canon_key_expr(key_expr)),
            [handler = std::move(handler)](zenoh::Sample& sample) { handler(sample.get_payload().as_string()); },
            []() {}, zenoh::Session::SubscriberOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "订阅失败: " + key_expr;
            return false;
        }
        impl_->subscribers.push_back(std::move(subscriber));
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

bool ZenohTransport::subscribe_liveliness(const std::string& key_expr, LivelinessHandler handler, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    try {
        auto options = zenoh::Session::LivelinessSubscriberOptions::create_default();
        options.history = true;
        zenoh::ZResult result = Z_OK;
        auto subscriber = impl_->session->liveliness_declare_subscriber(
            zenoh::KeyExpr(canon_key_expr(key_expr)),
            [handler = std::move(handler)](zenoh::Sample& sample) {
                handler(std::string(sample.get_keyexpr().as_string_view()), sample.get_kind() == Z_SAMPLE_KIND_PUT);
            },
            []() {}, std::move(options), &result);
        if (result != Z_OK) {
            error = "存活性订阅失败: " + key_expr;
            return false;
        }
        impl_->subscribers.push_back(std::move(subscriber));
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

bool ZenohTransport::declare_liveliness_token(const std::string& key_expr, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    try {
        zenoh::ZResult result = Z_OK;
        auto token = impl_->session->liveliness_declare_token(
            zenoh::KeyExpr(canon_key_expr(key_expr)), zenoh::Session::LivelinessDeclarationOptions::create_default(),
            &result);
        if (result != Z_OK) {
            error = "存活标记声明失败: " + key_expr;
            return false;
        }
        impl_->liveliness_tokens.push_back(std::move(token));
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

bool ZenohTransport::put(const std::string& key_expr, const std::string& payload, std::string& error) {
    if (!impl_ || !impl_->session.has_value()) {
        error = "Session 未打开";
        return false;
    }
    if (impl_->debug_log) {
        SPDLOG_INFO("[zenoh:debug] put topic={} -> 远端端点=[{}], payload={}", key_expr,
                    endpoints_or_default(impl_->remote_endpoints), payload);
    }
    try {
        zenoh::ZResult result = Z_OK;
        impl_->session->put(zenoh::KeyExpr(canon_key_expr(key_expr)), zenoh::Bytes(payload),
                            zenoh::Session::PutOptions::create_default(), &result);
        if (result != Z_OK) {
            error = "发送失败: " + key_expr;
            return false;
        }
        return true;
    } catch (const zenoh::ZException& e) {
        error = e.what();
        return false;
    }
}

}  // namespace plane::communication
