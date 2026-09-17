#pragma once

#include <eventpp/eventdispatcher.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace plane::core {

/// 进程内异步事件总线（ARCHITECTURE §3.2/§22）。
/// 事件标识为字符串常量；载荷类型由 subscribe<T>/publish<T> 决定。
class EventBus {
   public:
    template <typename T>
    void subscribe(std::string_view event_id, std::function<void(const T&)> handler) {
        channel<T>().dispatcher.appendListener(std::string(event_id), std::move(handler));
    }

    template <typename T>
    void publish(std::string_view event_id, const T& event) {
        channel<T>().dispatcher.dispatch(std::string(event_id), event);
    }

   private:
    struct ChannelBase {
        virtual ~ChannelBase() = default;
    };

    template <typename T>
    struct Channel final : ChannelBase {
        eventpp::EventDispatcher<std::string, void(const T&)> dispatcher;
    };

    template <typename T>
    Channel<T>& channel() {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::type_index key(typeid(T));
        auto it = channels_.find(key);
        if (it == channels_.end()) {
            auto channel = std::make_unique<Channel<T>>();
            Channel<T>* raw = channel.get();
            channels_.emplace(key, std::move(channel));
            return *raw;
        }
        return static_cast<Channel<T>&>(*it->second);
    }

    std::mutex mutex_;
    std::unordered_map<std::type_index, std::unique_ptr<ChannelBase>> channels_;
};

}  // namespace plane::core
