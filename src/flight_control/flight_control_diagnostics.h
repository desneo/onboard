#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "message.h"

namespace plane::flight_control {

/// 飞控诊断只记录连接内的文本去重、状态基线和 Offboard 发送故障边沿。
/// 返回的报文由调用方发布，持有本类互斥锁时不调用外部模块。
class FlightControlDiagnostics {
   public:
    using Clock = std::chrono::steady_clock;

    std::optional<plane::model::Alert> link(std::uint64_t generation, bool connected) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation < generation_) return std::nullopt;
        if (generation != generation_ || !connected_ || *connected_ != connected) {
            reset_connection_locked();
        }
        generation_ = generation;
        return state_locked("fcLinkState", connected ? "connected" : "disconnected", connected_, connected);
    }

    std::optional<plane::model::Alert> status_text(std::uint64_t generation, const std::string& type,
                                                 const std::string& text, Clock::time_point now = Clock::now()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_connection_locked(generation) || text.empty() || !reportable_status_type(type)) {
            return std::nullopt;
        }
        for (auto it = recent_status_texts_.begin(); it != recent_status_texts_.end();) {
            if (now - it->second >= kStatusTextWindow) {
                it = recent_status_texts_.erase(it);
            } else {
                ++it;
            }
        }
        const auto key = type + '\n' + text;
        if (recent_status_texts_.find(key) != recent_status_texts_.end()) return std::nullopt;
        if (recent_status_texts_.size() >= kStatusTextLimit) {
            auto oldest = recent_status_texts_.begin();
            for (auto it = recent_status_texts_.begin(); it != recent_status_texts_.end(); ++it) {
                if (it->second < oldest->second) oldest = it;
            }
            recent_status_texts_.erase(oldest);
        }
        recent_status_texts_.emplace(key, now);
        return plane::model::Alert{"flight_control_status_text", "飞控 STATUSTEXT：type=" + type + " text=" + text};
    }

    std::optional<plane::model::Alert> flight_mode(std::uint64_t generation, const std::string& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_connection_locked(generation) || value.empty() || value == "Unknown") return std::nullopt;
        if (flight_mode_ && *flight_mode_ == value) return std::nullopt;
        if (!flight_mode_) {
            flight_mode_ = value;
            return plane::model::Alert{"flight_control_state_changed", "飞控当前状态：flightMode=" + value};
        }
        const std::string previous = *flight_mode_;
        flight_mode_ = value;
        return plane::model::Alert{"flight_control_state_changed",
                                   "飞控状态变化：flightMode previous=" + previous + " value=" + value};
    }

    std::optional<plane::model::Alert> armed(std::uint64_t generation, bool value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_connection_locked(generation)) return std::nullopt;
        return state_locked("armed", value ? "true" : "false", armed_, value);
    }

    std::optional<plane::model::Alert> inair(std::uint64_t generation, bool value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_connection_locked(generation)) return std::nullopt;
        return state_locked("inair", value ? "true" : "false", inair_, value);
    }

    /// 每个设定值种类只报告首个错误、错误原因变化和首次恢复。
    std::optional<plane::model::Alert> offboard_result(std::uint64_t generation, const std::string& operation,
                                                     const std::string& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_) return std::nullopt;
        const auto it = offboard_errors_.find(operation);
        if (error.empty()) {
            // 非法新输入不会覆盖旧设定值；继续成功发送旧值不能解除新输入故障。
            if (invalid_offboard_inputs_.count(operation) != 0) return std::nullopt;
            if (it == offboard_errors_.end()) return std::nullopt;
            offboard_errors_.erase(it);
            return plane::model::Alert{"offboard_send_recovered", "飞控 Offboard 设定值下发恢复：" + operation};
        }
        if (it != offboard_errors_.end() && it->second == error) return std::nullopt;
        offboard_errors_[operation] = error;
        return plane::model::Alert{"flight_control_command_failed", error};
    }

    void offboard_input_validity(std::uint64_t generation, const std::string& operation, bool valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_) return;
        if (valid) {
            invalid_offboard_inputs_.erase(operation);
        } else {
            invalid_offboard_inputs_.insert(operation);
        }
    }

    /// 停止会话只清除故障锁存，不将主动停止误报为发送恢复。
    void clear_offboard() {
        std::lock_guard<std::mutex> lock(mutex_);
        offboard_errors_.clear();
        invalid_offboard_inputs_.clear();
    }

   private:
    static constexpr auto kStatusTextWindow = std::chrono::seconds(5);
    static constexpr std::size_t kStatusTextLimit = 256;

    static bool reportable_status_type(const std::string& type) {
        return type == "Info" || type == "Notice" || type == "Warning" || type == "Error" || type == "Critical" ||
               type == "Alert" || type == "Emergency";
    }

    bool current_connection_locked(std::uint64_t generation) const {
        return generation == generation_ && connected_.value_or(false);
    }

    std::optional<plane::model::Alert> state_locked(const std::string& field, const std::string& text,
                                                  std::optional<bool>& baseline, bool value) {
        if (baseline && *baseline == value) return std::nullopt;
        if (!baseline) {
            baseline = value;
            return plane::model::Alert{"flight_control_state_changed", "飞控当前状态：" + field + "=" + text};
        }
        const std::string previous = field == "fcLinkState" ? (*baseline ? "connected" : "disconnected")
                                                           : (*baseline ? "true" : "false");
        baseline = value;
        return plane::model::Alert{"flight_control_state_changed",
                                   "飞控状态变化：" + field + " previous=" + previous + " value=" + text};
    }

    void reset_connection_locked() {
        recent_status_texts_.clear();
        flight_mode_.reset();
        armed_.reset();
        inair_.reset();
        offboard_errors_.clear();
        invalid_offboard_inputs_.clear();
    }

    std::mutex mutex_;
    std::uint64_t generation_ = 0;
    std::optional<bool> connected_;
    std::optional<std::string> flight_mode_;
    std::optional<bool> armed_;
    std::optional<bool> inair_;
    std::unordered_map<std::string, Clock::time_point> recent_status_texts_;
    std::unordered_map<std::string, std::string> offboard_errors_;
    std::unordered_set<std::string> invalid_offboard_inputs_;
};

}  // namespace plane::flight_control
