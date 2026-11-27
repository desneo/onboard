#pragma once

#include <string>
#include <vector>

#include "algorithm/midcourse_alg.h"
#include "core/log.h"
#include "message.h"

namespace plane::core {

inline constexpr int kDefaultPodStateReportHz = 5;
inline constexpr int kDefaultMidcourseTargetHistorySize = 20;
inline constexpr int kMinMidcourseTargetHistorySize = 5;
inline constexpr int kMaxMidcourseTargetHistorySize = 30;
inline constexpr int kDefaultMidcourseTargetTimeoutMs = 5000;

/// 运行配置；第一版系统参数与 SystemParam 对齐（ARCHITECTURE §7）。
struct AppConfig {
    plane::model::SystemParam system_param;
    std::string log_dir = "logs";
    int log_flush_interval_ms = kDefaultLogFlushIntervalMs;
    int state_report_period_ms = 1000;

    std::string board_ip;
    std::string pod_model;
    std::string pod_video_url;
    std::string pod_control_url;
    int pod_state_report_hz = kDefaultPodStateReportHz;

    std::string plane_id;
    std::string zenoh_config_file;
    std::string zenoh_connect;
    std::string zenoh_listen;
    bool debug_zenoh_log = false;  ///< Zenoh 调试日志：打印每条消息发送的目标端点。
    int comm_ack_timeout_ms = 500;
    int comm_ack_max_retries = 3;
    int comm_rx_queue_max = 1024;
    int gcs_link_timeout_ms = 2000;

    std::string fc_link_type;
    std::string fc_udp_url;
    std::string fc_serial_device;
    int fc_serial_baud = 0;
    int fc_heartbeat_timeout_ms = 5000;
    double fc_route_speed_m_s = 5.0;

    /// 是否仿真飞机：udp 链路且 fc_udp_url 端口处于场景 02 的 14030-14039／14280-14289
    /// 或旧 SITL 的 14580-14589 时置为 true，同时用于 AgentInfo.isSim。
    bool is_sim = false;

    int bt_tick_interval_ms = 50;
    int bt_node_timeout_ms = 600000;
    double bt_height_adjust_tolerance_m = 1.0;

    double alert_voltage_threshold_v = 21.0;

    int midcourse_init_timeout_ms = 30000;
    int midcourse_target_history_size = kDefaultMidcourseTargetHistorySize;
    int midcourse_target_timeout_ms = kDefaultMidcourseTargetTimeoutMs;
    bool midcourse_use_px4_limits = false;
    plane::algorithm::MidCourseConfig midcourse_algorithm = [](int tick_interval_ms) {
        plane::algorithm::MidCourseConfig config;
        config.max_horizontal_speed_m_s = 60.0;
        config.max_climb_speed_m_s = 20.0;
        config.max_descent_speed_m_s = 20.0;
        config.cmd_accel_max_m_s2 = 20.0;
        config.update_dt_s = tick_interval_ms / 1000.0;
        config.target_invalid_grace_s = 3.0;
        config.control_output_mode = plane::algorithm::MidcourseControlMode::VelocityAcceleration;
        config.yaw_mode = 1;
        config.enable_internal_terminal_confirmation = false;
        return config;
    }(bt_tick_interval_ms);
};

/// 加载主配置和必填的部署预置。加载失败时保留原配置并返回 false。
/// 未提供地址列表时，从本机网卡获取地址以匹配预置中的唯一飞机。
bool load_config(const std::string& path, AppConfig& out,
                 const std::vector<std::string>* local_ipv4_addresses = nullptr);

}  // namespace plane::core
