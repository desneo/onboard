#pragma once

#include <string>

#include "message.h"

namespace plane::core {

/// 运行配置；第一版系统参数与 SystemParam 对齐（ARCHITECTURE §7）。
struct AppConfig {
    plane::model::SystemParam system_param;
    std::string log_dir = "logs";
    int state_report_period_ms = 1000;

    std::string board_ip;
    std::string pod_video_url;

    std::string plane_id = "uav1";
    std::string station_id = "gcs";
    std::string zenoh_config_file;
    std::string zenoh_connect;
    std::string zenoh_listen;
    bool debug_zenoh_log = false;  ///< Zenoh 调试日志：打印每条消息发送的目标端点。
    int comm_ack_timeout_ms = 500;
    int comm_ack_max_retries = 3;
    int comm_rx_queue_max = 1024;
    int gcs_link_timeout_ms = 2000;

    std::string fc_link_type = "udp";
    std::string fc_udp_url = "udp://:14540";
    std::string fc_serial_device = "/dev/ttyUSB0";
    int fc_serial_baud = 57600;
    int fc_heartbeat_timeout_ms = 5000;
    double fc_route_speed_m_s = 5.0;

    /// 是否仿真飞机：udp 链路且 fc_udp_url 端口处于 14580-14589 时置为 true，同时用于 VehicleInfo.isSim。
    bool is_sim = false;

    int bt_tick_interval_ms = 50;
    int bt_node_timeout_ms = 30000;
    double bt_takeoff_tolerance_m = 1.0;
    int bt_height_adjust_timeout_ms = 120000;
    double bt_height_adjust_tolerance_m = 1.0;

    double alert_voltage_threshold_v = 21.0;

    int midcourse_init_timeout_ms = 30000;
    double midcourse_target_invalid_grace_s = 3.0;
    double midcourse_r_min_m = 50.0;
};

/// 从带 section 的 INI 文件加载配置；文件不存在时保留默认值并返回 false。
bool load_config(const std::string& path, AppConfig& out);

}  // namespace plane::core
