#pragma once

#include <string>

#include "message.h"

namespace plane::core {

/// 运行配置；第一版系统参数与 SystemParam 对齐（ARCHITECTURE §7）。
struct AppConfig {
    plane::model::SystemParam system_param;
    std::string log_dir = "logs";
    int state_report_period_ms = 1000;

    std::string plane_id = "uav1";
    std::string station_id = "gcs";
    std::string zenoh_config_file;
    std::string zenoh_connect;
    std::string zenoh_listen;
    int comm_ack_timeout_ms = 500;
    int comm_ack_max_retries = 3;
    int comm_rx_queue_max = 1024;
    int gcs_link_timeout_ms = 2000;

    std::string fc_link_type = "udp";
    std::string fc_udp_url = "udp://:14540";
    std::string fc_serial_device = "/dev/ttyUSB0";
    int fc_serial_baud = 57600;
    int fc_heartbeat_timeout_ms = 2000;
    double fc_route_speed_m_s = 5.0;

    /// 是否仿真飞机：udp 链路且 fc_udp_url 端口处于 14200-14299 时置为 true（仅机载内部使用）。
    bool is_sim = false;

    int bt_tick_interval_ms = 50;
    int bt_node_timeout_ms = 30000;
    double bt_takeoff_tolerance_m = 1.0;

    std::string plugins_dir = "build/plugins";
    std::string image_source;
    int image_reconnect_interval_ms = 2000;
    int perception_process_interval_ms = 100;
    bool perception_selftest_on_start = false;

    int rtsp_port = 8554;
    std::string rtsp_app = "live";
    std::string rtsp_stream = "onboard";
    int video_fps = 25;
    int video_bitrate_kbps = 2000;
    bool video_overlay_enabled = true;
    double alert_voltage_threshold_v = 21.0;

    double suicide_detection_start_distance_m = 200.0;
    double suicide_approach_speed_m_s = 10.0;
    double suicide_track_approach_speed_m_s = 5.0;
    double suicide_climb_speed_m_s = 2.0;
    std::string suicide_target_select_rule = "nearest_center";
    int suicide_guidance_start_stable_ms = 500;
    int suicide_track_lost_timeout_ms = 3000;
    int suicide_phase_timeout_ms = 300000;
    double suicide_impact_height_m = 2.0;
    double suicide_impact_vd_m_s = 3.0;
};

/// 从 key = value 文本文件加载配置；文件不存在时保留默认值并返回 false。
bool load_config(const std::string& path, AppConfig& out);

}  // namespace plane::core
