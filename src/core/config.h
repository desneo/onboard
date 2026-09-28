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

    /// 是否仿真飞机：udp 链路且 fc_udp_url 端口处于 14200-14299 时置为 true（仅机载内部使用）。
    bool is_sim = false;

    int bt_tick_interval_ms = 50;
    int bt_node_timeout_ms = 30000;
    double bt_takeoff_tolerance_m = 1.0;

    std::string plugins_dir = "build/plugins";

    /// 目标调试日志：按需打开逐帧高频日志（默认关闭）。
    bool debug_target_log = false;

    // 取图插件（image_input）
    std::string image_source_type = "Mipi";  ///< Mipi | Rtsp
    std::string image_mipi_device;           ///< 空=自动发现
    std::string image_rtsp_url = "rtsp://192.168.1.220:8554/drone";
    int image_reconnect_interval_s = 10;
    int image_width = 640;   ///< 读流尺寸（MIPI 采集宽），传给取图插件
    int image_height = 640;  ///< 读流尺寸（MIPI 采集高），传给取图插件

    // 检测插件（det_mot）
    std::string det_model_path = "models/yolo11/yolo11n";
    std::string det_config_dir = "/home/firefly/code/projects/anti-uav/algorithm/perception/configs";
    float det_confidence_threshold = 0.4F;
    float det_nms_threshold = 0.45F;
    float det_track_thresh = 0.5F;
    int det_track_buffer = 30;
    int det_input_width = 640;   ///< 检测模型输入宽：读流后统一 resize 到该尺寸
    int det_input_height = 640;  ///< 检测模型输入高：读流后统一 resize 到该尺寸

    // 推流插件（videostream_output）
    std::string stream_type = "WebRtc";  ///< WebRtc | Rtsp
    std::string stream_url;
    std::string stream_app = "live";
    std::string stream_name = "stream";
    int stream_rtsp_port = 8554;
    int stream_http_port = 80;
    int stream_rtc_port = 8000;
    int stream_width = 1280;
    int stream_height = 720;
    int stream_max_fps = 30;
    int stream_bitrate_kbps = 4096;
    int stream_gop = 30;

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
