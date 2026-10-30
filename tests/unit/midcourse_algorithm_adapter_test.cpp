#ifdef NDEBUG
#undef NDEBUG
#endif
#include "task/nodes/midcourse_algorithm_adapter.h"

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

plane::task::MidcourseMeasurement measurement(int index) {
    plane::task::MidcourseMeasurement result;
    result.geo.latitude = 30.010 + 0.00001 * static_cast<double>(index);
    result.geo.longitude = 120.0;
    result.geo.altitude = 300.0F;
    result.timestamp_ms = 1'800'000'000'000LL + static_cast<std::int64_t>(index) * 1000;
    return result;
}

void run_session() {
    using namespace plane::task;

    std::vector<MidcourseMeasurement> history;
    for (int index = 0; index < 20; ++index) history.push_back(measurement(index));

    MidcourseGeoPoint reference;
    reference.latitude = 30.0;
    reference.longitude = 120.0;
    reference.altitude = 100.0F;

    MidcourseAlgorithmAdapter adapter;
    std::string error;
    assert(adapter.init_estimator(history, reference, 1.0, error));

    MidcourseAlgorithmConfig config;
    config.max_horizontal_speed_m_s = 15.0;
    config.max_climb_speed_m_s = 5.0;
    config.max_descent_speed_m_s = 5.0;
    config.max_acceleration_m_s2 = 6.0;
    config.update_dt_s = 0.05;
    config.target_invalid_grace_s = 3.0;
    config.r_min_m = 50.0;
    assert(adapter.set_config(config, error));

    MidcourseVehicleState vehicle;
    vehicle.geo = reference;
    vehicle.start = reference;

    // 第 21 帧到来前允许无效估计，规划器必须正常返回 Attitude 悬停指令。
    MidcourseAttitudeOutput output;
    assert(adapter.update(vehicle, output, error));
    assert(std::isfinite(output.thrust_body[2]));

    assert(adapter.update_target(measurement(20), error));
    assert(adapter.update(vehicle, output, error));
    double norm_squared = 0.0;
    for (double value : output.quat_body_to_ned) {
        assert(std::isfinite(value));
        norm_squared += value * value;
    }
    for (double value : output.thrust_body) assert(std::isfinite(value));
    assert(std::fabs(std::sqrt(norm_squared) - 1.0) < 1e-6);
}

}  // namespace

int main() {
    std::ostringstream log_output;
    auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log_output);
    auto logger = std::make_shared<spdlog::logger>("中制导日志格式测试", sink);
    logger->set_pattern("%v");
    spdlog::set_default_logger(logger);

    run_session();
    run_session();  // 新会话必须能完整重置算法状态。

    const std::string logs = log_output.str();
    assert(logs.find("TargetEstimator::init 输入") != std::string::npos);
    assert(logs.find("ned_ref={latitude=30.000000,longitude=120.000000,height=0.00,altitude=100.00}") !=
           std::string::npos);
    assert(logs.find("measurement={geo={latitude=30.010000,longitude=120.000000") != std::string::npos);
    assert(logs.find("MidcoursePlanner::setConfig 输入") != std::string::npos);
    assert(logs.find("config={max_horizontal_speed_m_s=15.00,max_climb_speed_m_s=5.00") != std::string::npos);
    assert(logs.find("MidcoursePlanner::update 输入") != std::string::npos);
    assert(logs.find("vn_m_s=0.00,ve_m_s=0.00,vd_m_s=0.00") != std::string::npos);
    assert(logs.find("MidcoursePlanner::update 输出") != std::string::npos);
    assert(logs.find("中制导会话创建: session=") != std::string::npos);
    assert(logs.find("中制导会话销毁: session=") != std::string::npos);
    assert(logs.find("calls=") != std::string::npos);
    assert(logs.find("estimator_updates=") != std::string::npos);
    assert(logs.find("planner_updates=") != std::string::npos);
    assert(logs.find(" 输入 [会话=") == std::string::npos);
    assert(logs.find(" 输出 [会话=") == std::string::npos);
    assert(logs.find("校验失败 [会话=") == std::string::npos);
    assert(logs.find("中制导接口 方向=") == std::string::npos);
    assert(logs.find("最大水平速度米每秒") == std::string::npos);
    assert(logs.find("纬度=") == std::string::npos);
    return 0;
}
