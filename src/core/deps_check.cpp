#include "core/deps_check.h"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <eventpp/eventdispatcher.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <GeographicLib/Geodesic.hpp>
#include <Pluma/Pluma.hpp>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <zenoh/api.hxx>

namespace plane::core {
namespace {

enum class CheckState { Ok, Failed };

}  // namespace

int run_deps_check() {
    const std::string state = std::string(magic_enum::enum_name(CheckState::Ok));
    const std::string json = nlohmann::json::parse(R"({"deps":"ok"})").dump();

    eventpp::EventDispatcher<int, void()> dispatcher;
    dispatcher.appendListener(1, []() {});
    dispatcher.dispatch(1);

    pluma::Pluma pluma;
    static_cast<void>(pluma);

    BT::BehaviorTreeFactory factory;
    static_cast<void>(factory);

    const GeographicLib::Geodesic& geodesic = GeographicLib::Geodesic::WGS84();
    double distance_m = 0.0;
    geodesic.Inverse(0.0, 0.0, 0.0, 0.001, distance_m);

    zenoh::Config zenoh_config = zenoh::Config::create_default();
    static_cast<void>(zenoh_config);

    const cv::Mat empty_frame;
    static_cast<void>(empty_frame);

    SPDLOG_INFO("deps-check: magic_enum={} nlohmann={} fmt={} spdlog={} bt={} geodesic={:.1f}m cv={}", state,
                NLOHMANN_JSON_VERSION_MAJOR, FMT_VERSION, SPDLOG_VERSION, BT::LibraryVersionString(), distance_m,
                CV_VERSION);
    return 0;
}

}  // namespace plane::core
