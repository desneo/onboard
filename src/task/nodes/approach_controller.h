#pragma once

#include <string>

#include "core/config.h"
#include "flight_control/flight_control_api.h"
#include "state/state_store_api.h"
#include "target/target_store_api.h"

namespace plane::task {

/// suicide 各阶段共享的逼近控制：解析 GCS 目标、计算距离、下发逼近速度。
class ApproachController {
   public:
    ApproachController(plane::flight_control::FlightControlApi* flight_control,
                       plane::state::StateStoreApi* state_store, plane::target::TargetStoreApi* target_store,
                       const plane::core::AppConfig* app_config);

    /// 解析目标：track_id 非空按 id 查询；为空且仅有一个 GCS 目标时使用该目标。
    bool resolve_target(const std::string& track_id, plane::model::GcsTarget& out) const;

    /// 到目标的地面水平距离（米）。
    double horizontal_distance_m(const plane::model::GcsTarget& target) const;

    /// 朝目标下发 NED 速度（水平 speed，垂直按高度差限幅，机头指向目标）。
    bool approach(const plane::model::GcsTarget& target, double speed_m_s, std::string& error);

   private:
    plane::flight_control::FlightControlApi* flight_control_;
    plane::state::StateStoreApi* state_store_;
    plane::target::TargetStoreApi* target_store_;
    const plane::core::AppConfig* app_config_;
};

}  // namespace plane::task
