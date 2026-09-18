#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::target {

/// 跟踪目标状态快照（M10）。
struct TrackingState {
    plane::model::DetectTarget target;  ///< 最后已知目标（valid=false 时无意义）
    bool valid = false;                 ///< 是否收到过跟踪结果
    bool lost = false;                  ///< 最新一次跟踪结果是否丢失
};

/// 目标统一查询接口（ARCHITECTURE §16/§22）。
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::DetectTarget> vision_targets() const = 0;
    virtual bool tracked_target(plane::model::DetectTarget& out) const = 0;
    virtual TrackingState tracking_state() const = 0;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;
};

}  // namespace plane::target
