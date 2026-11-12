#pragma once

#include <string>
#include <vector>

#include "message.h"

namespace plane::target {

/// 目标统一查询接口（ARCHITECTURE §16/§22）。
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;
};

}  // namespace plane::target
