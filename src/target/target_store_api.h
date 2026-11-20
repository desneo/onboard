#pragma once

#include <optional>
#include <string>
#include <vector>

#include "message.h"

namespace plane::target {

/// 目标缓存查询与当前目标需求接口。
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;

    /// nullopt 取消目标需求；空字符串表示自动选择，非空字符串指定航迹。
    virtual void set_required_gcs_target(std::optional<std::string> track_id) = 0;
    /// 仅将已启用的自动需求锁定到实际航迹，不重新启用已取消的需求。
    virtual void lock_required_gcs_target(const std::string& track_id) = 0;
};

}  // namespace plane::target
