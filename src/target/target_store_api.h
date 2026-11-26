#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "message.h"

namespace plane::target {

struct GcsTargetUpdates {
    std::vector<plane::model::GcsTarget> history;
    std::optional<std::int64_t> last_received_ms;
};

/// 目标缓存查询与当前目标需求接口。
class TargetStoreApi {
   public:
    virtual ~TargetStoreApi() = default;

    virtual std::vector<plane::model::GcsTarget> gcs_targets() const = 0;
    virtual bool gcs_target(const std::string& track_id, plane::model::GcsTarget& out) const = 0;
    virtual std::vector<plane::model::GcsTarget> gcs_target_history(const std::string& track_id) const = 0;
    /// 最近一次被接受的目标更新的本机单调接收时间；从未收到时返回 nullopt。
    virtual std::optional<std::int64_t> gcs_target_last_received_ms(const std::string& track_id) const = 0;
    /// 当前进程内已接受目标更新的全局版本。
    virtual std::uint64_t gcs_target_revision() const = 0;
    /// 返回指定航迹在给定全局版本之后仍保留的有效更新及其最后接收时间。
    virtual GcsTargetUpdates gcs_target_updates_after(const std::string& track_id,
                                                      std::uint64_t after_revision) const = 0;

    /// nullopt 取消目标需求；空字符串表示自动选择，非空字符串指定航迹。
    virtual void set_required_gcs_target(std::optional<std::string> track_id) = 0;
    /// 仅将已启用的自动需求锁定到实际航迹，不重新启用已取消的需求。
    virtual void lock_required_gcs_target(const std::string& track_id) = 0;
};

}  // namespace plane::target
