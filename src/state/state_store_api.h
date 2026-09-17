#pragma once

#include <string>
#include <vector>

#include "message.h"
#include "state/state_types.h"

namespace plane::state {

/// 状态管理对外稳定接口：查询 + 写入（写入仅供数据来源模块调用）。
class StateStoreApi {
   public:
    virtual ~StateStoreApi() = default;

    virtual plane::model::VehicleState own_state() const = 0;
    virtual std::vector<plane::model::VehicleState> peer_states() const = 0;
    virtual bool peer_state(const std::string& plane_id, plane::model::VehicleState& out) const = 0;
    virtual GcsLinkState gcs_link() const = 0;

    virtual void update_own_state(const plane::model::VehicleState& state) = 0;
    virtual void update_peer_state(const plane::model::VehicleState& state) = 0;
    virtual void notify_station_seen() = 0;
};

}  // namespace plane::state
