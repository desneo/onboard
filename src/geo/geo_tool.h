#pragma once

#include "message.h"

namespace plane::geo {

/// 统一地理计算工具（ARCHITECTURE §21/ADR-012）；GeographicLib 类型不出本模块。
class GeoCalculator {
   public:
    /// 两点间大地线距离（米）。
    static double distance_m(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);

    /// from 到 to 的方位角（度，北 0，顺时针，0–360）。
    static double bearing_deg(const plane::model::GeoPoint& from, const plane::model::GeoPoint& to);
};

}  // namespace plane::geo
