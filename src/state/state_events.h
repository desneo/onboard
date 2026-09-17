#pragma once

#include <string_view>

namespace plane::state {
namespace events {

inline constexpr std::string_view kOwnStateChanged = "state.own_changed";
inline constexpr std::string_view kPeerStateChanged = "state.peer_changed";
inline constexpr std::string_view kGcsLinkChanged = "state.gcs_link_changed";

}  // namespace events
}  // namespace plane::state
