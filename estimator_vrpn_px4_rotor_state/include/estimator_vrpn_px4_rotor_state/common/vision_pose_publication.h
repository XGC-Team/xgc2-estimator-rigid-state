#pragma once

#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state {
// Shared ROS/native publication policy. This is the existing ROS consumer's
// exact admission rule, independent of transport and filter implementation.
inline bool canPublishVisionPose(const VrpnPx4RotorStateEstimatorOutput& output) {
    constexpr uint32_t kVisionBlockingFlags = kVrpnMissing | kVrpnStale | kInvalidVrpn | kTimeJump |
                                              kPoseTimeAlignmentRejected | kVrpnFault |
                                              kFilterImuOnly;
    return output.has_corrected_vision_pose && (output.flags & kVisionBlockingFlags) == 0u;
}
}  // namespace estimator_vrpn_px4_rotor_state
