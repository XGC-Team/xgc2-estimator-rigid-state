#pragma once

#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state {
// Publication policy of the corrected vision pose: the ROS consumer's exact
// admission rule, independent of the filter implementation.
inline bool canPublishVisionPose(const VrpnPx4RotorStateEstimatorOutput& output) {
    constexpr uint32_t kVisionBlockingFlags = kVrpnMissing | kVrpnStale | kInvalidVrpn | kTimeJump |
                                              kPoseTimeAlignmentRejected | kVrpnFault |
                                              kFilterImuOnly;
    return output.has_corrected_vision_pose && (output.flags & kVisionBlockingFlags) == 0u;
}
}  // namespace estimator_vrpn_px4_rotor_state
