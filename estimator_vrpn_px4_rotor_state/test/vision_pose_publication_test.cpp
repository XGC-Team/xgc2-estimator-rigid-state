#include <gtest/gtest.h>
#include "estimator_vrpn_px4_rotor_state/common/vision_pose_publication.h"
namespace estimator_vrpn_px4_rotor_state {
TEST(VisionPosePublication, RequiresCorrectedPoseAndRejectsEachOriginalBlockingFlag) {
    VrpnPx4RotorStateEstimatorOutput output{};
    EXPECT_FALSE(canPublishVisionPose(output));
    output.has_corrected_vision_pose = true;
    EXPECT_TRUE(canPublishVisionPose(output));
    for (uint32_t flag : {kVrpnMissing, kVrpnStale, kInvalidVrpn, kTimeJump,
                          kPoseTimeAlignmentRejected, kVrpnFault, kFilterImuOnly}) {
        output.flags = flag;
        EXPECT_FALSE(canPublishVisionPose(output)) << flag;
    }
    output.flags = kImuMissing;
    EXPECT_TRUE(canPublishVisionPose(output));
}
}
