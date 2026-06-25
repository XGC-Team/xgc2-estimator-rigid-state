#include <gtest/gtest.h>

#include <cmath>

#include "estimator_vrpn_ugv_state/common/health_checks.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {
namespace {

xgc2_math::PlanarInertialSample makeImu(double stamp_sec, double gyro_z,
                                        const Eigen::Vector2d& accel) {
    xgc2_math::PlanarInertialSample sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.angular_velocity_z = gyro_z;
    sample.linear_acceleration = accel;
    return sample;
}

xgc2_math::PlanarPoseMeasurement makePose(double stamp_sec, const Eigen::Vector2d& position,
                                          double yaw = 0.0) {
    xgc2_math::PlanarPoseMeasurement sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.pose.position = position;
    sample.pose.yaw = yaw;
    return sample;
}

VrpnUgvStateEstimatorConfig testConfig() {
    VrpnUgvStateEstimatorConfig config;
    config.extrinsic_verified = true;
    config.imu_timeout_s = 0.3;
    config.vrpn_timeout_s = 0.2;
    config.coasting_timeout_s = 0.5;
    config.min_imu_rate_hz = 1.0;
    config.min_vrpn_rate_hz = 1.0;
    config.vrpn_position_noise_std = 0.01;
    config.vrpn_yaw_noise_std = 0.01;
    return config;
}

::state_machine::Event inputEvent(::state_machine::EventId event_id, double stamp_sec) {
    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{stamp_sec});
    event.category = ::state_machine::EventCategory::kInput;
    return event;
}

}  // namespace

TEST(VrpnUgvStateRuntimeTest, InitializesFromVrpnPoseWithSe2Extrinsic) {
    VrpnUgvStateEstimatorConfig config = testConfig();
    config.field_to_world.position = Eigen::Vector2d(1.0, 2.0);
    config.body_to_vrpn_marker.position = Eigen::Vector2d(0.2, 0.0);

    VrpnUgvStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    VrpnUgvStateEstimatorInput input;
    input.imu = makeImu(1.0, 0.0, Eigen::Vector2d::Zero());
    input.vrpn_pose = makePose(1.0, Eigen::Vector2d(2.0, 0.0), 0.1);

    const auto status =
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.0), input);
    ASSERT_TRUE(status.ok()) << status.message;
    runtime.update(1.0);
    runtime.update(1.01);

    ASSERT_TRUE(runtime.estimator().initialized());
    const auto output = runtime.refreshOutputSnapshot();
    const xgc2_math::Pose2 marker_world =
        xgc2_math::compose(config.field_to_world, input.vrpn_pose.pose);
    const xgc2_math::Pose2 expected_body =
        xgc2_math::compose(marker_world, xgc2_math::inverse(config.body_to_vrpn_marker));
    EXPECT_NEAR(output.state.position.x(), expected_body.position.x(), 1.0e-9);
    EXPECT_NEAR(output.state.position.y(), expected_body.position.y(), 1.0e-9);
    EXPECT_NEAR(output.state.yaw, expected_body.yaw, 1.0e-9);
}

TEST(VrpnUgvStateRuntimeTest, ImuEventPropagatesOnlyWhenPosted) {
    VrpnUgvStateEstimatorRuntime runtime;
    runtime.setConfig(testConfig());

    VrpnUgvStateEstimatorInput input;
    input.imu = makeImu(1.0, 0.0, Eigen::Vector2d::Zero());
    input.vrpn_pose = makePose(1.0, Eigen::Vector2d::Zero(), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.0), input).ok());
    runtime.update(1.0);
    runtime.update(1.01);

    input.imu = makeImu(1.02, 0.1, Eigen::Vector2d(0.2, 0.0));
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_IMU_UPDATED, 1.02), input).ok());
    runtime.update(1.02);
    runtime.update(1.03);

    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_GT(output.state.position.x(), 0.0);
    EXPECT_GT(output.state.yaw, 0.0);
}

TEST(VrpnUgvStateRuntimeTest, OutOfOrderVrpnPoseSetsTimeAlignmentFlagAndHoldsState) {
    VrpnUgvStateEstimatorRuntime runtime;
    runtime.setConfig(testConfig());

    VrpnUgvStateEstimatorInput input;
    input.imu = makeImu(1.0, 0.0, Eigen::Vector2d::Zero());
    input.vrpn_pose = makePose(1.0, Eigen::Vector2d::Zero(), 0.0);
    runtime.estimator().initializeFromPose(input.vrpn_pose, &input.imu);
    ASSERT_TRUE(runtime.estimator().initialized());

    runtime.estimator().propagateInertial(makeImu(1.02, 0.0, Eigen::Vector2d(0.5, 0.0)));
    const auto held_state = runtime.estimator().state();

    input.vrpn_pose = makePose(1.01, Eigen::Vector2d(0.01, 0.0), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.01), input).ok());
    runtime.processVrpnInput();

    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_FALSE(output.last_pose_accepted);
    EXPECT_EQ(output.last_pose_reject_reason, xgc2_math::PoseFusionRejectReason::kTimeAlignment);
    EXPECT_NE(output.flags & kPoseTimeAlignmentRejected, 0u);
    EXPECT_NE(output.flags & kVrpnSuspected, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kSuspected);
    EXPECT_NEAR(output.state.position.x(), held_state.position.x(), 1.0e-12);
    EXPECT_NEAR(output.state.position.y(), held_state.position.y(), 1.0e-12);
    EXPECT_NEAR(output.state.yaw, held_state.yaw, 1.0e-12);
}

TEST(VrpnUgvStateRuntimeTest, VrpnFaultFlagsAndFilteredPoseOutputRecover) {
    VrpnUgvStateEstimatorConfig config = testConfig();
    config.innovation_position_gate_m = 0.1;
    config.vrpn_health.fault_after_rejects = 2;
    config.vrpn_health.recovery_after_accepts = 2;

    VrpnUgvStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    VrpnUgvStateEstimatorInput input;
    input.imu = makeImu(1.0, 0.0, Eigen::Vector2d::Zero());
    input.vrpn_pose = makePose(1.0, Eigen::Vector2d::Zero(), 0.0);
    runtime.estimator().initializeFromPose(input.vrpn_pose, &input.imu);

    input.vrpn_pose = makePose(1.01, Eigen::Vector2d(1.0, 0.0), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.01), input).ok());
    runtime.processVrpnInput();
    auto output = runtime.refreshOutputSnapshot();
    EXPECT_NE(output.flags & kInnovationRejected, 0u);
    EXPECT_NE(output.flags & kVrpnSuspected, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kSuspected);
    EXPECT_EQ(output.filter_health, xgc2_math::FilterHealth::kDegraded);

    input.vrpn_pose = makePose(1.02, Eigen::Vector2d(1.0, 0.0), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.02), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_NE(output.flags & kVrpnFault, 0u);
    EXPECT_NE(output.flags & kFilterImuOnly, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kFault);

    input.vrpn_pose = makePose(1.03, Eigen::Vector2d::Zero(), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.03), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_FALSE(output.last_pose_accepted);
    EXPECT_EQ(output.last_pose_reject_reason, xgc2_math::PoseFusionRejectReason::kVrpnFault);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kRecovery);

    input.vrpn_pose = makePose(1.04, Eigen::Vector2d::Zero(), 0.0);
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.04), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_TRUE(output.last_pose_accepted);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kTrusted);
    EXPECT_EQ(output.filter_health, xgc2_math::FilterHealth::kNominal);
    ASSERT_TRUE(output.has_corrected_body_pose);
    EXPECT_NEAR(output.corrected_body_pose.position.x(), output.state.position.x(), 1.0e-12);
}

TEST(VrpnUgvStateHealthTest, InitializedEstimatorCoastsOnShortVrpnLossThenFaults) {
    VrpnUgvStateEstimatorConfig config = testConfig();
    VrpnUgvStateEstimatorInput input;
    input.imu = makeImu(10.0, 0.0, Eigen::Vector2d::Zero());
    input.vrpn_pose = makePose(9.7, Eigen::Vector2d::Zero(), 0.0);

    auto health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.0);
    EXPECT_EQ(health.state, state_type::Coasting);
    EXPECT_NE(health.flags & kCoasting, 0u);

    health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.6);
    EXPECT_EQ(health.state, state_type::Fault);
    EXPECT_NE(health.flags & kFault, 0u);
}

}  // namespace estimator_vrpn_ugv_state

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
