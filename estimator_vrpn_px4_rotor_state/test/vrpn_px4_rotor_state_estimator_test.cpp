#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "estimator_vrpn_px4_rotor_state/common/health_checks.h"
#include "estimator_vrpn_px4_rotor_state/common/math_utils.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

xgc2_math::InertialSample makeImu(double stamp_sec, const Eigen::Vector3d& gyro,
                                  const Eigen::Vector3d& accel) {
    xgc2_math::InertialSample sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.angular_velocity = gyro;
    sample.linear_acceleration = accel;
    return sample;
}

xgc2_math::PoseMeasurement makePose(
    double stamp_sec, const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity()) {
    xgc2_math::PoseMeasurement sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.pose.position = position;
    sample.pose.orientation = math_utils::normalized(orientation);
    return sample;
}

xgc2_math::VelocityMeasurement makeVelocity(double stamp_sec, const Eigen::Vector3d& velocity) {
    xgc2_math::VelocityMeasurement sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.velocity = velocity;
    return sample;
}

VrpnPx4RotorStateEstimatorConfig testConfig() {
    VrpnPx4RotorStateEstimatorConfig config;
    config.extrinsic_verified = true;
    config.imu_timeout_s = 0.2;
    config.vrpn_timeout_s = 0.2;
    config.coasting_timeout_s = 0.5;
    config.min_imu_rate_hz = 1.0;
    config.min_vrpn_rate_hz = 1.0;
    return config;
}

::state_machine::Event inputEvent(::state_machine::EventId event_id, double stamp_sec) {
    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{stamp_sec});
    event.category = ::state_machine::EventCategory::kInput;
    return event;
}

}  // namespace

TEST(RigidStateMathUtilsTest, NormalizedQuaternionIsUnitAndUsesNonnegativeScalarPart) {
    const Eigen::Quaterniond input(-2.0, 0.1, -0.2, 0.3);
    const Eigen::Quaterniond output = math_utils::normalized(input);

    EXPECT_NEAR(output.norm(), 1.0, 1.0e-12);
    EXPECT_GE(output.w(), 0.0);

    const Eigen::Quaterniond invalid(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
    const Eigen::Quaterniond fallback = math_utils::normalized(invalid);
    EXPECT_NEAR(fallback.w(), 1.0, 1.0e-12);
    EXPECT_NEAR(fallback.vec().norm(), 0.0, 1.0e-12);
}

TEST(RigidStateRuntimeTest, DirectlyUsesObserverEstimatorToInitializeState) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    config.field_to_world.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    config.imu_to_vrpn_marker.position = Eigen::Vector3d(0.1, 0.0, 0.0);

    VrpnPx4RotorStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    const auto imu = makeImu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    const auto pose = makePose(1.0, Eigen::Vector3d(2.0, 0.0, 1.0));
    runtime.estimator().initializeFromPose(pose, &imu);

    ASSERT_TRUE(runtime.estimator().initialized());
    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_NEAR(output.state.position.x(), 2.9, 1.0e-9);
    EXPECT_NEAR(output.state.position.y(), 2.0, 1.0e-9);
    EXPECT_NEAR(output.state.position.z(), 4.0, 1.0e-9);
    EXPECT_NEAR(output.state.linear_acceleration.norm(), 0.0, 1.0e-9);
    ASSERT_TRUE(output.has_corrected_vision_pose);
    EXPECT_NEAR(output.corrected_vision_pose.position.x(), 2.9, 1.0e-9);
}

TEST(RigidStateRuntimeTest, OutOfOrderVrpnPoseSetsTimeAlignmentFlagAndHoldsState) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    config.innovation_position_gate_m = 1.0;
    config.pose_nis_gate = 1.0e6;

    VrpnPx4RotorStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    VrpnPx4RotorStateEstimatorInput input;
    input.imu = makeImu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.vrpn_pose = makePose(1.0, Eigen::Vector3d::Zero());
    runtime.estimator().initializeFromPose(input.vrpn_pose, &input.imu);
    ASSERT_TRUE(runtime.estimator().initialized());

    runtime.estimator().propagateInertial(
        makeImu(1.02, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 9.8066)));
    const auto held_state = runtime.estimator().state();

    input.vrpn_pose = makePose(1.01, Eigen::Vector3d(0.01, 0.0, 0.0));
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.01), input).ok());
    runtime.processVrpnInput();

    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_FALSE(output.last_pose_accepted);
    EXPECT_EQ(output.last_pose_reject_reason, xgc2_math::PoseFusionRejectReason::kTimeAlignment);
    EXPECT_NE(output.flags & kPoseTimeAlignmentRejected, 0u);
    EXPECT_NE(output.flags & kVrpnSuspected, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kSuspected);
    EXPECT_NEAR(output.state.last_inertial_stamp_sec, 1.02, 1.0e-12);
    EXPECT_NEAR(output.last_fused_pose_stamp_sec, 1.0, 1.0e-12);
    ASSERT_TRUE(output.has_corrected_vision_pose);
    EXPECT_NEAR(output.state.position.x(), held_state.position.x(), 1.0e-12);
    EXPECT_NEAR(output.state.position.y(), held_state.position.y(), 1.0e-12);
    EXPECT_NEAR(output.state.position.z(), held_state.position.z(), 1.0e-12);
}

TEST(RigidStateRuntimeTest, VrpnVelocityMeasurementCorrectsInertialVelocityDrift) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    config.vrpn_velocity_noise_std = 0.02;

    VrpnPx4RotorStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    VrpnPx4RotorStateEstimatorInput input;
    input.imu = makeImu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.vrpn_pose = makePose(1.0, Eigen::Vector3d::Zero());
    runtime.estimator().initializeFromPose(input.vrpn_pose, &input.imu);
    ASSERT_TRUE(runtime.estimator().initialized());

    runtime.estimator().propagateInertial(
        makeImu(1.1, Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 0.0, 9.8066)));
    ASSERT_GT(runtime.estimator().state().velocity.x(), 0.05);

    input.vrpn_velocity = makeVelocity(1.1, Eigen::Vector3d::Zero());
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_VELOCITY_UPDATED, 1.1), input)
            .ok());
    runtime.processVrpnVelocityInput();

    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_NEAR(output.state.velocity.x(), 0.0, 0.01);
    EXPECT_NEAR(output.state.velocity.y(), 0.0, 0.01);
    EXPECT_NEAR(output.state.velocity.z(), 0.0, 0.01);
}

TEST(RigidStateRuntimeTest, VrpnFaultFlagsAndFilteredVisionPoseRecover) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    config.innovation_position_gate_m = 0.1;
    config.vrpn_health.fault_after_rejects = 2;
    config.vrpn_health.recovery_after_accepts = 2;

    VrpnPx4RotorStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    VrpnPx4RotorStateEstimatorInput input;
    input.imu = makeImu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.vrpn_pose = makePose(1.0, Eigen::Vector3d::Zero());
    runtime.estimator().initializeFromPose(input.vrpn_pose, &input.imu);

    input.vrpn_pose = makePose(1.01, Eigen::Vector3d(1.0, 0.0, 0.0));
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.01), input).ok());
    runtime.processVrpnInput();
    auto output = runtime.refreshOutputSnapshot();
    EXPECT_NE(output.flags & kInnovationRejected, 0u);
    EXPECT_NE(output.flags & kVrpnSuspected, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kSuspected);
    EXPECT_EQ(output.filter_health, xgc2_math::FilterHealth::kDegraded);

    input.vrpn_pose = makePose(1.02, Eigen::Vector3d(1.0, 0.0, 0.0));
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.02), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_NE(output.flags & kVrpnFault, 0u);
    EXPECT_NE(output.flags & kFilterImuOnly, 0u);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kFault);

    input.vrpn_pose = makePose(1.03, Eigen::Vector3d::Zero());
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.03), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_TRUE(output.last_pose_accepted);
    EXPECT_EQ(output.last_pose_reject_reason, xgc2_math::PoseFusionRejectReason::kNone);
    EXPECT_EQ(output.flags & kFilterImuOnly, 0u);
    EXPECT_NE(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kFault);
    EXPECT_EQ(output.filter_health, xgc2_math::FilterHealth::kDegraded);

    input.vrpn_pose = makePose(1.04, Eigen::Vector3d::Zero());
    ASSERT_TRUE(
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.04), input).ok());
    runtime.processVrpnInput();
    output = runtime.refreshOutputSnapshot();
    EXPECT_TRUE(output.last_pose_accepted);
    EXPECT_EQ(output.vrpn_observation_state, xgc2_math::VrpnObservationState::kTrusted);
    EXPECT_EQ(output.filter_health, xgc2_math::FilterHealth::kNominal);
    ASSERT_TRUE(output.has_corrected_vision_pose);
    EXPECT_NEAR(output.corrected_vision_pose.position.x(), output.state.position.x(), 1.0e-12);
}

TEST(RigidStateHealthTest, InitializedEstimatorCoastsOnShortVrpnLossThenFaults) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    VrpnPx4RotorStateEstimatorInput input;
    input.imu = makeImu(10.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.vrpn_pose = makePose(9.7, Eigen::Vector3d::Zero());

    auto health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.0);
    EXPECT_EQ(health.state, state_type::Coasting);
    EXPECT_NE(health.flags & kCoasting, 0u);

    health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.6);
    EXPECT_EQ(health.state, state_type::Fault);
    EXPECT_NE(health.flags & kFault, 0u);
}

TEST(RigidStateHealthTest, DuplicateTimestampCanRemainRunningWhenSamplesAreFresh) {
    VrpnPx4RotorStateEstimatorConfig config = testConfig();
    VrpnPx4RotorStateEstimatorInput input;
    input.imu = makeImu(10.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.imu.last_dt_sec = 0.0;
    input.imu.estimated_rate_hz = config.min_imu_rate_hz + 1.0;
    input.vrpn_pose = makePose(10.0, Eigen::Vector3d::Zero());
    input.vrpn_pose.last_dt_sec = 0.0;
    input.vrpn_pose.estimated_rate_hz = config.min_vrpn_rate_hz + 1.0;

    const auto health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.0);
    EXPECT_EQ(health.state, state_type::Running);
    EXPECT_EQ(health.flags & kTimeJump, 0u);
    EXPECT_EQ(health.flags & kCoasting, 0u);
    EXPECT_TRUE(health.imu_ready);
    EXPECT_TRUE(health.vrpn_ready);
}

}  // namespace estimator_vrpn_px4_rotor_state

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
