#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "estimator_rigid_state/common/health_checks.h"
#include "estimator_rigid_state/common/math_utils.h"
#include "estimator_rigid_state/rigid_state_estimator_runtime.h"

namespace estimator_rigid_state {
namespace {

xgc2_observer::InertialSample makeImu(double stamp_sec, const Eigen::Vector3d& gyro,
                                      const Eigen::Vector3d& accel) {
    xgc2_observer::InertialSample sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.angular_velocity = gyro;
    sample.linear_acceleration = accel;
    return sample;
}

xgc2_observer::PoseMeasurement makePose(
    double stamp_sec, const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity()) {
    xgc2_observer::PoseMeasurement sample;
    sample.received = true;
    sample.valid = true;
    sample.stamp_sec = stamp_sec;
    sample.pose.position = position;
    sample.pose.orientation = math_utils::normalized(orientation);
    return sample;
}

RigidStateEstimatorConfig testConfig() {
    RigidStateEstimatorConfig config;
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
    RigidStateEstimatorConfig config = testConfig();
    config.field_to_world.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    config.imu_to_vrpn_marker.position = Eigen::Vector3d(0.1, 0.0, 0.0);

    RigidStateEstimatorRuntime runtime;
    runtime.setConfig(config);

    RigidStateEstimatorInput input;
    input.imu = makeImu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.vrpn_pose = makePose(1.0, Eigen::Vector3d(2.0, 0.0, 1.0));

    const auto status =
        runtime.postInputEvent(inputEvent(event_type::INPUT_VRPN_POSE_UPDATED, 1.0), input);
    ASSERT_TRUE(status.ok()) << status.message;
    runtime.update(1.0);
    runtime.update(1.01);

    ASSERT_TRUE(runtime.estimator().initialized());
    const auto output = runtime.refreshOutputSnapshot();
    EXPECT_NEAR(output.state.position.x(), 2.9, 1.0e-9);
    EXPECT_NEAR(output.state.position.y(), 2.0, 1.0e-9);
    EXPECT_NEAR(output.state.position.z(), 4.0, 1.0e-9);
    ASSERT_TRUE(output.has_corrected_vision_pose);
    EXPECT_NEAR(output.corrected_vision_pose.position.x(), 2.9, 1.0e-9);
}

TEST(RigidStateHealthTest, InitializedEstimatorCoastsOnShortVrpnLossThenFaults) {
    RigidStateEstimatorConfig config = testConfig();
    RigidStateEstimatorInput input;
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
    RigidStateEstimatorConfig config = testConfig();
    RigidStateEstimatorInput input;
    input.imu = makeImu(10.0, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8066));
    input.imu.last_dt_sec = 0.0;
    input.imu.estimated_rate_hz = 50.0;
    input.vrpn_pose = makePose(10.0, Eigen::Vector3d::Zero());
    input.vrpn_pose.last_dt_sec = 0.0;
    input.vrpn_pose.estimated_rate_hz = 120.0;

    const auto health = health_checks::classify(input, config, true, false, 1.0, 0u, 10.0);
    EXPECT_EQ(health.state, state_type::Running);
    EXPECT_EQ(health.flags & kTimeJump, 0u);
    EXPECT_EQ(health.flags & kCoasting, 0u);
    EXPECT_TRUE(health.imu_ready);
    EXPECT_TRUE(health.vrpn_ready);
}

}  // namespace estimator_rigid_state

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
