#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "estimator_vrpn_px4_rotor_state/common/health_checks.h"
#include "estimator_vrpn_px4_rotor_state/common/input_sample_timing.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

template <typename Sample>
void admitTiming(Sample& sample, double source_stamp, double receipt_time) {
    input_timing::updateSampleTiming(sample, source_stamp, receipt_time);
    sample.stamp_sec = source_stamp;
    sample.received = true;
    sample.valid = true;
}

VrpnPx4RotorStateEstimatorInput timedInput(double imu_stamp, double pose_stamp, double now) {
    VrpnPx4RotorStateEstimatorInput input;
    input.imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.8066);
    admitTiming(input.imu, imu_stamp, now);
    admitTiming(input.vrpn_pose, pose_stamp, now);
    return input;
}

TEST(RigidInputTiming, RejectsFirstFutureImuWithoutPreviousSample) {
    const auto input = timedInput(100.0, 10.0, 10.0);
    EXPECT_TRUE(input.imu.time_jump);
    const auto health = health_checks::classify(input, {}, false, false, 1.0, 0u, 10.0);
    EXPECT_FALSE(health.imu_ready);
    EXPECT_EQ(health.condition, HealthCondition::kInputUnhealthy);
    EXPECT_NE(health.flags & kTimeJump, 0u);
    EXPECT_DOUBLE_EQ(input.imu.stamp_sec, 100.0);
}

TEST(RigidInputTiming, RejectsFirstFuturePoseBeforeInitialization) {
    const auto input = timedInput(10.0, 100.0, 10.0);
    const auto health = health_checks::classify(input, {}, false, false, 1.0, 0u, 10.0);
    EXPECT_TRUE(health.imu_ready);
    EXPECT_FALSE(health.vrpn_ready);
    EXPECT_EQ(health.condition, HealthCondition::kInputUnhealthy);
    EXPECT_NE(health.flags & kTimeJump, 0u);
}

TEST(RigidInputTiming, OrderedFutureStreamDoesNotClearTimeJump) {
    xgc2_math::InertialSample sample;
    admitTiming(sample, 100.0, 10.0);
    admitTiming(sample, 100.01, 10.01);
    EXPECT_TRUE(sample.time_jump);
    EXPECT_NEAR(sample.last_dt_sec, 0.01, 1.0e-9);
    EXPECT_DOUBLE_EQ(sample.estimated_rate_hz, 0.0);
}

TEST(RigidInputTiming, SmallFutureSkewAndOrderedDelayRemainAdmissible) {
    auto input = timedInput(10.01, 9.95, 10.0);
    const auto health = health_checks::classify(input, {}, false, false, 1.0, 0u, 10.0);
    EXPECT_FALSE(input.imu.time_jump);
    EXPECT_FALSE(input.vrpn_pose.time_jump);
    EXPECT_EQ(health.condition, HealthCondition::kInitializationReady);
    EXPECT_DOUBLE_EQ(input.vrpn_pose.stamp_sec, 9.95);
}

TEST(RigidInputTiming, ArrivalDoesNotRefreshAnOldSourceSample) {
    const auto input = timedInput(1.0, 1.0, 10.0);
    const auto health = health_checks::classify(input, {}, false, false, 1.0, 0u, 10.0);
    EXPECT_FALSE(input.imu.time_jump);
    EXPECT_FALSE(input.vrpn_pose.time_jump);
    EXPECT_EQ(health.condition, HealthCondition::kInputUnhealthy);
    EXPECT_NE(health.flags & kImuStale, 0u);
    EXPECT_NE(health.flags & kVrpnStale, 0u);
}

TEST(RigidInputTiming, DuplicateRetainsRateAndBackwardSampleSetsJump) {
    xgc2_math::PoseMeasurement sample;
    admitTiming(sample, 10.0, 10.0);
    admitTiming(sample, 10.01, 10.01);
    EXPECT_NEAR(sample.estimated_rate_hz, 100.0, 1.0e-6);
    admitTiming(sample, 10.01, 10.02);
    EXPECT_FALSE(sample.time_jump);
    EXPECT_DOUBLE_EQ(sample.last_dt_sec, 0.0);
    EXPECT_NEAR(sample.estimated_rate_hz, 100.0, 1.0e-6);
    admitTiming(sample, 9.99, 10.03);
    EXPECT_TRUE(sample.time_jump);
    EXPECT_DOUBLE_EQ(sample.last_dt_sec, 0.0);
    EXPECT_DOUBLE_EQ(sample.estimated_rate_hz, 0.0);
}

TEST(RigidInputTiming, NonFiniteSourceOrReceiptClockFailsClosed) {
    const double invalid[] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()};
    for (double value : invalid) {
        xgc2_math::InertialSample sample;
        admitTiming(sample, value, 10.0);
        EXPECT_TRUE(sample.time_jump);
        sample = xgc2_math::InertialSample{};
        admitTiming(sample, 10.0, value);
        EXPECT_TRUE(sample.time_jump);
    }
}

TEST(RigidInputTiming, OrderedSamplesRecoverAfterClockIsCorrected) {
    xgc2_math::InertialSample sample;
    admitTiming(sample, 100.0, 10.0);
    admitTiming(sample, 10.01, 10.01);
    EXPECT_TRUE(sample.time_jump);
    admitTiming(sample, 10.02, 10.02);
    EXPECT_FALSE(sample.time_jump);
    EXPECT_NEAR(sample.estimated_rate_hz, 100.0, 1.0e-6);
}

TEST(RigidInputTiming, FutureVelocityCarriesTimeJumpToFilterAdmission) {
    xgc2_math::VelocityMeasurement sample;
    admitTiming(sample, 100.0, 10.0);
    EXPECT_TRUE(sample.time_jump);
    EXPECT_DOUBLE_EQ(sample.stamp_sec, 100.0);
}

TEST(RigidInputTiming, FutureStreamCannotInitializeRuntime) {
    VrpnPx4RotorStateEstimatorRuntime runtime;
    const auto input = timedInput(100.0, 100.0, 10.0);
    ::state_machine::Event event(event_type::INPUT_IMU_UPDATED,
                                 ::state_machine::EventTimestamp{10.0});
    ASSERT_TRUE(runtime.postInputEvent(event, input).ok());
    const auto output = runtime.update(10.0);
    EXPECT_EQ(output.estimator_state, state_type::SelfCheck);
    EXPECT_FALSE(runtime.estimator().initialized());
    EXPECT_NE(output.flags & kTimeJump, 0u);
    EXPECT_DOUBLE_EQ(runtime.currentTime(), 10.0);
    EXPECT_DOUBLE_EQ(runtime.input().imu.stamp_sec, 100.0);
}

}  // namespace
}  // namespace estimator_vrpn_px4_rotor_state
