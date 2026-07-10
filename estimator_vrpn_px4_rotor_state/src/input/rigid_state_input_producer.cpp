#include "estimator_vrpn_px4_rotor_state/input/rigid_state_input_producer.h"

#include <ros1_utils/time_utils.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <xgc2_math/geometry/se3.hpp>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

constexpr double kTimestampDuplicateToleranceSec = 1.0e-9;
constexpr double kMinRateDeltaSec = 1.0e-6;

Eigen::Vector3d toEigen(const geometry_msgs::Vector3& value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

Eigen::Vector3d pointToEigen(const geometry_msgs::Point& value) {
    return Eigen::Vector3d(value.x, value.y, value.z);
}

Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& value) {
    return xgc2_math::normalizedQuaternion(Eigen::Quaterniond(value.w, value.x, value.y, value.z));
}

bool isValidQuaternion(const geometry_msgs::Quaternion& value) {
    const Eigen::Quaterniond q(value.w, value.x, value.y, value.z);
    return xgc2_math::isFinite(q) && q.norm() > 1.0e-9;
}

template <typename Sample>
void updateSampleTiming(Sample& sample, double stamp_sec) {
    const bool has_prev = sample.received && std::isfinite(sample.stamp_sec);
    const double raw_dt_sec = ros1_utils::samplePeriodSec(has_prev, sample.stamp_sec, stamp_sec);
    const bool finite_dt = std::isfinite(raw_dt_sec);
    sample.time_jump = has_prev && (!finite_dt || raw_dt_sec < -kTimestampDuplicateToleranceSec);
    sample.last_dt_sec = has_prev && finite_dt ? std::max(0.0, raw_dt_sec) : 0.0;
    if (!has_prev || sample.time_jump) {
        sample.estimated_rate_hz = 0.0;
    } else if (raw_dt_sec > kMinRateDeltaSec) {
        sample.estimated_rate_hz = 1.0 / raw_dt_sec;
    }
}

}  // namespace

RigidStateInputProducer::RigidStateInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                                                 std::string vrpn_pose_topic,
                                                 std::string vrpn_twist_topic, uint32_t queue_size,
                                                 EventSink event_sink)
    : event_sink_(std::move(event_sink)) {
    imu_sub_ =
        nh.subscribe(std::move(imu_topic), queue_size, &RigidStateInputProducer::imuCallback, this);
    vrpn_pose_sub_ = nh.subscribe(std::move(vrpn_pose_topic), queue_size,
                                  &RigidStateInputProducer::vrpnPoseCallback, this);
    if (!vrpn_twist_topic.empty()) {
        vrpn_twist_sub_ = nh.subscribe(std::move(vrpn_twist_topic), queue_size,
                                       &RigidStateInputProducer::vrpnTwistCallback, this);
    }
}

void RigidStateInputProducer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    auto& sample = runtime_input_.imu;
    updateSampleTiming(sample, stamp_sec);
    sample.angular_velocity = toEigen(msg->angular_velocity);
    sample.linear_acceleration = toEigen(msg->linear_acceleration);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.angular_velocity) &&
                   xgc2_math::isFinite(sample.linear_acceleration);
    postInputEvent(event_type::INPUT_IMU_UPDATED, "raw_imu", stamp_sec);
}

void RigidStateInputProducer::vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    auto& sample = runtime_input_.vrpn_pose;
    updateSampleTiming(sample, stamp_sec);
    sample.pose.position = pointToEigen(msg->pose.position);
    sample.pose.orientation = toEigen(msg->pose.orientation);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid =
        xgc2_math::isFinite(sample.pose.position) && isValidQuaternion(msg->pose.orientation);
    postInputEvent(event_type::INPUT_VRPN_POSE_UPDATED, "vrpn_pose", stamp_sec);
}

void RigidStateInputProducer::vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    auto& sample = runtime_input_.vrpn_velocity;
    updateSampleTiming(sample, stamp_sec);
    sample.velocity = toEigen(msg->twist.linear);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.velocity);
    postInputEvent(event_type::INPUT_VRPN_VELOCITY_UPDATED, "vrpn_twist", stamp_sec);
}

void RigidStateInputProducer::postInputEvent(::state_machine::EventId event_id, const char* source,
                                             double timestamp_sec) {
    if (!event_sink_) {
        ROS_ERROR("[RigidStateInputProducer] Event sink is not configured");
        return;
    }

    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{timestamp_sec});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event), runtime_input_);
    if (!status.ok()) {
        ROS_ERROR_THROTTLE(1.0,
                           "[RigidStateInputProducer] Failed to post input event %u from %s: %s",
                           event_id, source, status.message.c_str());
    }
}

}  // namespace estimator_vrpn_px4_rotor_state
