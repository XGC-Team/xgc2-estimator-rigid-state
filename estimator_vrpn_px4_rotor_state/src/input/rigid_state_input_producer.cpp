#include "estimator_vrpn_px4_rotor_state/input/rigid_state_input_producer.h"

#include <utility>
#include <xgc2_math/geometry/se3.hpp>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/common/input_sample_timing.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

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

}  // namespace

RigidStateInputProducer::RigidStateInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                                                 std::string vrpn_pose_topic,
                                                 std::string vrpn_twist_topic,
                                                 std::string pose_transport, uint32_t queue_size,
                                                 EventSink event_sink)
    : event_sink_(std::move(event_sink)) {
    imu_sub_ =
        nh.subscribe(std::move(imu_topic), queue_size, &RigidStateInputProducer::imuCallback, this);
    if (pose_transport == "odometry") {
        odometry_sub_ = nh.subscribe(std::move(vrpn_pose_topic), queue_size,
                                     &RigidStateInputProducer::odometryCallback, this);
    } else {
        vrpn_pose_sub_ = nh.subscribe(std::move(vrpn_pose_topic), queue_size,
                                      &RigidStateInputProducer::vrpnPoseCallback, this);
    }
    if (!vrpn_twist_topic.empty()) {
        vrpn_twist_sub_ = nh.subscribe(std::move(vrpn_twist_topic), queue_size,
                                       &RigidStateInputProducer::vrpnTwistCallback, this);
    }
}

void RigidStateInputProducer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double now_sec = ros::Time::now().toSec();
    const double stamp_sec = msg->header.stamp.isZero() ? now_sec : msg->header.stamp.toSec();
    auto& sample = runtime_input_.imu;
    input_timing::updateSampleTiming(sample, stamp_sec, now_sec);
    sample.angular_velocity = toEigen(msg->angular_velocity);
    sample.linear_acceleration = toEigen(msg->linear_acceleration);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.angular_velocity) &&
                   xgc2_math::isFinite(sample.linear_acceleration);
    postInputEvent(event_type::INPUT_IMU_UPDATED, "raw_imu", now_sec);
}

void RigidStateInputProducer::applyPose(const std_msgs::Header& header,
                                        const geometry_msgs::Pose& pose, const char* source) {
    const double now_sec = ros::Time::now().toSec();
    const double stamp_sec = header.stamp.isZero() ? now_sec : header.stamp.toSec();
    auto& sample = runtime_input_.vrpn_pose;
    input_timing::updateSampleTiming(sample, stamp_sec, now_sec);
    sample.pose.position = pointToEigen(pose.position);
    sample.pose.orientation = toEigen(pose.orientation);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.pose.position) && isValidQuaternion(pose.orientation);
    postInputEvent(event_type::INPUT_VRPN_POSE_UPDATED, source, now_sec);
}

void RigidStateInputProducer::vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    applyPose(msg->header, msg->pose, "vrpn_pose");
}

void RigidStateInputProducer::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    applyPose(msg->header, msg->pose.pose, "lio_odom");
}

void RigidStateInputProducer::vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double now_sec = ros::Time::now().toSec();
    const double stamp_sec = msg->header.stamp.isZero() ? now_sec : msg->header.stamp.toSec();
    auto& sample = runtime_input_.vrpn_velocity;
    input_timing::updateSampleTiming(sample, stamp_sec, now_sec);
    sample.velocity = toEigen(msg->twist.linear);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.velocity);
    postInputEvent(event_type::INPUT_VRPN_VELOCITY_UPDATED, "vrpn_twist", now_sec);
}

void RigidStateInputProducer::postInputEvent(::state_machine::EventId event_id, const char* source,
                                             double timestamp_sec) {
    if (!event_sink_) {
        ROS_ERROR("[RigidStateInputProducer] Event sink is not configured");
        return;
    }

    // Events use receipt time for health evaluation; samples retain source stamps.
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
