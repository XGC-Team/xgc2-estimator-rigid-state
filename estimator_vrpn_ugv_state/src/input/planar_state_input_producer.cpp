#include "estimator_vrpn_ugv_state/input/planar_state_input_producer.h"

#include <ros1_utils/time_utils.h>
#include <xgc2_math/geometry/se3.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "estimator_vrpn_ugv_state/common/event_types.h"

namespace estimator_vrpn_ugv_state {
namespace {

constexpr double kTimestampDuplicateToleranceSec = 1.0e-9;
constexpr double kMinRateDeltaSec = 1.0e-6;

Eigen::Vector2d pointToPlanar(const geometry_msgs::Point& value) {
    return Eigen::Vector2d(value.x, value.y);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& value) {
    const Eigen::Quaterniond q(value.w, value.x, value.y, value.z);
    if (!xgc2_math::isFinite(q) || q.norm() <= 1.0e-9) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::Quaterniond normalized_q = xgc2_math::normalizedQuaternion(q);
    const double siny_cosp =
        2.0 * (normalized_q.w() * normalized_q.z() + normalized_q.x() * normalized_q.y());
    const double cosy_cosp =
        1.0 - 2.0 * (normalized_q.y() * normalized_q.y() + normalized_q.z() * normalized_q.z());
    return xgc2_math::normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
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

PlanarStateInputProducer::PlanarStateInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                                                   std::string vrpn_pose_topic, uint32_t queue_size,
                                                   EventSink event_sink)
    : event_sink_(std::move(event_sink)) {
    imu_sub_ = nh.subscribe(std::move(imu_topic), queue_size,
                            &PlanarStateInputProducer::imuCallback, this);
    vrpn_pose_sub_ = nh.subscribe(std::move(vrpn_pose_topic), queue_size,
                                  &PlanarStateInputProducer::vrpnPoseCallback, this);
}

void PlanarStateInputProducer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    auto& sample = runtime_input_.imu;
    updateSampleTiming(sample, stamp_sec);
    sample.angular_velocity_z = msg->angular_velocity.z;
    sample.linear_acceleration =
        Eigen::Vector2d(msg->linear_acceleration.x, msg->linear_acceleration.y);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.angular_velocity_z) &&
                   xgc2_math::isFinite(sample.linear_acceleration);
    postInputEvent(event_type::INPUT_IMU_UPDATED, "raw_imu", stamp_sec);
}

void PlanarStateInputProducer::vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    auto& sample = runtime_input_.vrpn_pose;
    updateSampleTiming(sample, stamp_sec);
    sample.pose.position = pointToPlanar(msg->pose.position);
    sample.pose.yaw = yawFromQuaternion(msg->pose.orientation);
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.valid = xgc2_math::isFinite(sample.pose);
    postInputEvent(event_type::INPUT_VRPN_POSE_UPDATED, "vrpn_pose", stamp_sec);
}

void PlanarStateInputProducer::postInputEvent(::state_machine::EventId event_id, const char* source,
                                              double timestamp_sec) {
    if (!event_sink_) {
        ROS_ERROR("[PlanarStateInputProducer] Event sink is not configured");
        return;
    }

    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{timestamp_sec});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event), runtime_input_);
    if (!status.ok()) {
        ROS_ERROR_THROTTLE(1.0,
                           "[PlanarStateInputProducer] Failed to post input event %u from %s: %s",
                           event_id, source, status.message.c_str());
    }
}

}  // namespace estimator_vrpn_ugv_state
