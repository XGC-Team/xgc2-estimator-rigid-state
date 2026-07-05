#include "estimator_vrpn_px4_rotor_state/output/rigid_state_output_consumer.h"

#include <geometry_msgs/PoseStamped.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <xgc2_math/geometry/se3.hpp>

#include <memory>
#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/event_types.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

ros::Time eventStampOrNow(const ::state_machine::Event& event) {
    return event.timestamp > 0.0 ? ros::Time(event.timestamp) : ros::Time::now();
}

geometry_msgs::Point toPoint(const Eigen::Vector3d& value) {
    geometry_msgs::Point msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
    return msg;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector3d& value) {
    geometry_msgs::Vector3 msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
    return msg;
}

geometry_msgs::Quaternion toQuaternion(const Eigen::Quaterniond& value) {
    const Eigen::Quaterniond q = xgc2_math::normalizedQuaternion(value);
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishStateTask(
    ros::Publisher state_pub, rigid_state_estimator_msgs::RigidStateEstimate state_msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        "PublishRigidStateEstimate",
        [state_pub = std::move(state_pub), state_msg = std::move(state_msg)](
            ros::NodeHandle&) mutable { state_pub.publish(state_msg); });
}

std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishVisionPoseTask(
    ros::Publisher vision_pose_pub, geometry_msgs::PoseStamped vision_pose_msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        "PublishUavVisionPose",
        [vision_pose_pub = std::move(vision_pose_pub),
         vision_pose_msg = std::move(vision_pose_msg)](ros::NodeHandle&) mutable {
            vision_pose_pub.publish(vision_pose_msg);
        });
}

rigid_state_estimator_msgs::RigidStateEstimate makeStateMessage(
    const VrpnPx4RotorStateEstimatorOutput& output, const ros::Time& stamp);
geometry_msgs::PoseStamped makeVisionPoseMessage(const VrpnPx4RotorStateEstimatorOutput& output,
                                                 const ros::Time& stamp);
bool canPublishVisionPose(const VrpnPx4RotorStateEstimatorOutput& output);

}  // namespace

RigidStateOutputConsumer::RigidStateOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    VrpnPx4RotorStateEstimatorRuntime& runtime, std::string state_topic,
    std::string vision_pose_topic, uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    state_pub_ = nh.advertise<rigid_state_estimator_msgs::RigidStateEstimate>(
        std::move(state_topic), queue_size);
    vision_pose_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>(std::move(vision_pose_topic), queue_size);
}

bool RigidStateOutputConsumer::handle(const ::state_machine::Event& event) {
    const ros::Time stamp = eventStampOrNow(event);
    if (event.id == output_event_type::PUBLISH_STATE) {
        const VrpnPx4RotorStateEstimatorOutput output = runtime_.refreshOutputSnapshot();
        executor_.pushTask(makePublishStateTask(state_pub_, makeStateMessage(output, stamp)));
        return true;
    }

    if (event.id == output_event_type::PUBLISH_VISION_POSE) {
        const VrpnPx4RotorStateEstimatorOutput output = runtime_.snapshotOutput();
        if (canPublishVisionPose(output)) {
            executor_.pushTask(
                makePublishVisionPoseTask(vision_pose_pub_, makeVisionPoseMessage(output, stamp)));
        }
        return true;
    }

    return false;
}

namespace {

rigid_state_estimator_msgs::RigidStateEstimate makeStateMessage(
    const VrpnPx4RotorStateEstimatorOutput& output, const ros::Time& stamp) {
    rigid_state_estimator_msgs::RigidStateEstimate msg;
    msg.header.stamp = stamp;
    msg.estimator_state = output.estimator_state;
    msg.flags = output.flags;
    msg.position = toPoint(output.state.position);
    msg.velocity = toVector3(output.state.velocity);
    msg.orientation = toQuaternion(output.state.orientation);
    msg.angular_velocity = toVector3(output.state.angular_velocity);
    msg.linear_acceleration = toVector3(output.state.linear_acceleration);
    msg.gravity = toVector3(output.state.gravity);
    msg.accel_bias = toVector3(output.state.accel_bias);
    msg.vrpn_observation_state = static_cast<uint8_t>(output.vrpn_observation_state);
    msg.filter_health = static_cast<uint8_t>(output.filter_health);
    msg.last_pose_reject_reason = static_cast<uint8_t>(output.last_pose_reject_reason);
    msg.last_pose_accepted = output.last_pose_accepted;
    msg.last_fused_pose_stamp_sec = output.last_fused_pose_stamp_sec;
    msg.vrpn_innovation_window_chi_square = output.vrpn_innovation_window_chi_square;
    msg.last_pose_position_innovation_norm_m = output.last_pose_position_innovation_norm_m;
    msg.last_pose_orientation_innovation_norm_rad =
        output.last_pose_orientation_innovation_norm_rad;
    msg.last_pose_mahalanobis_distance = output.last_pose_mahalanobis_distance;
    msg.innovation_position_gate_m = output.innovation_position_gate_m;
    msg.innovation_orientation_gate_rad = output.innovation_orientation_gate_rad;
    msg.pose_nis_gate = output.pose_nis_gate;
    msg.last_imu_sample_stamp_sec = output.last_imu_sample_stamp_sec;
    msg.last_vrpn_pose_stamp_sec = output.last_vrpn_pose_stamp_sec;
    msg.filter_inertial_stamp_sec = output.filter_inertial_stamp_sec;
    msg.filter_pose_stamp_sec = output.filter_pose_stamp_sec;
    msg.vrpn_consecutive_rejects = output.vrpn_consecutive_rejects;
    msg.vrpn_consecutive_accepts = output.vrpn_consecutive_accepts;
    return msg;
}

geometry_msgs::PoseStamped makeVisionPoseMessage(const VrpnPx4RotorStateEstimatorOutput& output,
                                                 const ros::Time& stamp) {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.pose.position = toPoint(output.corrected_vision_pose.position);
    msg.pose.orientation = toQuaternion(output.corrected_vision_pose.orientation);
    return msg;
}

bool canPublishVisionPose(const VrpnPx4RotorStateEstimatorOutput& output) {
    constexpr uint32_t kVisionBlockingFlags = kVrpnMissing | kVrpnStale | kInvalidVrpn | kTimeJump |
                                              kPoseTimeAlignmentRejected | kVrpnFault |
                                              kFilterImuOnly;
    return output.has_corrected_vision_pose && (output.flags & kVisionBlockingFlags) == 0u;
}

}  // namespace

}  // namespace estimator_vrpn_px4_rotor_state
