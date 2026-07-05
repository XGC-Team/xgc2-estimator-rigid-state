#include "estimator_vrpn_ugv_state/output/planar_state_output_consumer.h"

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <utility>

#include "estimator_vrpn_ugv_state/common/event_types.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {
namespace {

ros::Time eventStampOrNow(const ::state_machine::Event& event) {
    return event.timestamp > 0.0 ? ros::Time(event.timestamp) : ros::Time::now();
}

geometry_msgs::Point toPoint(const Eigen::Vector2d& value) {
    geometry_msgs::Point msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = 0.0;
    return msg;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector2d& value) {
    geometry_msgs::Vector3 msg;
    msg.x = value.x();
    msg.y = value.y();
    msg.z = 0.0;
    return msg;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw) {
    if (!std::isfinite(yaw)) {
        yaw = 0.0;
    }
    const Eigen::Quaterniond q(
        Eigen::AngleAxisd(xgc2_math::normalizeAngle(yaw), Eigen::Vector3d::UnitZ()));
    geometry_msgs::Quaternion msg;
    msg.w = q.w() >= 0.0 ? q.w() : -q.w();
    msg.x = q.w() >= 0.0 ? q.x() : -q.x();
    msg.y = q.w() >= 0.0 ? q.y() : -q.y();
    msg.z = q.w() >= 0.0 ? q.z() : -q.z();
    return msg;
}

geometry_msgs::Vector3 yawRateToVector3(double yaw_rate) {
    geometry_msgs::Vector3 msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = std::isfinite(yaw_rate) ? yaw_rate : 0.0;
    return msg;
}

std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishStateTask(
    ros::Publisher state_pub, rigid_state_estimator_msgs::PlanarStateEstimate state_msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        "PublishPlanarStateEstimate",
        [state_pub = std::move(state_pub), state_msg = std::move(state_msg)](
            ros::NodeHandle&) mutable { state_pub.publish(state_msg); });
}

}  // namespace

PlanarStateOutputConsumer::PlanarStateOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    VrpnUgvStateEstimatorRuntime& runtime, std::string state_topic, uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    state_pub_ = nh.advertise<rigid_state_estimator_msgs::PlanarStateEstimate>(
        std::move(state_topic), queue_size);
}

bool PlanarStateOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_STATE) {
        return false;
    }
    const VrpnUgvStateEstimatorOutput output = runtime_.refreshOutputSnapshot();
    executor_.pushTask(
        makePublishStateTask(state_pub_, makeStateMessage(output, eventStampOrNow(event))));
    return true;
}

rigid_state_estimator_msgs::PlanarStateEstimate PlanarStateOutputConsumer::makeStateMessage(
    const VrpnUgvStateEstimatorOutput& output, const ros::Time& stamp) {
    rigid_state_estimator_msgs::PlanarStateEstimate msg;
    msg.header.stamp = stamp;
    msg.estimator_state = output.estimator_state;
    msg.flags = output.flags;
    msg.position = toPoint(output.state.position);
    msg.velocity = toVector3(output.state.velocity);
    msg.orientation = yawToQuaternion(output.state.yaw);
    msg.angular_velocity = yawRateToVector3(output.state.yaw_rate);
    msg.vrpn_observation_state = static_cast<uint8_t>(output.vrpn_observation_state);
    msg.filter_health = static_cast<uint8_t>(output.filter_health);
    msg.last_pose_reject_reason = static_cast<uint8_t>(output.last_pose_reject_reason);
    msg.last_pose_accepted = output.last_pose_accepted;
    msg.last_fused_pose_stamp_sec = output.last_fused_pose_stamp_sec;
    msg.vrpn_innovation_window_chi_square = output.vrpn_innovation_window_chi_square;
    return msg;
}

}  // namespace estimator_vrpn_ugv_state
