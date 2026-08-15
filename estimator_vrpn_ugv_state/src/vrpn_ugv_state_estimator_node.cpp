#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_node.h"

#include <ros1_utils/param_utils.h>

#include <cmath>
#include <memory>
#include <utility>

#include "estimator_vrpn_ugv_state/common/config_utils.h"

namespace estimator_vrpn_ugv_state {
namespace {

constexpr uint32_t kRosQueueSize = 20;

void readPose2Params(ros::NodeHandle& nh, const char* xyz_name, const char* rpy_name,
                     xgc2_math::Pose2& pose, const char* description) {
    Eigen::Vector3d xyz(pose.position.x(), pose.position.y(), 0.0);
    Eigen::Vector3d rpy(0.0, 0.0, pose.yaw);
    ros1_utils::getVector3ParamWithLog(nh, xyz_name, xyz, std::string(description) + " xyz");
    ros1_utils::getVector3ParamWithLog(nh, rpy_name, rpy, std::string(description) + " rpy");
    pose.position = Eigen::Vector2d(xyz.x(), xyz.y());
    pose.yaw = xgc2_math::normalizeAngle(rpy.z());
}

}  // namespace

VrpnUgvStateEstimatorNode::VrpnUgvStateEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();
    runtime_.setConfig(config_);

    output_event_dispatcher_.addConsumer(std::make_unique<PlanarStateOutputConsumer>(
        nh_, output_event_executor_, runtime_, state_topic_, kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event,
                                   const VrpnUgvStateEstimatorInput& input) {
        return runtime_.postInputEvent(std::move(event), input);
    };
    input_producer_ = std::make_unique<PlanarStateInputProducer>(
        nh_, imu_topic_, vrpn_pose_topic_, pose_transport_, kRosQueueSize,
        std::move(post_input_event));

    output_event_executor_.start();

    ROS_INFO(
        "[VrpnUgvStateEstimatorNode] Initialized: imu=%s pose=%s transport=%s source=%s "
        "state=%s loop=%.1f state_pub=%.1f max_pose_delay=%.3f",
        imu_topic_.c_str(), vrpn_pose_topic_.c_str(), pose_transport_.c_str(),
        pose_source_.c_str(), state_topic_.c_str(), loop_rate_hz_, config_.state_publish_rate_hz,
        config_.max_pose_delay_s);
}

VrpnUgvStateEstimatorNode::~VrpnUgvStateEstimatorNode() {
    output_event_executor_.stop();
}

void VrpnUgvStateEstimatorNode::run(double frequency) {
    const double loop_frequency =
        std::isfinite(frequency) && frequency > 0.0 ? frequency : loop_rate_hz_;
    ROS_INFO("[VrpnUgvStateEstimatorNode] Starting state estimator loop at %.1f Hz",
             loop_frequency);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();
        const double now_sec = ros::Time::now().toSec();
        runtime_.update(now_sec);
        dispatchOutputEvents(runtime_.getStateMachine().currentOutputEvents());
        rate.sleep();
    }

    ROS_INFO("[VrpnUgvStateEstimatorNode] State estimator loop exited");
}

double VrpnUgvStateEstimatorNode::loopRateHz() const {
    return loop_rate_hz_;
}

void VrpnUgvStateEstimatorNode::loadParams() {
    ros1_utils::getParamWithLog(private_nh_, "imu_topic", imu_topic_, "IMU topic");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_pose_topic", vrpn_pose_topic_,
                                "VRPN pose topic");
    ros1_utils::getParamWithLog(private_nh_, "state_topic", state_topic_, "State topic");
    ros1_utils::getParamWithLog(private_nh_, "pose_source", pose_source_, "Pose source vrpn|lio");
    ros1_utils::getParamWithLog(private_nh_, "pose_transport", pose_transport_,
                                "Pose transport pose_stamped|odometry");
    if (pose_source_ == "lio" && !private_nh_.hasParam("pose_transport")) {
        pose_transport_ = "odometry";
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("vrpn_pose_topic")) {
        vrpn_pose_topic_ = "/Odometry";
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("max_pose_delay_s")) {
        config_.max_pose_delay_s = 0.30;
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("min_vrpn_rate_hz")) {
        config_.min_vrpn_rate_hz = 8.0;
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("vrpn_timeout_s")) {
        config_.vrpn_timeout_s = 0.30;
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("use_imu_horizontal_accel")) {
        config_.use_imu_horizontal_accel = false;
    }
    if (pose_source_ == "lio" && !private_nh_.hasParam("reinitialize_on_innovation_gate")) {
        config_.reinitialize_on_innovation_gate = true;
    }

    ros1_utils::getParamWithLog(private_nh_, "loop_rate_hz", loop_rate_hz_, "Loop rate");
    ros1_utils::getParamWithLog(private_nh_, "state_publish_rate_hz", config_.state_publish_rate_hz,
                                "State publish rate");
    config_.loop_rate_hz = loop_rate_hz_;

    readPose2Params(private_nh_, "field_offset_xyz", "field_offset_rpy", config_.field_to_world,
                    "Field offset");
    readPose2Params(private_nh_, "body_to_vrpn_marker_xyz", "body_to_vrpn_marker_rpy",
                    config_.body_to_vrpn_marker, "Body to VRPN marker");
    ros1_utils::getParamWithLog(private_nh_, "extrinsic_verified", config_.extrinsic_verified,
                                "Extrinsic verified");

    ros1_utils::getParamWithLog(private_nh_, "imu_timeout_s", config_.imu_timeout_s, "IMU timeout");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_timeout_s", config_.vrpn_timeout_s,
                                "VRPN timeout");
    ros1_utils::getParamWithLog(private_nh_, "coasting_timeout_s", config_.coasting_timeout_s,
                                "Coasting timeout");
    ros1_utils::getParamWithLog(private_nh_, "min_imu_rate_hz", config_.min_imu_rate_hz,
                                "Minimum IMU rate");
    ros1_utils::getParamWithLog(private_nh_, "min_vrpn_rate_hz", config_.min_vrpn_rate_hz,
                                "Minimum VRPN rate");
    ros1_utils::getParamWithLog(private_nh_, "max_time_jump_s", config_.max_time_jump_s,
                                "Maximum time jump");
    ros1_utils::getParamWithLog(private_nh_, "max_pose_delay_s", config_.max_pose_delay_s,
                                "Maximum delayed pose age");
    ros1_utils::getParamWithLog(private_nh_, "use_imu_horizontal_accel",
                                config_.use_imu_horizontal_accel, "Use IMU horizontal accel");
    ros1_utils::getParamWithLog(private_nh_, "reinitialize_on_innovation_gate",
                                config_.reinitialize_on_innovation_gate,
                                "Reinitialize on pose innovation gate");

    ros1_utils::getParamWithLog(private_nh_, "gyro_noise_std", config_.gyro_noise_std,
                                "Gyroscope noise std");
    ros1_utils::getParamWithLog(private_nh_, "accel_noise_std", config_.accel_noise_std,
                                "Accelerometer noise std");
    ros1_utils::getParamWithLog(private_nh_, "gyro_bias_random_walk_std",
                                config_.gyro_bias_random_walk_std, "Gyro bias random walk std");
    ros1_utils::getParamWithLog(private_nh_, "accel_bias_random_walk_std",
                                config_.accel_bias_random_walk_std, "Accel bias random walk std");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_position_noise_std",
                                config_.vrpn_position_noise_std, "VRPN position noise std");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_yaw_noise_std", config_.vrpn_yaw_noise_std,
                                "VRPN yaw noise std");
    ros1_utils::getParamWithLog(private_nh_, "innovation_position_gate_m",
                                config_.innovation_position_gate_m, "Position innovation gate");
    ros1_utils::getParamWithLog(private_nh_, "innovation_yaw_gate_rad",
                                config_.innovation_yaw_gate_rad, "Yaw innovation gate");
    ros1_utils::getParamWithLog(private_nh_, "covariance_high_threshold",
                                config_.covariance_high_threshold, "Covariance high threshold");

    config_utils::normalizeConfig(config_);
    loop_rate_hz_ = config_.loop_rate_hz;
}

void VrpnUgvStateEstimatorNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    const auto result = output_event_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[VrpnUgvStateEstimatorNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[VrpnUgvStateEstimatorNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace estimator_vrpn_ugv_state
