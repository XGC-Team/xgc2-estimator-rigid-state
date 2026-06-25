#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_node.h"

#include <ros1_utils/param_utils.h>

#include <cmath>
#include <memory>
#include <utility>

#include "estimator_vrpn_px4_rotor_state/common/config_utils.h"
#include "estimator_vrpn_px4_rotor_state/common/math_utils.h"

namespace estimator_vrpn_px4_rotor_state {
namespace {

constexpr uint32_t kRosQueueSize = 20;

void readPoseParams(ros::NodeHandle& nh, const char* xyz_name, const char* rpy_name,
                    xgc2_math::Pose3& pose, const char* description) {
    Eigen::Vector3d xyz = pose.position;
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    ros1_utils::getVector3ParamWithLog(nh, xyz_name, xyz, std::string(description) + " xyz");
    ros1_utils::getVector3ParamWithLog(nh, rpy_name, rpy, std::string(description) + " rpy");
    pose.position = xyz;
    pose.orientation = math_utils::rpyToQuaternion(rpy);
}

}  // namespace

VrpnPx4RotorStateEstimatorNode::VrpnPx4RotorStateEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();
    runtime_.setConfig(config_);

    output_event_dispatcher_.addConsumer(std::make_unique<RigidStateOutputConsumer>(
        nh_, output_event_executor_, runtime_, state_topic_, vision_pose_topic_, kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event,
                                   const VrpnPx4RotorStateEstimatorInput& input) {
        return runtime_.postInputEvent(std::move(event), input);
    };

    input_producer_ = std::make_unique<RigidStateInputProducer>(
        nh_, imu_topic_, vrpn_pose_topic_, kRosQueueSize, std::move(post_input_event));

    output_event_executor_.start();

    ROS_INFO(
        "[VrpnPx4RotorStateEstimatorNode] Initialized: imu=%s vrpn_pose=%s state=%s "
        "vision_pose=%s loop=%.1f state_pub=%.1f vision_pub=%.1f",
        imu_topic_.c_str(), vrpn_pose_topic_.c_str(), state_topic_.c_str(),
        vision_pose_topic_.c_str(), loop_rate_hz_, config_.state_publish_rate_hz,
        config_.vision_publish_rate_hz);
}

VrpnPx4RotorStateEstimatorNode::~VrpnPx4RotorStateEstimatorNode() {
    output_event_executor_.stop();
}

void VrpnPx4RotorStateEstimatorNode::run(double frequency) {
    const double loop_frequency =
        std::isfinite(frequency) && frequency > 0.0 ? frequency : loop_rate_hz_;
    ROS_INFO("[VrpnPx4RotorStateEstimatorNode] Starting state estimator loop at %.1f Hz",
             loop_frequency);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();
        const double now_sec = ros::Time::now().toSec();
        runtime_.update(now_sec);
        dispatchOutputEvents(runtime_.getStateMachine().currentOutputEvents());
        rate.sleep();
    }

    ROS_INFO("[VrpnPx4RotorStateEstimatorNode] State estimator loop exited");
}

double VrpnPx4RotorStateEstimatorNode::loopRateHz() const {
    return loop_rate_hz_;
}

void VrpnPx4RotorStateEstimatorNode::loadParams() {
    ros1_utils::getParamWithLog(private_nh_, "imu_topic", imu_topic_, "IMU topic");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_pose_topic", vrpn_pose_topic_,
                                "VRPN pose topic");
    ros1_utils::getParamWithLog(private_nh_, "state_topic", state_topic_, "State topic");
    ros1_utils::getParamWithLog(private_nh_, "vision_pose_topic", vision_pose_topic_,
                                "Vision pose topic");

    ros1_utils::getParamWithLog(private_nh_, "loop_rate_hz", loop_rate_hz_, "Loop rate");
    ros1_utils::getParamWithLog(private_nh_, "state_publish_rate_hz", config_.state_publish_rate_hz,
                                "State publish rate");
    ros1_utils::getParamWithLog(private_nh_, "vision_publish_rate_hz",
                                config_.vision_publish_rate_hz, "Vision publish rate");
    config_.loop_rate_hz = loop_rate_hz_;

    ros1_utils::getParamWithLog(private_nh_, "gravity_mps2", config_.gravity_mps2, "Gravity");
    readPoseParams(private_nh_, "field_offset_xyz", "field_offset_rpy", config_.field_to_world,
                   "Field offset");
    readPoseParams(private_nh_, "imu_to_vrpn_marker_xyz", "imu_to_vrpn_marker_rpy",
                   config_.imu_to_vrpn_marker, "IMU to VRPN marker");
    ros1_utils::getParamWithLog(private_nh_, "extrinsic_verified", config_.extrinsic_verified,
                                "Extrinsic verified");
    ros1_utils::getParamWithLog(private_nh_, "estimate_extrinsic", config_.estimate_extrinsic,
                                "Estimate extrinsic");

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

    ros1_utils::getParamWithLog(private_nh_, "accel_noise_std", config_.accel_noise_std,
                                "Accelerometer noise std");
    ros1_utils::getParamWithLog(private_nh_, "gyro_noise_std", config_.gyro_noise_std,
                                "Gyroscope noise std");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_position_noise_std",
                                config_.vrpn_position_noise_std, "VRPN position noise std");
    ros1_utils::getParamWithLog(private_nh_, "vrpn_orientation_noise_std",
                                config_.vrpn_orientation_noise_std, "VRPN orientation noise std");
    ros1_utils::getParamWithLog(private_nh_, "gyro_bias_random_walk_std",
                                config_.gyro_bias_random_walk_std, "Gyro bias random walk std");
    ros1_utils::getParamWithLog(private_nh_, "accel_bias_random_walk_std",
                                config_.accel_bias_random_walk_std, "Accel bias random walk std");
    ros1_utils::getParamWithLog(private_nh_, "extrinsic_position_random_walk_std",
                                config_.extrinsic_position_random_walk_std,
                                "Extrinsic position random walk std");
    ros1_utils::getParamWithLog(private_nh_, "extrinsic_orientation_random_walk_std",
                                config_.extrinsic_orientation_random_walk_std,
                                "Extrinsic orientation random walk std");
    ros1_utils::getParamWithLog(private_nh_, "innovation_position_gate_m",
                                config_.innovation_position_gate_m, "Position innovation gate");
    ros1_utils::getParamWithLog(private_nh_, "innovation_orientation_gate_rad",
                                config_.innovation_orientation_gate_rad,
                                "Orientation innovation gate");
    ros1_utils::getParamWithLog(private_nh_, "pose_nis_gate", config_.pose_nis_gate,
                                "Pose NIS gate");
    ros1_utils::getParamWithLog(private_nh_, "covariance_high_threshold",
                                config_.covariance_high_threshold, "Covariance high threshold");
    ros1_utils::getParamWithLog(private_nh_, "max_propagation_dt_s", config_.max_propagation_dt_s,
                                "Max propagation dt");
    int inertial_buffer_capacity = static_cast<int>(config_.inertial_buffer_capacity);
    ros1_utils::getParamWithLog(private_nh_, "inertial_buffer_capacity", inertial_buffer_capacity,
                                "Inertial history capacity");
    if (inertial_buffer_capacity > 0) {
        config_.inertial_buffer_capacity = static_cast<std::size_t>(inertial_buffer_capacity);
    }
    ros1_utils::getParamWithLog(private_nh_, "initial_position_variance",
                                config_.initial_position_variance, "Initial position variance");
    ros1_utils::getParamWithLog(private_nh_, "initial_velocity_variance",
                                config_.initial_velocity_variance, "Initial velocity variance");
    ros1_utils::getParamWithLog(private_nh_, "initial_orientation_variance",
                                config_.initial_orientation_variance,
                                "Initial orientation variance");
    ros1_utils::getParamWithLog(private_nh_, "initial_gyro_bias_variance",
                                config_.initial_gyro_bias_variance, "Initial gyro bias variance");
    ros1_utils::getParamWithLog(private_nh_, "initial_accel_bias_variance",
                                config_.initial_accel_bias_variance, "Initial accel bias variance");

    config_utils::normalizeConfig(config_);
    loop_rate_hz_ = config_.loop_rate_hz;
}

void VrpnPx4RotorStateEstimatorNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    const auto result = output_event_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[VrpnPx4RotorStateEstimatorNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[VrpnPx4RotorStateEstimatorNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace estimator_vrpn_px4_rotor_state
