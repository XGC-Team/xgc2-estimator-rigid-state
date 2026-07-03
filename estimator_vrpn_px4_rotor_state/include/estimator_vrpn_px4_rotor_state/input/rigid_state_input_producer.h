#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "estimator_vrpn_px4_rotor_state/common/types.h"

namespace estimator_vrpn_px4_rotor_state {

class RigidStateInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(
        ::state_machine::Event, const VrpnPx4RotorStateEstimatorInput&)>;

    RigidStateInputProducer(ros::NodeHandle& nh, std::string imu_topic, std::string vrpn_pose_topic,
                            std::string vrpn_twist_topic, uint32_t queue_size,
                            EventSink event_sink);

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source,
                        double timestamp_sec);
    static ros::Time messageStampOrNow(const ros::Time& stamp);
    static void updateImuPeriod(xgc2_math::InertialSample& sample, double stamp_sec);
    static void updatePosePeriod(xgc2_math::PoseMeasurement& sample, double stamp_sec);
    static void updateVelocityPeriod(xgc2_math::VelocityMeasurement& sample, double stamp_sec);

    EventSink event_sink_;
    VrpnPx4RotorStateEstimatorInput runtime_input_{};
    ros::Subscriber imu_sub_;
    ros::Subscriber vrpn_pose_sub_;
    ros::Subscriber vrpn_twist_sub_;
};

}  // namespace estimator_vrpn_px4_rotor_state
