#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "estimator_vrpn_ugv_state/common/types.h"

namespace estimator_vrpn_ugv_state {

class PlanarStateInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event,
                                                            const VrpnUgvStateEstimatorInput&)>;

    PlanarStateInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                             std::string vrpn_pose_topic, uint32_t queue_size,
                             EventSink event_sink);

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source,
                        double timestamp_sec);

    EventSink event_sink_;
    VrpnUgvStateEstimatorInput runtime_input_{};
    ros::Subscriber imu_sub_;
    ros::Subscriber vrpn_pose_sub_;
};

}  // namespace estimator_vrpn_ugv_state
