#include <ros/ros.h>

#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "vrpn_px4_rotor_state_estimator_node");
    ros::NodeHandle nh;

    estimator_vrpn_px4_rotor_state::VrpnPx4RotorStateEstimatorNode node(nh);
    const double loop_frequency_hz = node.loopRateHz();
    ROS_INFO("[VrpnPx4RotorStateEstimatorNode] Configured loop frequency: %.1f Hz",
             loop_frequency_hz);
    node.run(loop_frequency_hz);
    return 0;
}
