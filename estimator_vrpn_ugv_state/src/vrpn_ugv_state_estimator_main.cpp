#include <ros/ros.h>

#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "vrpn_ugv_state_estimator_node");
    ros::NodeHandle nh;

    estimator_vrpn_ugv_state::VrpnUgvStateEstimatorNode node(nh);
    const double loop_frequency_hz = node.loopRateHz();
    ROS_INFO("[VrpnUgvStateEstimatorNode] Configured loop frequency: %.1f Hz", loop_frequency_hz);
    node.run(loop_frequency_hz);
    return 0;
}
