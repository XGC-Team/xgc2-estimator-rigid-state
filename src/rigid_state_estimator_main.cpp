#include <ros/ros.h>

#include "estimator_rigid_state/rigid_state_estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rigid_state_estimator_node");
    ros::NodeHandle nh;

    estimator_rigid_state::RigidStateEstimatorNode node(nh);
    const double loop_frequency_hz = node.loopRateHz();
    ROS_INFO("[RigidStateEstimatorNode] Configured loop frequency: %.1f Hz", loop_frequency_hz);
    node.run(loop_frequency_hz);
    return 0;
}
