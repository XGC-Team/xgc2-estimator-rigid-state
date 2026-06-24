#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <state_machine/state_machine.hpp>
#include <string>
#include <vector>

#include "estimator_rigid_state/input/rigid_state_input_producer.h"
#include "estimator_rigid_state/output/rigid_state_output_consumer.h"
#include "estimator_rigid_state/rigid_state_estimator_runtime.h"

namespace estimator_rigid_state {

class RigidStateEstimatorNode {
   public:
    explicit RigidStateEstimatorNode(ros::NodeHandle& nh);
    ~RigidStateEstimatorNode();

    void run(double frequency);
    double loopRateHz() const;

   private:
    void loadParams();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle& nh_;
    ros::NodeHandle private_nh_;
    RigidStateEstimatorRuntime runtime_{};
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_event_executor_;
    ::state_machine::runtime::EventDispatcher output_event_dispatcher_;
    std::unique_ptr<RigidStateInputProducer> input_producer_;

    RigidStateEstimatorConfig config_{};
    std::string imu_topic_{"mavros/imu/data_raw"};
    std::string vrpn_pose_topic_{"/vrpn_client_node/uav1/pose"};
    std::string state_topic_{"alg/state_estimator/state"};
    std::string vision_pose_topic_{"mavros/vision_pose/pose"};
    double loop_rate_hz_{1000.0};
};

}  // namespace estimator_rigid_state
