#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <state_machine/state_machine.hpp>
#include <string>
#include <vector>

#include "estimator_vrpn_ugv_state/input/planar_state_input_producer.h"
#include "estimator_vrpn_ugv_state/output/planar_state_output_consumer.h"
#include "estimator_vrpn_ugv_state/vrpn_ugv_state_estimator_runtime.h"

namespace estimator_vrpn_ugv_state {

class VrpnUgvStateEstimatorNode {
   public:
    explicit VrpnUgvStateEstimatorNode(ros::NodeHandle& nh);
    ~VrpnUgvStateEstimatorNode();

    void run(double frequency);
    double loopRateHz() const;

   private:
    void loadParams();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle& nh_;
    ros::NodeHandle private_nh_;
    VrpnUgvStateEstimatorRuntime runtime_{};
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_event_executor_;
    ::state_machine::runtime::EventDispatcher output_event_dispatcher_;
    std::unique_ptr<PlanarStateInputProducer> input_producer_;

    VrpnUgvStateEstimatorConfig config_{};
    std::string imu_topic_{"mavros/imu/data_raw"};
    std::string vrpn_pose_topic_{"/vrpn_client_node/ugv1/pose"};
    std::string state_topic_{"alg/state_estimator/state"};
    double loop_rate_hz_{1000.0};
};

}  // namespace estimator_vrpn_ugv_state
