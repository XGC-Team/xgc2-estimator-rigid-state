#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <state_machine/state_machine.hpp>
#include <string>
#include <vector>

#include "estimator_vrpn_px4_rotor_state/input/rigid_state_input_producer.h"
#include "estimator_vrpn_px4_rotor_state/output/rigid_state_output_consumer.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace estimator_vrpn_px4_rotor_state {

class VrpnPx4RotorStateEstimatorNode {
   public:
    explicit VrpnPx4RotorStateEstimatorNode(ros::NodeHandle& nh);
    ~VrpnPx4RotorStateEstimatorNode();

    void run(double frequency);
    double loopRateHz() const;

   private:
    void loadParams();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);
    void publishStateTimerCallback(const ros::TimerEvent& event);
    void dispatchTimerOutputEvent(::state_machine::EventId event_id, const ros::Time& stamp,
                                  const char* source);

    ros::NodeHandle& nh_;
    ros::NodeHandle private_nh_;
    VrpnPx4RotorStateEstimatorRuntime runtime_{};
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_event_executor_;
    ::state_machine::runtime::EventDispatcher output_event_dispatcher_;
    std::unique_ptr<RigidStateInputProducer> input_producer_;
    ros::Timer state_publish_timer_;

    VrpnPx4RotorStateEstimatorConfig config_{};
    std::string imu_topic_{"mavros/imu/data_raw"};
    std::string vrpn_pose_topic_{"/vrpn_client_node/uav1/pose"};
    std::string state_topic_{"alg/state_estimator/state"};
    std::string vision_pose_topic_{"mavros/vision_pose/pose"};
    double loop_rate_hz_{1000.0};
};

}  // namespace estimator_vrpn_px4_rotor_state
