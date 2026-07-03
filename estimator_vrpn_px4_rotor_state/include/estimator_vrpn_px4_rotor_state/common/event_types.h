#pragma once

#include <cstdint>
#include <state_machine/state_machine.hpp>

namespace estimator_vrpn_px4_rotor_state {

namespace region_type {
constexpr ::state_machine::RegionId HEALTH = 1;
constexpr ::state_machine::RegionId ESTIMATION = 2;
}  // namespace region_type

namespace state_type {
constexpr ::state_machine::StateId SelfCheck = 1;
constexpr ::state_machine::StateId Initializing = 2;
constexpr ::state_machine::StateId Running = 3;
constexpr ::state_machine::StateId Coasting = 4;
constexpr ::state_machine::StateId Fault = 5;
constexpr ::state_machine::StateId HealthMonitor = 100;
}  // namespace state_type

namespace event_type {
constexpr ::state_machine::EventId INPUT_IMU_UPDATED = 1001;
constexpr ::state_machine::EventId INPUT_VRPN_POSE_UPDATED = 1002;
constexpr ::state_machine::EventId INPUT_VRPN_VELOCITY_UPDATED = 1003;

constexpr ::state_machine::EventId HEALTH_TO_SELF_CHECK = 2001;
constexpr ::state_machine::EventId HEALTH_TO_INITIALIZING = 2002;
constexpr ::state_machine::EventId HEALTH_TO_RUNNING = 2003;
constexpr ::state_machine::EventId HEALTH_TO_COASTING = 2004;
constexpr ::state_machine::EventId HEALTH_TO_FAULT = 2005;
}  // namespace event_type

namespace output_event_type {
constexpr ::state_machine::EventId PUBLISH_STATE = 3001;
constexpr ::state_machine::EventId PUBLISH_VISION_POSE = 3002;
}  // namespace output_event_type

namespace transition_priority {
constexpr int FAULT = 100;
constexpr int AUTOMATIC = 10;
}  // namespace transition_priority

}  // namespace estimator_vrpn_px4_rotor_state
