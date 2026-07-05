#pragma once

#include <state_machine/state_machine.hpp>

namespace estimator_vrpn_ugv_state {

namespace region_type {
constexpr ::state_machine::RegionId HEALTH = 1;
constexpr ::state_machine::RegionId ESTIMATION = 2;
}  // namespace region_type

namespace state_type {
constexpr ::state_machine::StateId SelfCheck = 1;
constexpr ::state_machine::StateId Running = 2;
constexpr ::state_machine::StateId Coasting = 3;
constexpr ::state_machine::StateId HealthMonitor = 100;
}  // namespace state_type

namespace event_type {
constexpr ::state_machine::EventId INPUT_IMU_UPDATED = 1001;
constexpr ::state_machine::EventId INPUT_VRPN_POSE_UPDATED = 1002;

constexpr ::state_machine::EventId HEALTH_INPUT_UNHEALTHY = 2001;
constexpr ::state_machine::EventId HEALTH_ESTIMATION_READY = 2002;
constexpr ::state_machine::EventId HEALTH_VRPN_LOSS_COASTABLE = 2003;
}  // namespace event_type

namespace output_event_type {
constexpr ::state_machine::EventId PUBLISH_STATE = 3001;
}  // namespace output_event_type

namespace transition_priority {
constexpr int AUTOMATIC = 10;
}  // namespace transition_priority

}  // namespace estimator_vrpn_ugv_state
