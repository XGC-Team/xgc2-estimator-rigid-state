/* Domain-owned rigid-state wire records. Payload schemas, fields and bytes
 * are unchanged from the native adapter migration; no estimator policy or
 * Runtime/ROS/library dependency is introduced by this header. */
#ifndef XGC_RIGID_STATE_WIRE_V1_H
#define XGC_RIGID_STATE_WIRE_V1_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct xgc_rigid_state_v1 {
    double stamp;
    double position[3];
    double velocity[3];
    double q_wxyz[4];
    double body_rate[3];
} xgc_rigid_state_v1;

typedef struct xgc_rigid_state_estimate_v1 {
    double stamp;
    double position[3];
    double velocity[3];
    double q_wxyz[4];
    double angular_velocity[3];
    double linear_acceleration[3];
    double gravity[3];
    double accel_bias[3];
    double last_fused_pose_stamp_sec;
    double vrpn_innovation_window_chi_square;
    double last_pose_position_innovation_norm_m;
    double last_pose_orientation_innovation_norm_rad;
    double last_pose_mahalanobis_distance;
    double innovation_position_gate_m;
    double innovation_orientation_gate_rad;
    double pose_nis_gate;
    double last_imu_sample_stamp_sec;
    double last_vrpn_pose_stamp_sec;
    double filter_inertial_stamp_sec;
    double filter_pose_stamp_sec;
    uint32_t flags;
    uint32_t vrpn_consecutive_rejects;
    uint32_t vrpn_consecutive_accepts;
    uint8_t estimator_state;
    uint8_t vrpn_observation_state;
    uint8_t filter_health;
    uint8_t last_pose_reject_reason;
    uint8_t last_pose_accepted;
    uint8_t reserved[7];
} xgc_rigid_state_estimate_v1;

#ifdef __cplusplus
#define XGC_RIGID_WIRE_ASSERT static_assert
#else
#define XGC_RIGID_WIRE_ASSERT _Static_assert
#endif
XGC_RIGID_WIRE_ASSERT(sizeof(xgc_rigid_state_v1) == 112, "xgc_rigid_state_v1");
XGC_RIGID_WIRE_ASSERT(sizeof(xgc_rigid_state_estimate_v1) == 304, "xgc_rigid_state_estimate_v1");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_v1, stamp) == 0, "stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_v1, position) == 8, "position offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_v1, velocity) == 32, "velocity offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_v1, q_wxyz) == 56, "orientation offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_v1, body_rate) == 88, "body rate offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, stamp) == 0, "estimate stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, position) == 8,
                      "estimate position offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, velocity) == 32,
                      "estimate velocity offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, q_wxyz) == 56,
                      "estimate orientation offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, angular_velocity) == 88,
                      "angular velocity offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, linear_acceleration) == 112,
                      "linear acceleration offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, gravity) == 136, "gravity offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, accel_bias) == 160,
                      "accel bias offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_fused_pose_stamp_sec) == 184,
                      "last fused pose stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, vrpn_innovation_window_chi_square) ==
                          192,
                      "innovation chi square offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_pose_position_innovation_norm_m) ==
                          200,
                      "position innovation offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1,
                               last_pose_orientation_innovation_norm_rad) == 208,
                      "orientation innovation offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_pose_mahalanobis_distance) == 216,
                      "mahalanobis distance offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, innovation_position_gate_m) == 224,
                      "position gate offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, innovation_orientation_gate_rad) == 232,
                      "orientation gate offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, pose_nis_gate) == 240,
                      "pose NIS gate offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_imu_sample_stamp_sec) == 248,
                      "last IMU stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_vrpn_pose_stamp_sec) == 256,
                      "last VRPN stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, filter_inertial_stamp_sec) == 264,
                      "filter inertial stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, filter_pose_stamp_sec) == 272,
                      "filter pose stamp offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, flags) == 280, "flags offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, vrpn_consecutive_rejects) == 284,
                      "VRPN rejects offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, vrpn_consecutive_accepts) == 288,
                      "VRPN accepts offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, estimator_state) == 292,
                      "state offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, vrpn_observation_state) == 293,
                      "observation state offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, filter_health) == 294,
                      "filter health offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_pose_reject_reason) == 295,
                      "reject reason offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, last_pose_accepted) == 296,
                      "accepted offset");
XGC_RIGID_WIRE_ASSERT(offsetof(xgc_rigid_state_estimate_v1, reserved) == 297, "reserved offset");
#undef XGC_RIGID_WIRE_ASSERT
#ifdef __cplusplus
}
#endif
#endif
