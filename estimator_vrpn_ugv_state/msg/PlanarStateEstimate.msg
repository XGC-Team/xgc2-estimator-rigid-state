std_msgs/Header header

uint8 STATE_SELF_CHECK=1
uint8 STATE_RUNNING=2
uint8 STATE_COASTING=3
uint8 STATE_FAULT=4

uint32 FLAG_IMU_MISSING=1
uint32 FLAG_VRPN_MISSING=2
uint32 FLAG_IMU_STALE=4
uint32 FLAG_VRPN_STALE=8
uint32 FLAG_IMU_RATE_LOW=16
uint32 FLAG_VRPN_RATE_LOW=32
uint32 FLAG_TIME_JUMP=64
uint32 FLAG_COASTING=128
uint32 FLAG_FAULT=256
uint32 FLAG_INNOVATION_REJECTED=512
uint32 FLAG_EXTRINSIC_UNVERIFIED=1024
uint32 FLAG_COVARIANCE_HIGH=2048
uint32 FLAG_INVALID_IMU=4096
uint32 FLAG_INVALID_VRPN=8192

uint8 estimator_state
uint32 flags
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
