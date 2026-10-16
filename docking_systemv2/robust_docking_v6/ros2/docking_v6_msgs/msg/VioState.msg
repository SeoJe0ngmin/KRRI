std_msgs/Header header
uint64 epoch
bool initialized
bool visual_valid
bool zero_velocity_update
uint32 features
# Row-major T_odom_imu, never JPL quaternion coefficients interpreted as Hamilton.
float64[16] t_odom_imu
# [world position, world LEFT rotation] error covariance, row-major.
float64[36] covariance
float64[3] gyro_bias
float64[3] accel_bias
float64[3] velocity_world
float64 gyro_norm
float64 last_imu_stamp
uint64 stereo_dropped
geometry_msgs/Point32[] left_points
string fault
# Relative motion uncertainty uses the previous cloned pose/current IMU joint P.
bool delta_valid
float64 delta_from_stamp
float64[16] relative_transform
float64[36] relative_covariance
