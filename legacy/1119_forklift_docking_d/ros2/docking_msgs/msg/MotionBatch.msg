# IMU samples drained by the sensor adapter since its previous flush, in time order (2026-11-01):
# one message per flush (~60/s) instead of one Motion per sample (200/s). sequence numbers are
# first_sequence, first_sequence+1, ...; gyro/accel hold 3 values per sample, row-major.
std_msgs/Header header
uint64 epoch
uint64 first_sequence
float64[] stamps
float64[] gyro
float64[] accel
