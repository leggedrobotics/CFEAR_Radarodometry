# A ROS message based on an FFT data message from a radar, with corrected types
# Vendored from the official Navtech IA SDK (bitbucket.org/navtechradar/iasdk-public,
# ros/ros2/src/navtech_msgs). Field names/types must stay identical to the driver's
# copy so the DDS types match on the wire.

# add a header message to hold message timestamp
std_msgs/Header header

# angle (float64)
float64 angle

# azimuth (uint16) - encoder tick
uint16 azimuth

# sweep_counter (uint16)
uint16 sweep_counter

# ntp_seconds (uint32)
uint32 ntp_seconds

# ntp_split_seconds (uint32)
uint32 ntp_split_seconds

# data (uint8[]) - one azimuth of FFT power returns
uint8[] data

# data_length (uint16)
uint16 data_length
