# A ROS message based on a configuration data message from a radar, with corrected types
# Vendored from the official Navtech IA SDK (bitbucket.org/navtechradar/iasdk-public,
# ros/ros2/src/navtech_msgs). Field names/types must stay identical to the driver's
# copy so the DDS types match on the wire.

# add a header message to hold message timestamp
std_msgs/Header header

# azimuth_samples (uint16)
uint16 azimuth_samples

# encoder_size (uint16)
uint16 encoder_size

# user-provided azimuth offset (uint16)
uint16 azimuth_offset

# bin_size (float32)
float32 bin_size

# range_in_bins (uint16)
uint16 range_in_bins

# expected_rotation_rate (uint16)
uint16 expected_rotation_rate

# range_gain (float32)
float32 range_gain

# range_offset (float32)
float32 range_offset

# --- polar-frame image pipeline (leggedrobotics fork extension) ---
# Note: polar_image_publisher fills bin_size in METERS and
# expected_rotation_rate in HZ (stock nodes publish raw Colossus units:
# 0.1 mm and mHz respectively).

# Leading columns of each radar_frame image row that carry per-azimuth
# metadata (Oxford/Boreas layout: bytes 0-7 uint64 LE UNIX time [us],
# bytes 8-9 uint16 LE encoder tick, byte 10 valid flag). 0 = no metadata.
uint16 metadata_columns

# Range crop applied to the published image: bins [start_bin, end_bin).
# Image width = metadata_columns + (end_bin - start_bin).
# range_in_bins above stays the sensor's full native bin count.
uint16 start_bin
uint16 end_bin
