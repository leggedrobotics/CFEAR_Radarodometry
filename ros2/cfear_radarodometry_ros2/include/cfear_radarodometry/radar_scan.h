#pragma once

#include <cstdint>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace CFEAR_Radarodometry {

// Radar geometry, normally taken from the driver's RadarConfigurationMsg,
// with node parameters as fallback.
struct RadarConfig {
  uint16_t azimuth_samples = 400;  // rows of the polar scan
  uint16_t encoder_size = 5600;    // encoder ticks per rotation
  uint16_t range_in_bins = 3768;   // cols of the polar scan
  float bin_size = 0.0596f;        // range resolution [m]
  double rotation_rate_hz = 4.0;
  // Image layout (leggedrobotics RadarConfigurationMsg extension): leading
  // per-row metadata columns (Oxford/Boreas 11-byte layout) and the range crop
  // [start_bin, end_bin) the published image covers. end_bin == 0 means unknown
  // (uncropped / config not received).
  uint16_t metadata_columns = 0;
  uint16_t start_bin = 0;
  uint16_t end_bin = 0;
};

// One whole radar rotation, as published by the driver's polar_image_publisher
// (metadata columns already stripped).
struct CompletedScan {
  cv::Mat polar;              // mono8, rows = azimuths, cols = range bins
  rclcpp::Time stamp_first;   // stamp of the first azimuth of the rotation
  rclcpp::Time stamp_last;    // stamp of the last azimuth of the rotation
  int missing_azimuths = 0;   // rows the publisher did not flag valid
};

}
