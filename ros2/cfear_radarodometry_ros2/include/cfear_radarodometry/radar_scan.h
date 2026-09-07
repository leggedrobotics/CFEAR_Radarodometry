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
  cv::Mat polar;             // mono8, rows = azimuths, cols = range bins
  int missing_azimuths = 0;  // rows the publisher did not flag valid

  // Two INDEPENDENT stamp families - see radar.stamp_source in the node. Keep
  // them apart: "the first row" and "the earliest spoke" are NOT the same thing.
  //
  // ROW-indexed: the embedded metadata time of the lowest / highest valid ROW
  // INDEX. Which row holds which spoke is a property of the publisher's
  // tick -> row mapping, so these are only the temporal ends of the rotation by
  // coincidence. They are on Boreas; they are NOT on the b2w RAS-3, where
  // polar_image_publisher's row = round(tick*400/16000) % 400 wraps tick 16000
  // onto row 0, putting the LAST spoke of the sweep at row 0. There both
  // stamp_first_row and stamp_last_row sit at the sweep END, ~249 ms after the
  // rotation began.
  rclcpp::Time stamp_first_row;
  rclcpp::Time stamp_last_row;

  // TIME-indexed: min / max over the valid rows' embedded times, i.e. the real
  // temporal extent of the rotation whatever the row mapping is. Prefer these:
  // they stay correct if the mapping ever changes, and they are invariant under
  // radar.reverse_azimuths (row reversal permutes rows, not the set of times).
  rclcpp::Time stamp_sweep_start;
  rclcpp::Time stamp_sweep_end;

  // Midpoint of the acquisition interval, and the instant the registered pose
  // actually refers to: OdometryKeyframeFuser deskews every scan with
  // Compensate(), whose GetRelTimeStamp() = +-(atan2(y,x)/2pi - 0.5) is zero at
  // half a rotation, so all points are transported to mid-sweep.
  //
  // Caveat: this is the midpoint of the OBSERVED interval. Azimuths dropped at
  // one temporal end bias it by half the dropped span (bounded by
  // radar.max_missing_fraction; the node warns whenever any row is missing).
  rclcpp::Time stamp_sweep_mid() const {
    return stamp_sweep_start + (stamp_sweep_end - stamp_sweep_start) * 0.5;
  }
};

}
