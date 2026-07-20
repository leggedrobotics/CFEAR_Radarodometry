#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace CFEAR_Radarodometry {

// Radar geometry, normally taken from the driver's RadarConfigurationMsg,
// with node parameters as fallback.
struct RadarConfig {
  uint16_t azimuth_samples = 400;  // rows of the assembled polar scan
  uint16_t encoder_size = 5600;    // encoder ticks per rotation
  uint16_t range_in_bins = 3768;   // cols of the assembled polar scan
  float bin_size = 0.0596f;        // range resolution [m]
  double rotation_rate_hz = 4.0;
};

struct CompletedScan {
  cv::Mat polar;              // mono8, rows = azimuths, cols = range bins
  rclcpp::Time stamp_first;   // stamp of the first azimuth of the rotation
  rclcpp::Time stamp_last;    // stamp of the last azimuth of the rotation
  int missing_azimuths = 0;   // rows that stayed empty (dropped FFT messages)
};

/* Assembles per-azimuth Navtech FFT messages into full polar rotations.
 *
 * Row index: nearest-integer mapping of encoder tick to azimuth row. A rotation
 * is considered complete when the encoder tick wraps (decreases) or the sweep
 * counter changes; rows never received stay zero (harmless: the k-strongest
 * filter thresholds at z_min anyway). First write wins on duplicate rows. */
class FftRotationAssembler {
public:
  void SetConfig(const RadarConfig& cfg);

  bool Configured() const { return configured_; }

  const RadarConfig& Config() const { return cfg_; }

  // Returns the completed rotation when `azimuth_tick` starts a new one.
  std::optional<CompletedScan> AddAzimuth(uint16_t azimuth_tick,
                                          uint16_t sweep_counter,
                                          const uint8_t* data, size_t len,
                                          const rclcpp::Time& stamp);

private:
  void ResetScan();

  RadarConfig cfg_;
  bool configured_ = false;

  cv::Mat scan_;
  std::vector<bool> row_written_;
  int rows_filled_ = 0;
  bool scan_started_ = false;
  rclcpp::Time first_stamp_, last_stamp_;
  int last_tick_ = -1;
  int last_sweep_ = -1;
};

}
