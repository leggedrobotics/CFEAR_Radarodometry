#include "cfear_radarodometry/fft_assembler.h"

#include <algorithm>
#include <cstring>

namespace CFEAR_Radarodometry {

void FftRotationAssembler::SetConfig(const RadarConfig& cfg){
  cfg_ = cfg;
  configured_ = true;
  ResetScan();
  last_tick_ = -1;
  last_sweep_ = -1;
}

void FftRotationAssembler::ResetScan(){
  // Must allocate a FRESH buffer: `scan_ = cv::Mat::zeros(...)` would write into
  // the existing allocation when size/type match, which is shared with the
  // CompletedScan already handed to the consumer (data race / scan corruption).
  scan_ = cv::Mat(cfg_.azimuth_samples, cfg_.range_in_bins, CV_8UC1, cv::Scalar(0));
  row_written_.assign(cfg_.azimuth_samples, false);
  rows_filled_ = 0;
  scan_started_ = false;
}

std::optional<CompletedScan> FftRotationAssembler::AddAzimuth(
    uint16_t azimuth_tick, uint16_t sweep_counter,
    const uint8_t* data, size_t len, const rclcpp::Time& stamp){

  if(!configured_ || data == nullptr)
    return std::nullopt;

  std::optional<CompletedScan> completed;

  // Rotation boundary: encoder tick wrapped around or the sweep counter changed.
  const bool wrapped = scan_started_ &&
      ((last_tick_ >= 0 && static_cast<int>(azimuth_tick) < last_tick_) ||
       (last_sweep_ >= 0 && static_cast<int>(sweep_counter) != last_sweep_));
  if(wrapped){
    CompletedScan out;
    out.polar = scan_;  // handing over ownership; ResetScan() allocates a new Mat
    out.stamp_first = first_stamp_;
    out.stamp_last = last_stamp_;
    out.missing_azimuths = static_cast<int>(cfg_.azimuth_samples) - rows_filled_;
    completed = std::move(out);
    ResetScan();
  }

  // Nearest-integer encoder-tick -> row mapping (supports encoder_size that is
  // not an integer multiple of azimuth_samples, e.g. future RAS3 sensors).
  const int row = (static_cast<int>(azimuth_tick) * cfg_.azimuth_samples
                   + cfg_.encoder_size / 2) / cfg_.encoder_size;

  if(row >= 0 && row < static_cast<int>(cfg_.azimuth_samples) && !row_written_[row]){
    const size_t n = std::min(len, static_cast<size_t>(cfg_.range_in_bins));
    std::memcpy(scan_.ptr(row), data, n); // remaining bins stay zero
    row_written_[row] = true;
    rows_filled_++;
    if(!scan_started_){
      scan_started_ = true;
      first_stamp_ = stamp;
    }
    last_stamp_ = stamp;
  }

  last_tick_ = azimuth_tick;
  last_sweep_ = sweep_counter;
  return completed;
}

}
