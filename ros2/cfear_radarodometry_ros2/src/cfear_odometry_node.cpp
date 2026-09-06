/* Online CFEAR radar odometry node (ROS2 Jazzy).
 *
 * Pipeline:
 *   sensor_msgs/Image whole-rotation polar frame (mono8, rows=azimuths, e.g.
 *   /radar_data/radar_frame)
 *     -> radarDriver (k-strongest filtering -> point clouds)
 *     -> OdometryKeyframeFuser (CFEAR registration -> nav_msgs/Odometry on /odometry [+ tf])
 *
 * Frames come from the leggedrobotics navtech_radar_sdk polar_image_publisher or
 * from radarsplat_replay, carrying the Oxford/Boreas 11-column per-row metadata
 * prefix (bytes 0-7 uint64 LE UNIX time [us], bytes 8-9 uint16 LE encoder tick,
 * byte 10 valid flag). The scan arrives already assembled: this node never sees
 * per-azimuth FFT spokes and never assembles rotations itself.
 *
 * Radar geometry (azimuth samples, encoder size, range bins, bin size, metadata
 * columns + range crop) is taken from the driver's radar configuration message
 * when available; after radar.config_timeout_sec it falls back to the radar.*
 * parameters.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navtech_msgs/msg/radar_configuration_msg.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "cfear_radarodometry/odometrykeyframefuser.h"
#include "cfear_radarodometry/radar_driver.h"
#include "cfear_radarodometry/radar_scan.h"

using CFEAR_Radarodometry::CompletedScan;
using CFEAR_Radarodometry::OdometryKeyframeFuser;
using CFEAR_Radarodometry::RadarConfig;
using CFEAR_Radarodometry::radarDriver;

class CfearOdometryNode : public rclcpp::Node {
public:
  CfearOdometryNode() : rclcpp::Node("cfear_odometry_node") {
    DeclareParameters();

    const std::string image_topic = get_parameter("radar.image_topic").as_string();
    const std::string config_topic = get_parameter("radar.config_topic").as_string();

    rclcpp::QoS config_qos(rclcpp::KeepLast(1));
    config_qos.reliable();
    if (get_parameter("radar.config_durability").as_string() == "transient_local")
      config_qos.transient_local();

    // Whole assembled rotations at a few Hz - a shallow reliable queue matching
    // the driver/replay frame publishers is enough.
    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    image_qos.reliable();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, image_qos,
        std::bind(&CfearOdometryNode::ImageCallback, this, std::placeholders::_1));
    config_sub_ = create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        config_topic, config_qos,
        std::bind(&CfearOdometryNode::ConfigCallback, this, std::placeholders::_1));

    const double timeout = get_parameter("radar.config_timeout_sec").as_double();
    config_timeout_timer_ = create_wall_timer(
        std::chrono::duration<double>(timeout),
        std::bind(&CfearOdometryNode::ConfigTimeout, this));

    worker_ = std::thread(&CfearOdometryNode::WorkerLoop, this);

    RCLCPP_INFO(get_logger(),
                "Listening for polar radar frames on '%s' (reliable), radar configuration on '%s'",
                image_topic.c_str(), config_topic.c_str());
  }

  ~CfearOdometryNode() override {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
    if (fuser_ != nullptr)
      RCLCPP_INFO(get_logger(), "%s", fuser_->GetStatus().c_str());
  }

private:
  void DeclareParameters() {
    // radar input / geometry fallbacks (Boreas-like Navtech CIR204-H defaults)
    declare_parameter<std::string>("radar.config_topic", "/radar_data/configuration_data");
    declare_parameter<std::string>("radar.config_durability", "transient_local"); // or "volatile"
    declare_parameter<double>("radar.config_timeout_sec", 5.0);
    // Whole-rotation polar frames (sensor_msgs/Image, mono8) from the
    // navtech_radar_sdk polar_image_publisher / radarsplat_replay, plus the
    // metadata-column fallback used when no RadarConfigurationMsg announces the
    // layout within the timeout.
    declare_parameter<std::string>("radar.image_topic", "/radar_data/radar_frame");
    declare_parameter<int>("radar.metadata_columns", 11);
    // Keep at most this many RANGE BINS per azimuth (columns after the metadata
    // prefix), i.e. crop the scan to max_range_bins * bin_size metres.
    // 0 = use the frame as published. This is
    // the consumer-side equivalent of the driver's start_bin/end_bin crop, for
    // recordings that were published at full range; every downstream stage
    // (k-strongest, surface points, registration) scales with the bin count.
    declare_parameter<int>("radar.max_range_bins", 0);
    // Reverse the azimuth ROW ORDER of every scan before it reaches the driver.
    //
    // radar_filters.cpp maps row -> bearing as theta = (row+1)/rows * 2*pi with
    // x = r*cos(theta), y = r*sin(theta): azimuth index increasing means
    // COUNTER-clockwise. That holds for Boreas, whose radar frame is z-down, but
    // the RAS-3 on b2w is read in a REP-103 frame (x fwd, y left, z UP) where the
    // very same physical rotation runs CLOCKWISE - the encoder tick increases with
    // a NEGATIVE bearing. Feeding it in unreversed reflects every scan about the
    // x axis, and because scan matching is equivariant under reflection the
    // odometry comes out self-consistent but mirrored: y -> -y, yaw -> -yaw.
    // (Measured on hg_nav_4: CFEAR net yaw +50.4 deg vs the DLIO reference's
    // -49.30 deg - right magnitude, wrong sign.)
    //
    // Reversing the rows is exact: row r -> rows-1-r gives
    // theta = 2*pi*(rows-r)/rows == -2*pi*r/rows, which is the clockwise mapping
    // AND cancels the upstream +1, removing its one-row (0.9 deg) bearing bias.
    //
    // NOTE this also reverses how acquisition time maps to bearing, so it must be
    // paired with the opposite fuser.radar_ccw - see that parameter.
    // Default false = upstream/Boreas behaviour; the b2w preset turns it on.
    declare_parameter<bool>("radar.reverse_azimuths", false);
    declare_parameter<int>("radar.azimuth_samples", 400);
    declare_parameter<int>("radar.encoder_size", 5600);
    declare_parameter<int>("radar.range_in_bins", 3768);
    declare_parameter<double>("radar.range_res", 0.0596);
    declare_parameter<double>("radar.rotation_rate_hz", 4.0);
    declare_parameter<double>("radar.max_missing_fraction", 0.25);
    declare_parameter<int>("radar.min_filtered_points", 20);
    // first_azimuth|last_azimuth|now|header ("header" uses the frame publisher's
    // header stamp instead of the embedded per-row metadata times -- needed when
    // replaying datasets whose metadata carries dataset-era time).
    declare_parameter<std::string>("radar.stamp_source", "first_azimuth");

    // filtering (radarDriver)
    declare_parameter<double>("driver.z_min", 60.0);
    declare_parameter<int>("driver.k_strongest", 40);
    declare_parameter<double>("driver.min_distance", 2.5);
    declare_parameter<std::string>("driver.filter_type", "kstrong"); // kstrong|CA-CFAR
    // The "peaks" cloud (axial non-max suppression on top of k-strongest) is not
    // used for registration; RadarScan stores it for the pose graph and nothing
    // else reads it. Off by default here: with fuser.store_graph false - the
    // deployed setting - it is measurable cost for no observable output. Turn it
    // back on together with fuser.store_graph.
    declare_parameter<bool>("driver.compute_peaks", false);
    declare_parameter<bool>("driver.publish_filtered", false);
    declare_parameter<std::string>("driver.filtered_topic", "radar_filtered");

    // registration / keyframe fusion (OdometryKeyframeFuser) - CFEAR-3 defaults
    declare_parameter<std::string>("fuser.cost_type", "P2P");
    declare_parameter<std::string>("fuser.loss_type", "Huber");
    declare_parameter<double>("fuser.loss_limit", 0.1);
    declare_parameter<int>("fuser.weight_option", 4);
    declare_parameter<bool>("fuser.weight_intensity", true);
    declare_parameter<double>("fuser.res", 3.0);
    declare_parameter<int>("fuser.submap_scan_size", 4);
    declare_parameter<double>("fuser.registered_min_keyframe_dist", 1.5);
    declare_parameter<double>("fuser.min_keyframe_rot_deg", 5.0);
    declare_parameter<bool>("fuser.use_keyframe", true);
    declare_parameter<bool>("fuser.use_guess", true);
    declare_parameter<bool>("fuser.compensate", true);
    // Motion-compensation (deskew) direction ONLY - it can never mirror the
    // output. Compensate() derives each point's acquisition time from its
    // bearing (GetRelTimeStamp: rel = +-(atan2(y,x)/2pi - 0.5)) and unwinds the
    // previous scan's motion by that fraction, so the sign must match how time
    // maps to bearing: false when bearing grows with time, true when it shrinks.
    // radar.reverse_azimuths flips that mapping, so the two parameters must move
    // together. Not cosmetic: on hg_nav_4 the two settings differ by 0.98 m in
    // position, 5.2 deg in yaw and 4.2 m in path length.
    declare_parameter<bool>("fuser.radar_ccw", false);
    declare_parameter<double>("fuser.covar_scale", 1.0);
    declare_parameter<double>("fuser.regularization", 0.0);
    declare_parameter<bool>("fuser.soft_constraint", false);
    declare_parameter<bool>("fuser.disable_registration", false);
    declare_parameter<double>("fuser.downsample_factor", 1.0);
    declare_parameter<bool>("fuser.store_graph", false);

    // output
    declare_parameter<std::string>("out.odom_topic", "/odometry");
    declare_parameter<std::string>("out.odom_keyframe_topic", "/odometry_keyframe");
    declare_parameter<std::string>("out.registered_cloud_topic", "radar_registered");
    declare_parameter<std::string>("out.registered_keyframe_cloud_topic", "radar_registered_keyframe");
    declare_parameter<bool>("out.publish_registered_cloud", true);
    declare_parameter<bool>("out.publish_tf", true);
    declare_parameter<std::string>("frames.odom_frame", "odom");
    declare_parameter<std::string>("frames.radar_frame", "radar_link");
    // When non-empty, publish odom_frame -> THIS frame instead of
    // odom_frame -> radar_frame, composing the estimate with radar_frame <-
    // tf_body_frame looked up from tf2 (normally the static radar mount
    // extrinsic, published by the driver container).
    //
    // Needed whenever something else already owns radar_frame in the TF tree: on
    // b2w the navtech_radar entrypoint publishes a static lidar -> radar_link, so
    // a second dynamic odom -> radar_link would give radar_link two parents and
    // make every tf2 lookup - rviz, "2D Goal Pose", the nav stack - depend on
    // which message arrived last. Retargeting the child to the robot root keeps
    // one parent per frame and matches the usual odom -> base convention.
    //
    // The odometry TOPIC is unaffected: it still carries the RADAR pose, which is
    // what radarsplat consumes. Empty = upstream behaviour (odom -> radar_frame).
    declare_parameter<std::string>("frames.tf_body_frame", "");

    // Log one machine-readable line per processed scan (stage times, point
    // counts, drop counter) so a replay can be profiled without a debugger.
    // Off by default - it is one INFO line per scan.
    declare_parameter<bool>("debug.log_scan_timing", false);
  }

  RadarConfig ConfigFromParameters() {
    RadarConfig cfg;
    cfg.azimuth_samples = static_cast<uint16_t>(get_parameter("radar.azimuth_samples").as_int());
    cfg.encoder_size = static_cast<uint16_t>(get_parameter("radar.encoder_size").as_int());
    cfg.range_in_bins = static_cast<uint16_t>(get_parameter("radar.range_in_bins").as_int());
    cfg.bin_size = static_cast<float>(get_parameter("radar.range_res").as_double());
    cfg.rotation_rate_hz = get_parameter("radar.rotation_rate_hz").as_double();
    cfg.metadata_columns = static_cast<uint16_t>(get_parameter("radar.metadata_columns").as_int());
    return cfg;
  }

  void ConfigCallback(const navtech_msgs::msg::RadarConfigurationMsg::SharedPtr msg) {
    RadarConfig cfg;
    cfg.azimuth_samples = msg->azimuth_samples;
    cfg.encoder_size = msg->encoder_size;
    cfg.range_in_bins = msg->range_in_bins;
    cfg.bin_size = msg->bin_size > 0.0f
        ? msg->bin_size
        : static_cast<float>(get_parameter("radar.range_res").as_double());
    cfg.rotation_rate_hz = msg->expected_rotation_rate > 0
        ? static_cast<double>(msg->expected_rotation_rate)
        : get_parameter("radar.rotation_rate_hz").as_double();
    // Image-layout fields (leggedrobotics extension; zero from stock drivers).
    cfg.metadata_columns = msg->metadata_columns;
    cfg.start_bin = msg->start_bin;
    cfg.end_bin = msg->end_bin;
    HandleRadarConfig(cfg);
  }

  void HandleRadarConfig(const RadarConfig& cfg) {
    if (!pipeline_ready_) {
      RCLCPP_INFO(get_logger(),
                  "Radar configuration received: %u azimuths, encoder %u, %u bins, bin size %.4f m, %.1f Hz",
                  cfg.azimuth_samples, cfg.encoder_size, cfg.range_in_bins,
                  cfg.bin_size, cfg.rotation_rate_hz);
      InitPipeline(cfg);
    } else if (cfg.azimuth_samples != active_config_.azimuth_samples ||
               cfg.encoder_size != active_config_.encoder_size ||
               cfg.range_in_bins != active_config_.range_in_bins ||
               std::abs(cfg.bin_size - active_config_.bin_size) > 1e-6f) {
      RCLCPP_WARN(get_logger(),
                  "Radar configuration CHANGED after startup (%u az/%u enc/%u bins/%.4f m vs active "
                  "%u az/%u enc/%u bins/%.4f m) - live reconfiguration is not supported, restart the node.",
                  cfg.azimuth_samples, cfg.encoder_size, cfg.range_in_bins, cfg.bin_size,
                  active_config_.azimuth_samples, active_config_.encoder_size,
                  active_config_.range_in_bins, active_config_.bin_size);
    }
  }

  void ConfigTimeout() {
    config_timeout_timer_->cancel();
    if (!pipeline_ready_) {
      const RadarConfig cfg = ConfigFromParameters();
      RCLCPP_WARN(get_logger(),
                  "No RadarConfigurationMsg within timeout - falling back to parameters: "
                  "%u azimuths, encoder %u, %u bins, bin size %.4f m, %.1f Hz",
                  cfg.azimuth_samples, cfg.encoder_size, cfg.range_in_bins,
                  cfg.bin_size, cfg.rotation_rate_hz);
      InitPipeline(cfg);
    }
  }

  void InitPipeline(const RadarConfig& cfg) {
    radarDriver::Parameters driver_par;
    driver_par.range_res = cfg.bin_size;
    driver_par.azimuths = cfg.azimuth_samples;
    driver_par.z_min = static_cast<float>(get_parameter("driver.z_min").as_double());
    driver_par.k_strongest = static_cast<int>(get_parameter("driver.k_strongest").as_int());
    driver_par.min_distance = static_cast<float>(get_parameter("driver.min_distance").as_double());
    driver_par.filter_type_ = CFEAR_Radarodometry::Str2filter(get_parameter("driver.filter_type").as_string());
    driver_par.compute_peaks = get_parameter("driver.compute_peaks").as_bool();
    driver_par.dataset = "oxford"; // assembled scans use the oxford layout: rows=azimuth, cols=range
    driver_par.radar_frameid = get_parameter("frames.radar_frame").as_string();
    driver_par.publish_filtered = get_parameter("driver.publish_filtered").as_bool();
    driver_par.topic_filtered = get_parameter("driver.filtered_topic").as_string();

    OdometryKeyframeFuser::Parameters fuser_par;
    fuser_par.cost_type = get_parameter("fuser.cost_type").as_string();
    fuser_par.loss_type_ = get_parameter("fuser.loss_type").as_string();
    fuser_par.loss_limit_ = get_parameter("fuser.loss_limit").as_double();
    fuser_par.weight_opt = static_cast<CFEAR_Radarodometry::weightoption>(
        get_parameter("fuser.weight_option").as_int());
    fuser_par.weight_intensity_ = get_parameter("fuser.weight_intensity").as_bool();
    fuser_par.res = get_parameter("fuser.res").as_double();
    fuser_par.submap_scan_size = static_cast<int>(get_parameter("fuser.submap_scan_size").as_int());
    fuser_par.min_keyframe_dist_ = get_parameter("fuser.registered_min_keyframe_dist").as_double();
    fuser_par.min_keyframe_rot_deg_ = get_parameter("fuser.min_keyframe_rot_deg").as_double();
    fuser_par.use_keyframe = get_parameter("fuser.use_keyframe").as_bool();
    fuser_par.use_guess = get_parameter("fuser.use_guess").as_bool();
    fuser_par.compensate = get_parameter("fuser.compensate").as_bool();
    fuser_par.radar_ccw = get_parameter("fuser.radar_ccw").as_bool();
    fuser_par.covar_scale_ = get_parameter("fuser.covar_scale").as_double();
    fuser_par.regularization_ = get_parameter("fuser.regularization").as_double();
    fuser_par.soft_constraint = get_parameter("fuser.soft_constraint").as_bool();
    fuser_par.disable_registration = get_parameter("fuser.disable_registration").as_bool();
    fuser_par.store_graph = get_parameter("fuser.store_graph").as_bool();
    fuser_par.rotation_rate_hz = cfg.rotation_rate_hz;
    fuser_par.odom_latest_topic = get_parameter("out.odom_topic").as_string();
    fuser_par.odom_keyframe_topic = get_parameter("out.odom_keyframe_topic").as_string();
    fuser_par.scan_registered_latest_topic = get_parameter("out.registered_cloud_topic").as_string();
    fuser_par.scan_registered_keyframe_topic = get_parameter("out.registered_keyframe_cloud_topic").as_string();
    fuser_par.visualize = get_parameter("out.publish_registered_cloud").as_bool();
    fuser_par.odometry_link_id = get_parameter("frames.odom_frame").as_string();
    fuser_par.child_frame_id = get_parameter("frames.radar_frame").as_string();

    odom_frame_ = fuser_par.odometry_link_id;
    radar_frame_ = fuser_par.child_frame_id;
    tf_body_frame_ = get_parameter("frames.tf_body_frame").as_string();
    const bool publish_tf = get_parameter("out.publish_tf").as_bool();
    // The fuser only knows how to broadcast odom -> radar. When a body frame is
    // requested the node does the broadcasting itself (see PublishBodyTf), so the
    // fuser must stay quiet or radar_frame ends up with two parents anyway.
    fuser_par.publish_tf_ = publish_tf && tf_body_frame_.empty();
    if (publish_tf && !tf_body_frame_.empty()) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
      RCLCPP_INFO(get_logger(), "Broadcasting %s -> %s (estimate composed with %s <- %s from tf2)",
                  odom_frame_.c_str(), tf_body_frame_.c_str(), radar_frame_.c_str(),
                  tf_body_frame_.c_str());
    }
    CFEAR_Radarodometry::MapPointNormal::downsample_factor =
        get_parameter("fuser.downsample_factor").as_double();

    if (fuser_par.store_graph && !driver_par.compute_peaks)
      RCLCPP_WARN(get_logger(),
                  "fuser.store_graph is on but driver.compute_peaks is off - the stored graph's "
                  "per-scan peaks clouds will be EMPTY. Set driver.compute_peaks:=true.");

    RCLCPP_INFO(get_logger(), "radarDriver parameters:\n%s", driver_par.ToString().c_str());
    RCLCPP_INFO(get_logger(), "OdometryKeyframeFuser parameters:\n%s", fuser_par.ToString().c_str());

    driver_ = std::make_unique<radarDriver>(driver_par, this);
    fuser_ = std::make_unique<OdometryKeyframeFuser>(fuser_par, this);

    max_missing_ = static_cast<int>(get_parameter("radar.max_missing_fraction").as_double()
                                    * cfg.azimuth_samples);
    min_filtered_points_ = static_cast<size_t>(get_parameter("radar.min_filtered_points").as_int());
    max_range_bins_ = static_cast<int>(get_parameter("radar.max_range_bins").as_int());
    reverse_azimuths_ = get_parameter("radar.reverse_azimuths").as_bool();
    log_scan_timing_ = get_parameter("debug.log_scan_timing").as_bool();

    // Reversing the rows reverses time-vs-bearing, so the deskew sign must follow.
    if (fuser_par.compensate && fuser_par.radar_ccw != reverse_azimuths_)
      RCLCPP_WARN(get_logger(),
                  "radar.reverse_azimuths=%s with fuser.radar_ccw=%s deskews scans BACKWARDS "
                  "(it adds the motion smear instead of removing it). The two must match.",
                  reverse_azimuths_ ? "true" : "false", fuser_par.radar_ccw ? "true" : "false");
    stamp_source_ = get_parameter("radar.stamp_source").as_string();

    active_config_ = cfg;
    pipeline_ready_ = true;
    RCLCPP_INFO(get_logger(), "Pipeline initialized - publishing odometry on '%s'",
                fuser_par.odom_latest_topic.c_str());
  }

  // Decode the Oxford/Boreas per-row metadata timestamp: bytes 0-7 uint64
  // little-endian UNIX time in microseconds.
  static uint64_t RowTimestampUs(const cv::Mat& img, int row) {
    const uint8_t* p = img.ptr<uint8_t>(row);
    uint64_t us = 0;
    for (int i = 7; i >= 0; i--)
      us = (us << 8) | p[i];
    return us;
  }

  // The frame IS the assembled scan. Strip the embedded per-row metadata
  // columns and decode them for stamps + row validity.
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!pipeline_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Radar frame received but radar configuration is still pending...");
      return;
    }

    const int h = static_cast<int>(msg->height);
    const int w = static_cast<int>(msg->width);
    const int meta = active_config_.metadata_columns;
    if (h <= 0 || w <= meta) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Radar frame %dx%d not wider than %d metadata columns - dropping", h, w, meta);
      return;
    }
    if (active_config_.end_bin > active_config_.start_bin) {
      const int expected = meta + active_config_.end_bin - active_config_.start_bin;
      if (w != expected)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Radar frame width %d != %d announced by the radar configuration "
                             "(metadata %d + bins [%u, %u))",
                             w, expected, meta, active_config_.start_bin, active_config_.end_bin);
    }

    const cv::Mat full(h, w, CV_8UC1, const_cast<uint8_t*>(msg->data.data()),
                       msg->step > 0 ? static_cast<size_t>(msg->step) : static_cast<size_t>(w));

    CompletedScan scan;
    const int avail_bins = w - meta;
    const int use_bins = (max_range_bins_ > 0 && max_range_bins_ < avail_bins)
        ? max_range_bins_ : avail_bins;
    RCLCPP_INFO_ONCE(get_logger(), "Radar frame %dx%d -> using %d of %d range bins (%.2f m at %.4f m/bin)",
                     h, w, use_bins, avail_bins, use_bins * active_config_.bin_size,
                     active_config_.bin_size);
    scan.polar = full.colRange(meta, meta + use_bins).clone();

    rclcpp::Time header_stamp(msg->header.stamp);
    if (header_stamp.nanoseconds() == 0)
      header_stamp = now();

    if (meta >= 11) {
      int first_valid = -1, last_valid = -1, missing = 0;
      for (int r = 0; r < h; r++) {
        if (full.ptr<uint8_t>(r)[10] == 255) {
          if (first_valid < 0)
            first_valid = r;
          last_valid = r;
        } else {
          missing++;
        }
      }
      scan.missing_azimuths = missing;
      if (first_valid >= 0 && stamp_source_ != "header") {
        scan.stamp_first = rclcpp::Time(
            static_cast<int64_t>(RowTimestampUs(full, first_valid)) * 1000, RCL_ROS_TIME);
        scan.stamp_last = rclcpp::Time(
            static_cast<int64_t>(RowTimestampUs(full, last_valid)) * 1000, RCL_ROS_TIME);
      } else {
        scan.stamp_first = header_stamp;
        scan.stamp_last = header_stamp;
      }
    } else {
      // No embedded metadata: synthesize the rotation span from the header
      // stamp and the configured rotation rate.
      scan.missing_azimuths = 0;
      scan.stamp_first = header_stamp;
      const double rate =
          active_config_.rotation_rate_hz > 0 ? active_config_.rotation_rate_hz : 4.0;
      scan.stamp_last =
          header_stamp + rclcpp::Duration::from_seconds((h - 1.0) / (h * rate));
    }

    EnqueueScan(std::move(scan));
  }

  void EnqueueScan(CompletedScan&& completed) {
    if (completed.missing_azimuths > max_missing_) {
      RCLCPP_WARN(get_logger(),
                  "Dropping scan: %d/%u azimuths missing (max allowed %d) - check the frame "
                  "publisher's radar link",
                  completed.missing_azimuths, active_config_.azimuth_samples, max_missing_);
      return;
    }
    if (completed.missing_azimuths > 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Radar frame misses %d azimuths (rows not flagged valid)",
                           completed.missing_azimuths);

    // Hand over to the worker; drop the previous scan if registration is still
    // busy - bounded latency beats an ever-growing queue, and the constant-
    // velocity prediction absorbs a skipped frame.
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (pending_) {
        dropped_scans_++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Registration cannot keep up - dropped %lu scans so far",
                             static_cast<unsigned long>(dropped_scans_.load()));
      }
      pending_ = std::move(completed);
    }
    cv_.notify_one();
  }

  void WorkerLoop() {
    while (true) {
      CompletedScan scan;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return pending_.has_value() || stop_; });
        if (stop_)
          return;
        scan = std::move(*pending_);
        pending_.reset();
      }
      ProcessScan(scan);
    }
  }

  void ProcessScan(CompletedScan& scan) {
    rclcpp::Time stamp = scan.stamp_first;
    if (stamp_source_ == "last_azimuth")
      stamp = scan.stamp_last;
    else if (stamp_source_ == "now")
      stamp = now();

    // Clockwise sensor -> REP-103 bearings. The single seam where every scan
    // passes, and it must happen before ProcessPolarImage: radar_filters.cpp is
    // hardcoded counter-clockwise. See radar.reverse_azimuths. Row order is used
    // for nothing else here - missing_azimuths is a count, and the stamps were
    // read from the metadata columns of the unreversed frame, so both stay right.
    if (reverse_azimuths_)
      cv::flip(scan.polar, scan.polar, 0);

    auto polar = std::make_shared<cv_bridge::CvImage>();
    polar->header.stamp = stamp;
    polar->header.frame_id = radar_frame_;
    polar->encoding = "mono8";
    polar->image = scan.polar;

    // Debug: dump assembled scans for bytewise comparison against source PNGs.
    if (const char* dump_dir = std::getenv("CFEAR_DUMP_DIR"))
      cv::imwrite(std::string(dump_dir) + "/" +
                  std::to_string(rclcpp::Time(scan.stamp_first).nanoseconds() / 1000) + ".png",
                  scan.polar);

    const auto t_start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cloud_peaks;
    driver_->ProcessPolarImage(polar, cloud, cloud_peaks);
    const auto t_filtered = std::chrono::steady_clock::now();

    if (cloud == nullptr || cloud->size() < min_filtered_points_) {
      RCLCPP_WARN(get_logger(), "Scan skipped: only %zu points after filtering (min %zu)",
                  cloud == nullptr ? 0 : cloud->size(), min_filtered_points_);
      return;
    }

    Eigen::Affine3d Tcurr;
    CFEAR_Radarodometry::Covariance cov;
    fuser_->pointcloudCallback(cloud, cloud_peaks, Tcurr, stamp, cov);
    const auto t_fused = std::chrono::steady_clock::now();

    if (log_scan_timing_) {
      const auto ms = [](const std::chrono::steady_clock::duration& d) {
        return std::chrono::duration<double, std::milli>(d).count();
      };
      // One CSV-ish line per scan: parsed by the cfear_speedup analysis to get
      // the per-stage cost distribution and the achievable rate.
      RCLCPP_INFO(get_logger(),
                  "SCANTIME stamp=%.6f cols=%d points=%zu filter_ms=%.3f fuse_ms=%.3f "
                  "total_ms=%.3f dropped=%lu",
                  rclcpp::Time(stamp).seconds(), scan.polar.cols, cloud->size(),
                  ms(t_filtered - t_start), ms(t_fused - t_filtered), ms(t_fused - t_start),
                  static_cast<unsigned long>(dropped_scans_.load()));
    }

    PublishBodyTf(Tcurr, stamp);

    RCLCPP_DEBUG(get_logger(), "pose: x=%.2f y=%.2f", Tcurr.translation().x(),
                 Tcurr.translation().y());
  }

  /* Broadcast odom_frame -> tf_body_frame for the pose the fuser just produced.
   *
   * Tcurr is odom <- radar_frame, so the body pose is Tcurr * (radar_frame <-
   * tf_body_frame). That second factor is the static mount extrinsic; it is
   * looked up once and cached, since re-reading a latched static transform every
   * scan buys nothing. Until it shows up (the driver publishes /tf_static at its
   * own pace) no TF is emitted - a wrong tree is worse than a late one.
   *
   * Note the body pose inherits the radar's height: CFEAR estimates a planar
   * pose AT THE SENSOR, so the body lands one mount height below the odom plane
   * (-0.4525 m on b2w). That is the honest reading of a radar-only estimate, and
   * rviz is happy with it.
   */
  void PublishBodyTf(const Eigen::Affine3d& Tcurr, const rclcpp::Time& stamp) {
    if (tf_broadcaster_ == nullptr)
      return;

    if (!have_radar_from_body_) {
      try {
        T_radar_body_ = tf2::transformToEigen(
            tf_buffer_->lookupTransform(radar_frame_, tf_body_frame_, tf2::TimePointZero));
        have_radar_from_body_ = true;
        RCLCPP_INFO(get_logger(), "Resolved %s <- %s: (%.3f, %.3f, %.3f) m", radar_frame_.c_str(),
                    tf_body_frame_.c_str(), T_radar_body_.translation().x(),
                    T_radar_body_.translation().y(), T_radar_body_.translation().z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No %s <- %s yet, not broadcasting %s -> %s: %s", radar_frame_.c_str(),
                             tf_body_frame_.c_str(), odom_frame_.c_str(), tf_body_frame_.c_str(),
                             ex.what());
        return;
      }
    }

    geometry_msgs::msg::TransformStamped tf =
        tf2::eigenToTransform(Eigen::Isometry3d(Tcurr.matrix()) * T_radar_body_);
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = tf_body_frame_;
    tf_broadcaster_->sendTransform(tf);
  }

  // input
  rclcpp::Subscription<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr config_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr config_timeout_timer_;

  // pipeline
  std::unique_ptr<radarDriver> driver_;
  std::unique_ptr<OdometryKeyframeFuser> fuser_;
  std::atomic<bool> pipeline_ready_{false};
  RadarConfig active_config_;
  int max_missing_ = 100;
  int max_range_bins_ = 0;
  bool reverse_azimuths_ = false;
  bool log_scan_timing_ = false;
  size_t min_filtered_points_ = 20;
  std::string stamp_source_ = "first_azimuth";

  // tf output: odom -> body, composed from the radar pose (see PublishBodyTf).
  // Only allocated when frames.tf_body_frame is set; otherwise the fuser
  // broadcasts odom -> radar itself.
  std::string odom_frame_ = "odom", radar_frame_ = "radar_link", tf_body_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Isometry3d T_radar_body_ = Eigen::Isometry3d::Identity();
  bool have_radar_from_body_ = false;

  // worker thread with a single-slot, drop-oldest queue
  std::thread worker_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::optional<CompletedScan> pending_;
  bool stop_ = false;
  std::atomic<uint64_t> dropped_scans_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CfearOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
