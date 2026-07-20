/* Online CFEAR radar odometry node (ROS2 Jazzy).
 *
 * Pipeline:
 *   navtech_msgs/RadarFftDataMsg (one azimuth per message, e.g. /radar_data/fft)
 *     -> FftRotationAssembler (full polar rotation, mono8 rows=azimuths)
 *     -> radarDriver (k-strongest filtering -> point clouds)
 *     -> OdometryKeyframeFuser (CFEAR registration -> nav_msgs/Odometry on /odometry [+ tf])
 *
 * Radar geometry (azimuth samples, encoder size, range bins, bin size) is taken
 * from the driver's RadarConfigurationMsg when available; after
 * radar.config_timeout_sec it falls back to the radar.* parameters.
 */
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navtech_msgs/msg/radar_configuration_msg.hpp>
#include <navtech_msgs/msg/radar_fft_data_msg.hpp>

#include "cfear_radarodometry/fft_assembler.h"
#include "cfear_radarodometry/odometrykeyframefuser.h"
#include "cfear_radarodometry/radar_driver.h"

using CFEAR_Radarodometry::CompletedScan;
using CFEAR_Radarodometry::FftRotationAssembler;
using CFEAR_Radarodometry::OdometryKeyframeFuser;
using CFEAR_Radarodometry::RadarConfig;
using CFEAR_Radarodometry::radarDriver;

class CfearOdometryNode : public rclcpp::Node {
public:
  CfearOdometryNode() : rclcpp::Node("cfear_odometry_node") {
    DeclareParameters();

    const std::string fft_topic = get_parameter("radar.fft_topic").as_string();
    const std::string config_topic = get_parameter("radar.config_topic").as_string();

    // FFT azimuths arrive at ~azimuth_samples * rotation_rate msg/s (e.g. 1600/s);
    // the depth buffers roughly two rotations of executor jitter.
    rclcpp::QoS fft_qos(rclcpp::KeepLast(800));
    if (get_parameter("radar.fft_reliability").as_string() == "reliable")
      fft_qos.reliable();
    else
      fft_qos.best_effort();

    rclcpp::QoS config_qos(rclcpp::KeepLast(1));
    config_qos.reliable();
    if (get_parameter("radar.config_durability").as_string() == "transient_local")
      config_qos.transient_local();

    fft_sub_ = create_subscription<navtech_msgs::msg::RadarFftDataMsg>(
        fft_topic, fft_qos,
        std::bind(&CfearOdometryNode::FftCallback, this, std::placeholders::_1));
    config_sub_ = create_subscription<navtech_msgs::msg::RadarConfigurationMsg>(
        config_topic, config_qos,
        std::bind(&CfearOdometryNode::ConfigCallback, this, std::placeholders::_1));

    const double timeout = get_parameter("radar.config_timeout_sec").as_double();
    config_timeout_timer_ = create_wall_timer(
        std::chrono::duration<double>(timeout),
        std::bind(&CfearOdometryNode::ConfigTimeout, this));

    worker_ = std::thread(&CfearOdometryNode::WorkerLoop, this);

    RCLCPP_INFO(get_logger(), "Listening for FFT data on '%s' (%s), radar configuration on '%s'",
                fft_topic.c_str(),
                get_parameter("radar.fft_reliability").as_string().c_str(),
                config_topic.c_str());
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
    declare_parameter<std::string>("radar.fft_topic", "/radar_data/fft");
    declare_parameter<std::string>("radar.config_topic", "/radar_data/configuration_data");
    declare_parameter<std::string>("radar.fft_reliability", "best_effort"); // or "reliable"
    declare_parameter<std::string>("radar.config_durability", "transient_local"); // or "volatile"
    declare_parameter<double>("radar.config_timeout_sec", 5.0);
    declare_parameter<int>("radar.azimuth_samples", 400);
    declare_parameter<int>("radar.encoder_size", 5600);
    declare_parameter<int>("radar.range_in_bins", 3768);
    declare_parameter<double>("radar.range_res", 0.0596);
    declare_parameter<double>("radar.rotation_rate_hz", 4.0);
    declare_parameter<double>("radar.max_missing_fraction", 0.25);
    declare_parameter<int>("radar.min_filtered_points", 20);
    declare_parameter<std::string>("radar.stamp_source", "first_azimuth"); // first_azimuth|last_azimuth|now

    // filtering (radarDriver)
    declare_parameter<double>("driver.z_min", 60.0);
    declare_parameter<int>("driver.k_strongest", 40);
    declare_parameter<double>("driver.min_distance", 2.5);
    declare_parameter<std::string>("driver.filter_type", "kstrong"); // kstrong|CA-CFAR
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
  }

  RadarConfig ConfigFromParameters() {
    RadarConfig cfg;
    cfg.azimuth_samples = static_cast<uint16_t>(get_parameter("radar.azimuth_samples").as_int());
    cfg.encoder_size = static_cast<uint16_t>(get_parameter("radar.encoder_size").as_int());
    cfg.range_in_bins = static_cast<uint16_t>(get_parameter("radar.range_in_bins").as_int());
    cfg.bin_size = static_cast<float>(get_parameter("radar.range_res").as_double());
    cfg.rotation_rate_hz = get_parameter("radar.rotation_rate_hz").as_double();
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
    fuser_par.publish_tf_ = get_parameter("out.publish_tf").as_bool();
    fuser_par.odometry_link_id = get_parameter("frames.odom_frame").as_string();
    fuser_par.child_frame_id = get_parameter("frames.radar_frame").as_string();
    CFEAR_Radarodometry::MapPointNormal::downsample_factor =
        get_parameter("fuser.downsample_factor").as_double();

    RCLCPP_INFO(get_logger(), "radarDriver parameters:\n%s", driver_par.ToString().c_str());
    RCLCPP_INFO(get_logger(), "OdometryKeyframeFuser parameters:\n%s", fuser_par.ToString().c_str());

    driver_ = std::make_unique<radarDriver>(driver_par, this);
    fuser_ = std::make_unique<OdometryKeyframeFuser>(fuser_par, this);

    max_missing_ = static_cast<int>(get_parameter("radar.max_missing_fraction").as_double()
                                    * cfg.azimuth_samples);
    min_filtered_points_ = static_cast<size_t>(get_parameter("radar.min_filtered_points").as_int());
    stamp_source_ = get_parameter("radar.stamp_source").as_string();

    assembler_.SetConfig(cfg);
    active_config_ = cfg;
    pipeline_ready_ = true;
    RCLCPP_INFO(get_logger(), "Pipeline initialized - publishing odometry on '%s'",
                fuser_par.odom_latest_topic.c_str());
  }

  void FftCallback(const navtech_msgs::msg::RadarFftDataMsg::SharedPtr msg) {
    if (!pipeline_ready_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "FFT data received but radar configuration is still pending...");
      return;
    }

    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0)
      stamp = now(); // driver did not stamp the message

    auto completed = assembler_.AddAzimuth(msg->azimuth, msg->sweep_counter,
                                           msg->data.data(), msg->data.size(), stamp);
    if (!completed)
      return;

    if (completed->missing_azimuths > max_missing_) {
      RCLCPP_WARN(get_logger(),
                  "Dropping scan: %d/%u azimuths missing (max allowed %d) - check FFT topic QoS/bandwidth",
                  completed->missing_azimuths, active_config_.azimuth_samples, max_missing_);
      return;
    }
    if (completed->missing_azimuths > 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Assembled scan misses %d azimuths (dropped FFT messages)",
                           completed->missing_azimuths);

    // Hand over to the worker; drop the previous scan if registration is still
    // busy - bounded latency beats an ever-growing queue, and the constant-
    // velocity prediction absorbs a skipped frame.
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (pending_) {
        dropped_scans_++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Registration cannot keep up - dropped %lu scans so far",
                             static_cast<unsigned long>(dropped_scans_));
      }
      pending_ = std::move(*completed);
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

    auto polar = std::make_shared<cv_bridge::CvImage>();
    polar->header.stamp = stamp;
    polar->header.frame_id = get_parameter("frames.radar_frame").as_string();
    polar->encoding = "mono8";
    polar->image = scan.polar;

    // Debug: dump assembled scans for bytewise comparison against source PNGs.
    if (const char* dump_dir = std::getenv("CFEAR_DUMP_DIR"))
      cv::imwrite(std::string(dump_dir) + "/" +
                  std::to_string(rclcpp::Time(scan.stamp_first).nanoseconds() / 1000) + ".png",
                  scan.polar);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cloud_peaks;
    driver_->ProcessPolarImage(polar, cloud, cloud_peaks);

    if (cloud == nullptr || cloud->size() < min_filtered_points_) {
      RCLCPP_WARN(get_logger(), "Scan skipped: only %zu points after filtering (min %zu)",
                  cloud == nullptr ? 0 : cloud->size(), min_filtered_points_);
      return;
    }

    Eigen::Affine3d Tcurr;
    CFEAR_Radarodometry::Covariance cov;
    fuser_->pointcloudCallback(cloud, cloud_peaks, Tcurr, stamp, cov);

    RCLCPP_DEBUG(get_logger(), "pose: x=%.2f y=%.2f", Tcurr.translation().x(),
                 Tcurr.translation().y());
  }

  // input
  rclcpp::Subscription<navtech_msgs::msg::RadarFftDataMsg>::SharedPtr fft_sub_;
  rclcpp::Subscription<navtech_msgs::msg::RadarConfigurationMsg>::SharedPtr config_sub_;
  rclcpp::TimerBase::SharedPtr config_timeout_timer_;

  // pipeline
  FftRotationAssembler assembler_;
  std::unique_ptr<radarDriver> driver_;
  std::unique_ptr<OdometryKeyframeFuser> fuser_;
  std::atomic<bool> pipeline_ready_{false};
  RadarConfig active_config_;
  int max_missing_ = 100;
  size_t min_filtered_points_ = 20;
  std::string stamp_source_ = "first_azimuth";

  // worker thread with a single-slot, drop-oldest queue
  std::thread worker_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::optional<CompletedScan> pending_;
  bool stop_ = false;
  uint64_t dropped_scans_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CfearOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
