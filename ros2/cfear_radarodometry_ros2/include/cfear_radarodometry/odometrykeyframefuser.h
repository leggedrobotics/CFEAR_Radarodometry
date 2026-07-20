#pragma once

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/common/transforms.h"

#include <fstream>
#include <boost/circular_buffer.hpp>

#include "pcl_conversions/pcl_conversions.h"

#include <time.h>
#include <cstdio>

#include "cfear_radarodometry/utils.h"
#include "cfear_radarodometry/pointnormal.h"
#include "cfear_radarodometry/n_scan_normal.h"
#include "cfear_radarodometry/statistics.h"
#include "boost/shared_ptr.hpp"
#include "cfear_radarodometry/types.h"
#include <opencv2/opencv.hpp>

using std::string;
using std::cout;
using std::cerr;
using std::endl;




namespace CFEAR_Radarodometry {

typedef std::vector<RadarScan> PoseScanVector;

class OdometryKeyframeFuser {

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  class Parameters {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:


    Parameters() {}
    std::string scan_registered_latest_topic = "radar_registered";
    std::string scan_registered_keyframe_topic = "radar_registered_keyframe";
    std::string odom_latest_topic = "/odometry";
    std::string odom_keyframe_topic = "/odometry_keyframe";
    std::string odometry_link_id = "odom";
    std::string child_frame_id = "radar_link"; // ROS1 port hardcoded "radar_link" (tf) / "sensor" (odom msg)
    std::string cost_type = "P2L";
    weightoption weight_opt = weightoption::Uniform;


    bool visualize = true;
    int submap_scan_size = 3;
    bool weight_intensity_ = false;

    bool use_guess = true, disable_registration = false, soft_constraint = false;
    bool compensate = true, radar_ccw = false;
    bool use_keyframe = true, enable_filter = false, use_raw_pointcloud = false;
    double res = 3.5;
    double min_keyframe_dist_ = 1.5, min_keyframe_rot_deg_ = 5;
    std::string loss_type_ = "Huber";
    double loss_limit_ = 0.1;
    double covar_scale_ = 1.0;
    double regularization_ = 0.0;
    double rotation_rate_hz = 4.0; // radar rotation rate, was hardcoded 4 Hz in the ROS1 tree

    bool estimate_cov_by_sampling = false;
    bool cov_samples_to_file_as_well = false; // Will save in the desired folder
    std::string cov_sampling_file_directory = "/tmp/cfear_out";
    double cov_sampling_xy_range = 0.4;  // Will sample from -0.2 to +0.2
    double cov_sampling_yaw_range = 0.0043625; //Will sample from -half to +half of this range as well
    unsigned int cov_sampling_samples_per_axis = 3; //Will do 5^3 in the end
    double cov_sampling_covariance_scaler = 4.0;


    bool publish_tf_ = true;
    bool store_graph = false;

    std::string ToString(){
      std::ostringstream stringStream;
      stringStream << "scan_registered_latest_topic, "<<scan_registered_latest_topic<<endl;
      stringStream << "scan_registered_keyframe_topic, "<<scan_registered_keyframe_topic<<endl;
      stringStream << "odom_latest_topic, "<<odom_latest_topic<<endl;
      stringStream << "odom_keyframe_topic, "<<odom_keyframe_topic<<endl;
      stringStream << "odometry_link_id, "<<odometry_link_id<<endl;
      stringStream << "child_frame_id, "<<child_frame_id<<endl;
      stringStream << "use raw pointcloud, "<<std::boolalpha<<use_raw_pointcloud<<endl;
      stringStream << "submap keyframes, "<<submap_scan_size<<endl;
      stringStream << "resolution r,"<<res<<endl;
      stringStream << "resample factor f, "<<MapPointNormal::downsample_factor<<endl;
      stringStream << "min. sensor distance [m], "<<min_keyframe_dist_<<endl;
      stringStream << "min. sensor rot. [deg], "<<min_keyframe_rot_deg_<<endl;
      stringStream << "use keyframe, "<<std::boolalpha<<use_keyframe<<endl;
      stringStream << "use initial guess, "<<std::boolalpha<<use_guess<<endl;
      stringStream << "radar reversed, "<<std::boolalpha<<radar_ccw<<endl;
      stringStream << "disable registration, "<<std::boolalpha<<disable_registration<<endl;
      stringStream << "soft velocity constraint, "<<std::boolalpha<<soft_constraint<<endl;
      stringStream << "compensate, "<<std::boolalpha<<compensate<<endl;
      stringStream << "rotation rate [hz], "<<rotation_rate_hz<<endl;
      stringStream << "cost type, "<<cost_type<<endl;
      stringStream << "loss type, "<<loss_type_<<endl;
      stringStream << "loss limit, "<<std::to_string(loss_limit_)<<endl;
      stringStream << "covar scale, "<<std::to_string(covar_scale_)<<endl;
      stringStream << "regularization, "<<std::to_string(regularization_)<<endl;
      stringStream << "weight intensity, "<<std::boolalpha<<weight_intensity_<<endl;
      stringStream << "publish_tf, "<<std::boolalpha<<publish_tf_<<endl;
      stringStream << "store graph, "<<store_graph<<endl;
      stringStream << "Weight, "<<weight_opt<<endl;
      stringStream << "Use cost sampling for covariance, "<<std::boolalpha<<estimate_cov_by_sampling<<endl;
      stringStream << "Save cost samples to a file, "<<std::boolalpha<<cov_samples_to_file_as_well<<endl;
      stringStream << "Cost-samples-file folder, "<<cov_sampling_file_directory<<endl;
      stringStream << "XY sampling range, "<<cov_sampling_xy_range<<endl;
      stringStream << "Yaw sampling range, "<<cov_sampling_yaw_range<<endl;
      stringStream << "Cost samples per axis, "<<cov_sampling_samples_per_axis<<endl;
      stringStream << "Sampled covariance scale, "<<cov_sampling_covariance_scaler<<endl;
      return stringStream.str();
    }
  };

  RadarScan scan_;
  bool updated = false;


protected:
  Eigen::Affine3d Tprev_fused, T_prev, Tmot;
  Eigen::Affine3d Tcurrent;
  Covariance cov_current;

  // Components mat publishing
  boost::shared_ptr<n_scan_normal_reg> radar_reg = NULL;
  PoseScanVector keyframes_;
  simple_graph graph_;

  unsigned int frame_nr_ = 0, nr_callbacks_ = 0;
  unsigned int nr_reg_failures_ = 0;
  double distance_traveled = 0.0;
  double Tsensor = 1.0/4.0; // set from par.rotation_rate_hz in the constructor


  Parameters par;
  rclcpp::Node* node_ = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_current_publisher, pose_keyframe_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubsrc_cloud_latest, pub_cloud_keyframe;
  std::unique_ptr<tf2_ros::TransformBroadcaster> Tbr;



public:

  // node may be nullptr: registration only, no publishing (useful for tests).
  OdometryKeyframeFuser(const Parameters& pars, rclcpp::Node* node = nullptr);

  void pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const rclcpp::Time& t);

  void pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const rclcpp::Time& t, Covariance &cov_curr);

  std::string GetStatus(){return "Distance traveled: "+std::to_string(distance_traveled)+", nr sensor readings: "+std::to_string(frame_nr_);}

  void PrintSurface(const std::string& path, const Eigen::MatrixXd& surface);

  void SaveGraph(const std::string& path);

  inline std::pair<RadarScan, std::vector<Constraint3d>> GetLastNode() {
    if (!graph_.empty()) {
      return graph_.back();
    }
    else {
      return std::make_pair(RadarScan(), std::vector<Constraint3d>());
    }
  }

private:

  bool AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev, const Eigen::Affine3d& Tmot_curr);

  bool KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist, double min_keyframe_rot_deg);

  void AddToGraph(PoseScanVector& reference, RadarScan& scan,  const Eigen::Matrix<double,6,6>& Cov);

  Eigen::Affine3d Interpolate(const Eigen::Affine3d &T2, double factor, const Eigen::Affine3d &T1 = Eigen::Affine3d::Identity());

  pcl::PointXYZI Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p);

  nav_msgs::msg::Odometry FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, const rclcpp::Time& t, Matrix6d &Cov);

  pcl::PointCloud<pcl::PointXYZI> FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in, Eigen::Affine3d& T);

  void processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const rclcpp::Time& t);

  bool approximateCovarianceBySampling(std::vector<CFEAR_Radarodometry::MapNormalPtr> &scans_vek, const std::vector<Eigen::Affine3d> &T_vek, Covariance &cov_sampled);





};

void AddToReference(PoseScanVector& reference, RadarScan& scan, size_t submap_scan_size);

void FormatScans(const PoseScanVector& reference,
                 const MapNormalPtr& Pcurrent,
                 const Eigen::Affine3d& Tcurrent,
                 std::vector<Matrix6d>& cov_vek,
                 std::vector<MapNormalPtr>& scans_vek,
                 std::vector<Eigen::Affine3d>& T_vek
                 );

template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in);

}


