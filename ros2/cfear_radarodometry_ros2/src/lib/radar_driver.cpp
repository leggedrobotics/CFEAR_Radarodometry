#include "cfear_radarodometry/radar_driver.h"

namespace CFEAR_Radarodometry {



filtertype Str2filter(const std::string& str){
  if (str=="CA-CFAR")
    return filtertype::CACFAR;
  else
    return filtertype::kstrong;

}

std::string Filter2str(const filtertype& filter){
  switch (filter){
  case filtertype::CACFAR: return "CA-CFAR";
  case kstrong: return "kstrong";
  }
  return "kstrong";
}

radarDriver::radarDriver(const Parameters& pars, rclcpp::Node* node):par(pars),node_(node) {

  min_distance_sqrd = par.min_distance*par.min_distance;
  max_distance_sqrd = par.max_distance*par.max_distance;
  if(node_ != nullptr && par.publish_filtered){
    pub_filtered_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(par.topic_filtered, 10);
    pub_peaks_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(par.topic_filtered+"_peaks", 10);
  }
}

void radarDriver::Process(){

  cloud_filtered_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_filtered_peaks_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  if(par.filter_type_ == filtertype::CACFAR) {
    cout<<"window: "<<par.window_size<<", par:false alarm:"<<par.false_alarm_rate<<", par:nb_guard:"<<par.nb_guard_cells<<endl;
    AzimuthCACFAR filter(par.window_size, par.false_alarm_rate, par.nb_guard_cells, par.range_res, par.z_min , par.min_distance, 400.0);
    filter.getFilteredPointCloud(cv_polar_image, cloud_filtered_);
  }
  else{
    StructuredKStrongest filt(cv_polar_image, par.z_min, par.k_strongest, par.min_distance, par.range_res);
    filt.getPeaksFilteredPointCloud(cloud_filtered_, false);
    // The peaks cloud costs a second pass plus axial non-max suppression and
    // feeds nothing but the pose graph - skip it unless asked. cloud_filtered_peaks_
    // stays the empty cloud allocated above, which every consumer handles
    // (Compensate iterates it, RadarScan just stores the pointer).
    if(par.compute_peaks)
      filt.getPeaksFilteredPointCloud(cloud_filtered_peaks_, true);
  }
  //Fill header

  cloud_filtered_peaks_->header.frame_id = cloud_filtered_->header.frame_id = par.radar_frameid;
  const rclcpp::Time tstamp(cv_polar_image->header.stamp);
  pcl_conversions::toPCL(tstamp, cloud_filtered_peaks_->header.stamp);
  pcl_conversions::toPCL(tstamp, cloud_filtered_->header.stamp );

  if(pub_filtered_ != nullptr){
    sensor_msgs::msg::PointCloud2 msg_filtered, msg_peaks;
    pcl::toROSMsg(*cloud_filtered_, msg_filtered);
    pcl::toROSMsg(*cloud_filtered_peaks_, msg_peaks);
    pub_filtered_->publish(msg_filtered);
    pub_peaks_->publish(msg_peaks);
  }
}

/*Assumptions (same as the ROS1 "oxford" path):
 *
 * mono8 / 8UC1
 * Rows azimuth
 * Cols Range, from left (rmin) to right (rmax)
 *
 * */
void radarDriver::ProcessPolarImage(const cv_bridge::CvImagePtr& radar_image_polar,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks){
  if(radar_image_polar == nullptr || radar_image_polar->image.empty()){
    cerr<<"Radar image NULL - scan skipped"<<endl;
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_peaks = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  cv_polar_image = radar_image_polar;
  Process();
  const auto t1 = std::chrono::steady_clock::now();
  CFEAR_Radarodometry::timing.Document("Filtering",CFEAR_Radarodometry::ToMs(t1-t0));

  cloud = cloud_filtered_;
  cloud_peaks = cloud_filtered_peaks_;
}

}
