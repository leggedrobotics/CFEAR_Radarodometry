#pragma once

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "cfear_radarodometry/radar_filters.h"
#include "cfear_radarodometry/statistics.h"
#include "cfear_radarodometry/cfar.h"
#include "cfear_radarodometry/pointnormal.h"


namespace CFEAR_Radarodometry  {
using std::cout;
using std::endl;
using std::cerr;

typedef enum filter_type{kstrong, CACFAR}filtertype;

std::string Filter2str(const filtertype& filter);

filtertype Str2filter(const std::string& filter);



/* ROS2 port: the driver no longer subscribes to an image topic itself. The node
 * assembles polar scans (from per-azimuth FFT messages) into a cv_bridge::CvImage
 * and pushes it through ProcessPolarImage() - the same seam the ROS1 offline
 * pipeline used via CallbackOffline(). Layout expectation is the "oxford" one:
 * mono8, rows = azimuths, cols = range bins. */
class radarDriver
{
public:
  class Parameters
  {
  public:
    Parameters() {}

    float z_min = 60; // min power
    float range_res = 0.0438;
    int azimuths = 400, k_strongest = 12;
    int nb_guard_cells = 20, window_size = 10;
    float false_alarm_rate = 0.01;
    float min_distance = 2.5, max_distance = 200;
    std::string radar_frameid = "sensor_est", topic_filtered = "/Navtech/Filtered";
    std::string dataset = "oxford";
    filtertype filter_type_ = filtertype::kstrong;
    bool publish_filtered = false; // debug: publish the filtered clouds as PointCloud2
    // Also run the axial non-maximum suppression pass that produces the "peaks"
    // cloud. That cloud is NOT used for registration - the only consumer is
    // RadarScan::cloud_peaks_, i.e. the serialized pose graph - so with
    // fuser.store_graph off it is pure cost: a second k-strongest extraction
    // plus, per azimuth, an unordered_map of neighbour scores and a 7-tap sum
    // per masked bin. Default true keeps the upstream behaviour for the offline
    // tools; the online node turns it off (see driver.compute_peaks).
    bool compute_peaks = true;

    std::string ToString(){
      std::ostringstream stringStream;
      stringStream << "range res, "<<range_res<<endl;
      stringStream << "z min, "<<z_min<<endl;
      stringStream << "min distance, "<<min_distance<<endl;
      stringStream << "max distance, "<<max_distance<<endl;
      stringStream << "k strongest, "<<k_strongest<<endl;
      stringStream << "topic_filtered, "<<topic_filtered<<endl;
      stringStream << "radar_frameid, "<<radar_frameid<<endl;
      stringStream << "dataset, "<<dataset<<endl;
      stringStream << "filter type, "<<Filter2str(filter_type_)<<endl;
      stringStream << "compute peaks, "<<(compute_peaks ? "true" : "false")<<endl;
      stringStream << "nb guard cells, "<<nb_guard_cells<<endl;
      stringStream << "window size, "<<window_size<<endl;
      stringStream << "false alarm rate, "<<false_alarm_rate<<endl;

      return stringStream.str();
    }

  };

  // node may be nullptr (no debug publishing).
  radarDriver(const Parameters& pars, rclcpp::Node* node = nullptr);

  ~radarDriver(){}

  // The online seam: filter an assembled polar scan into the two point clouds.
  void ProcessPolarImage(const cv_bridge::CvImagePtr& radar_image_polar,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks);

  cv_bridge::CvImagePtr cv_polar_image; //Latest radar image


private:

  void Process();

  Parameters par;
  float max_distance_sqrd, min_distance_sqrd;

  rclcpp::Node* node_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_, pub_peaks_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_, cloud_filtered_peaks_;

};

}
