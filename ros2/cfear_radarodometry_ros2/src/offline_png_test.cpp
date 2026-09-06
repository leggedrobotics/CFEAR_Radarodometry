/* Debug/validation tool: run the ported CFEAR pipeline directly on Boreas
 * radar PNGs (bypassing DDS entirely) and write a TUM
 * trajectory. Used to verify the ROS2 port reproduces the ROS1 offline result.
 *
 *   ros2 run cfear_radarodometry_ros2 offline_png_test <radar_dir> <out_tum> [n_scans] [ccw]
 */
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

#include "cfear_radarodometry/odometrykeyframefuser.h"
#include "cfear_radarodometry/radar_driver.h"

using namespace CFEAR_Radarodometry;

static constexpr int kMetadataCols = 11;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: offline_png_test <radar_dir> <out_tum> [n_scans] [ccw=0/1]\n";
    return 1;
  }
  const std::string radar_dir = argv[1];
  const std::string out_path = argv[2];
  const size_t n_max = argc > 3 ? std::stoul(argv[3]) : SIZE_MAX;
  const bool ccw = argc > 4 && std::string(argv[4]) == "1";

  std::vector<std::string> files;
  for (const auto& e : std::filesystem::directory_iterator(radar_dir))
    if (e.path().extension() == ".png")
      files.push_back(e.path().string());
  std::sort(files.begin(), files.end());
  if (files.size() > n_max)
    files.resize(n_max);
  std::cout << "processing " << files.size() << " scans, ccw=" << ccw << std::endl;

  radarDriver::Parameters dpar;
  dpar.range_res = 0.0596f;
  dpar.z_min = 60;
  dpar.k_strongest = 40;
  dpar.min_distance = 2.5;
  dpar.dataset = "oxford";
  radarDriver driver(dpar, nullptr);

  OdometryKeyframeFuser::Parameters fpar;
  fpar.cost_type = "P2P";
  fpar.loss_type_ = "Huber";
  fpar.loss_limit_ = 0.1;
  fpar.weight_opt = static_cast<weightoption>(4);
  fpar.weight_intensity_ = true;
  fpar.res = 3.0;
  fpar.submap_scan_size = 4;
  fpar.min_keyframe_dist_ = 1.5;
  fpar.min_keyframe_rot_deg_ = 5;
  fpar.compensate = true;
  fpar.radar_ccw = ccw;
  OdometryKeyframeFuser fuser(fpar, nullptr);

  std::ofstream out(out_path);
  out << std::fixed;
  for (size_t i = 0; i < files.size(); i++) {
    const std::string base = std::filesystem::path(files[i]).stem().string();
    const uint64_t t_us = std::stoull(base);
    const rclcpp::Time stamp(static_cast<int64_t>(t_us) * 1000);

    cv::Mat png = cv::imread(files[i], cv::IMREAD_GRAYSCALE);
    if (png.empty())
      continue;
    auto cvimg = std::make_shared<cv_bridge::CvImage>();
    cvimg->header.stamp = stamp;
    cvimg->encoding = "mono8";
    cvimg->image = png.colRange(kMetadataCols, png.cols).clone();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, peaks;
    driver.ProcessPolarImage(cvimg, cloud, peaks);
    if (cloud == nullptr || cloud->empty())
      continue;

    Eigen::Affine3d T;
    Covariance cov;
    fuser.pointcloudCallback(cloud, peaks, T, stamp, cov);

    const Eigen::Quaterniond q(T.linear());
    out << t_us / 1e6 << " " << T.translation().x() << " " << T.translation().y()
        << " 0 " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    if ((i + 1) % 200 == 0)
      std::cout << "  " << i + 1 << "/" << files.size() << std::endl;
  }
  std::cout << "wrote " << out_path << std::endl;
  return 0;
}
