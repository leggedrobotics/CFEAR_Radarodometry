cmake_minimum_required(VERSION 3.16)
project(cfear_radarodometry_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3)
# Keep flags portable: this package must build on both x86 and the Jetson (arm64),
# so no -march=native.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(angles REQUIRED)
find_package(navtech_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search filters)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS serialization)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_lib
  src/lib/cfar.cpp
  src/lib/n_scan_normal.cpp
  src/lib/odometrykeyframefuser.cpp
  src/lib/pointnormal.cpp
  src/lib/radar_driver.cpp
  src/lib/radar_filters.cpp
  src/lib/registration.cpp
  src/lib/statistics.cpp
  src/lib/types.cpp
  src/lib/utils.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  Ceres::ceres
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Boost::serialization
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib OpenMP::OpenMP_CXX)
endif()
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_eigen
  pcl_conversions cv_bridge angles
)

add_executable(cfear_odometry_node src/cfear_odometry_node.cpp)
target_link_libraries(cfear_odometry_node ${PROJECT_NAME}_lib)
ament_target_dependencies(cfear_odometry_node
  rclcpp navtech_msgs cv_bridge geometry_msgs tf2 tf2_ros tf2_eigen
)

# Debug/validation: run the pipeline directly on Boreas PNGs (see the file header).
add_executable(offline_png_test src/offline_png_test.cpp)
target_link_libraries(offline_png_test ${PROJECT_NAME}_lib)
ament_target_dependencies(offline_png_test rclcpp cv_bridge)

install(TARGETS cfear_odometry_node offline_png_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
