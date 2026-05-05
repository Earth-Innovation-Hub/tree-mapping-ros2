// SPDX-License-Identifier: MIT
// ROS 2 port of tree_mapping/src/main_UtilsAndParams.h.

#ifndef TREE_MAPPING_GEOMETRY__MAIN_UTILS_AND_PARAMS_HPP_
#define TREE_MAPPING_GEOMETRY__MAIN_UTILS_AND_PARAMS_HPP_

#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/filters/passthrough.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "tree_mapping_geometry/pcd_custom_types.hpp"

namespace tree_mapping_geometry
{

using PointType = velodyne_pointcloud::PointXYZIR;

namespace utils
{

std::string create_file_name(
  const std::string & path,
  const std::string & prefix,
  const std::string & file_num,
  int zeros_width,
  const std::string & append,
  const std::string & extension);

float distance(velodyne_pointcloud::PointXYZIR pt1, velodyne_pointcloud::PointXYZIR pt2);
float distance(pcl::PointXYZ pt1, velodyne_pointcloud::PointXYZIR pt2);
float distance(pcl::PointWithRange pt1, pcl::PointWithRange pt2);

bool is_nan_or_inf(float range);
bool is_nan_or_inf(double range);

double rad2deg(double number);
double deg2rad(double number);

}  // namespace utils
}  // namespace tree_mapping_geometry

#endif  // TREE_MAPPING_GEOMETRY__MAIN_UTILS_AND_PARAMS_HPP_
