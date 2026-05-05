// SPDX-License-Identifier: MIT
// ROS 2 port of tree_mapping/src/main_UtilsAndParams.cpp.

#include "tree_mapping_geometry/main_utils_and_params.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace tree_mapping_geometry
{
namespace utils
{

std::string create_file_name(
  const std::string & path,
  const std::string & prefix,
  const std::string & file_num,
  int zeros_width,
  const std::string & append,
  const std::string & extension)
{
  std::stringstream ss;
  ss << path << prefix;
  ss.fill('0');
  ss.width(zeros_width);
  ss << std::stoi(file_num) << append << extension;
  return ss.str();
}

namespace
{

template <typename PtA, typename PtB>
float pt_distance(PtA pt1, PtB pt2)
{
  if (std::isnan(pt1.x)) {pt1.x = std::numeric_limits<float>::infinity();}
  if (std::isnan(pt1.y)) {pt1.y = std::numeric_limits<float>::infinity();}
  if (std::isnan(pt1.z)) {pt1.z = std::numeric_limits<float>::infinity();}
  if (std::isnan(pt2.x)) {pt2.x = std::numeric_limits<float>::infinity();}
  if (std::isnan(pt2.y)) {pt2.y = std::numeric_limits<float>::infinity();}
  if (std::isnan(pt2.z)) {pt2.z = std::numeric_limits<float>::infinity();}
  const float dx = pt1.x - pt2.x;
  const float dy = pt1.y - pt2.y;
  const float dz = pt1.z - pt2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

float distance(velodyne_pointcloud::PointXYZIR pt1, velodyne_pointcloud::PointXYZIR pt2)
{
  return pt_distance(pt1, pt2);
}

float distance(pcl::PointXYZ pt1, velodyne_pointcloud::PointXYZIR pt2)
{
  return pt_distance(pt1, pt2);
}

float distance(pcl::PointWithRange pt1, pcl::PointWithRange pt2)
{
  return pt_distance(pt1, pt2);
}

bool is_nan_or_inf(float range)
{
  return std::isnan(range) || std::isinf(range) ||
         (std::abs(std::numeric_limits<float>::max() - range) <= 0.1f);
}

bool is_nan_or_inf(double range)
{
  return std::isnan(range) || std::isinf(range) ||
         (std::abs(std::numeric_limits<double>::max() - range) <= 0.1);
}

double rad2deg(double number)
{
  return (180.0 * number) / M_PI;
}

double deg2rad(double number)
{
  return (M_PI * number) / 180.0;
}

}  // namespace utils
}  // namespace tree_mapping_geometry
