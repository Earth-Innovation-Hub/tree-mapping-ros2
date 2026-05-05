// SPDX-License-Identifier: MIT
// Originally:
//   tree_mapping/src/pcd_custom_types.h (DREAMS-lab/tree-mapping)
//   Adapted from velodyne_pointcloud point types (Austin Robot Technology).
//
// PCL custom point type used throughout the tree-mapping pipeline:
// XYZ + intensity + ring index.  The pcl::PointCloud<> instantiations and
// conversions in cylinderFitting / branchInstanceSegmentation depend on
// this exact field layout, so it is vendored here verbatim.

#ifndef TREE_MAPPING_GEOMETRY__PCD_CUSTOM_TYPES_HPP_
#define TREE_MAPPING_GEOMETRY__PCD_CUSTOM_TYPES_HPP_

#include <cstdint>
#include <pcl/point_types.h>


namespace velodyne_pointcloud
{

struct EIGEN_ALIGN16 PointXYZIR
{
  PCL_ADD_POINT4D;                  // quad-word XYZ
  float    intensity;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring))

#endif  // TREE_MAPPING_GEOMETRY__PCD_CUSTOM_TYPES_HPP_
