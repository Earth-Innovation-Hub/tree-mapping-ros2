// SPDX-License-Identifier: MIT
// ROS 2 port of tree_mapping/src/cylinderFitting_Data.h.
//
// Provides the Circle / Cylinder data classes used to build synthetic
// test cylinders (manualCylinder()) for the cylinder-fitting residual
// evaluator.  No ROS or PCL dependencies here, only <cmath> + STL.

#ifndef TREE_MAPPING_GEOMETRY__CYLINDER_FITTING_DATA_HPP_
#define TREE_MAPPING_GEOMETRY__CYLINDER_FITTING_DATA_HPP_

#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace tree_mapping_geometry
{

class Circle
{
public:
  std::vector<double> calc_angular_point(double r, double theta, double phi) const
  {
    std::vector<double> point(3);
    point[0] = center[0] + r * std::cos(phi) * std::sin(theta);
    point[1] = center[1] + r * std::sin(phi) * std::sin(theta);
    point[2] = center[2] + r * std::cos(theta);
    return point;
  }

  void set_center(std::vector<double> c)
  {
    std::cout << "set center: " << c[0] << ", " << c[1] << ", " << c[2] << '\n';
    center = std::move(c);
  }

  void append_point(std::vector<double> point)
  {
    num_points += 1;
    points.push_back(std::move(point));
  }

  void print_points() const
  {
    std::cout << "Circle Points\n";
    for (const auto & pt : points) {
      std::cout << pt[0] << ", " << pt[1] << ", " << pt[2] << '\n';
    }
  }

  std::vector<std::vector<double>> get_points() const
  {
    return points;
  }

  void print_center() const
  {
    std::cout << "Circle Center\n"
              << center[0] << ", " << center[1] << ", " << center[2] << '\n';
  }

  std::vector<std::vector<double>> points;
  std::vector<double> center;
  int num_points = 0;
};


class Cylinder
{
public:
  void append_ring(int /*ring_num*/, const Circle & circle)
  {
    num_rings += 1;
    num_points += circle.num_points;
    rings.push_back(circle);
  }

  void print_points() const
  {
    std::cout << "CYLINDER POINTS\n";
    for (std::size_t i = 0; i < rings.size(); ++i) {
      std::cout << "ring: " << i << '\n';
      rings[i].print_points();
    }
  }

  std::vector<std::vector<std::vector<double>>> get_points() const
  {
    std::vector<std::vector<std::vector<double>>> out;
    out.reserve(rings.size());
    for (const auto & ring : rings) {
      out.push_back(ring.get_points());
    }
    return out;
  }

  std::vector<Circle> rings;
  int num_rings = 0;
  int num_points = 0;
};


inline void create_cylinder(
  const std::vector<std::vector<double>> & centers,
  double radius,
  double phi_start,
  double phi_resolution,
  double phi_end,
  Cylinder & cylinder)
{
  // Disks of cylinder are parallel to the XY plane.
  const double theta = M_PI_2;

  for (std::size_t i = 0; i < centers.size(); ++i) {
    Circle circle;
    circle.set_center(centers[i]);
    for (double phi = phi_start; phi <= phi_end; phi += phi_resolution) {
      circle.append_point(circle.calc_angular_point(radius, theta, phi));
    }
    std::cout << "circle_size: " << circle.num_points << '\n';
    cylinder.append_ring(static_cast<int>(i), circle);
  }
}


inline Cylinder manual_cylinder(const std::string & file_num = "0", bool automatic_angles = false)
{
  std::cout << "MANUAL CYLINDER\n";

  std::vector<std::vector<double>> centers;
  const int f = std::stoi(file_num);
  switch (f) {
    case 0: centers = {{0.0,  3.0, 0.0}, {0.0,  3.0, 1.0}, {0.0,  3.0, 2.0}}; break;
    case 1: centers = {{-3.0, 0.0, 0.0}, {-3.0, 0.0, 1.0}, {-3.0, 0.0, 2.0}}; break;
    case 2: centers = {{0.0, -3.0, 0.0}, {0.0, -3.0, 1.0}, {0.0, -3.0, 2.0}}; break;
    case 3: centers = {{3.0,  0.0, 0.0}, {3.0,  0.0, 1.0}, {3.0,  0.0, 2.0}}; break;
    case 4: centers = {{3.0,  3.0, 0.0}, {3.0,  3.0, 1.0}, {3.0,  3.0, 2.0}}; break;
    case 5: centers = {{-3.0, 3.0, 0.0}, {-3.0, 3.0, 1.0}, {-3.0, 3.0, 2.0}}; break;
    case 6: centers = {{-3.0,-3.0, 0.0}, {-3.0,-3.0, 1.0}, {-3.0,-3.0, 2.0}}; break;
    case 7: centers = {{3.0, -3.0, 0.0}, {3.0, -3.0, 1.0}, {3.0, -3.0, 2.0}}; break;
    case 99: centers = {{0.5, 0.5, 0.0}, {0.5, 0.5, 1.0}, {0.5, 0.5, 2.0}}; break;
    default:
      std::cerr << "NO MATCHING CYLINDER IDX (file_num=" << file_num << ")\n";
      std::exit(1);
  }

  const double radius = 1.0;
  const double phi_resolution = M_PI_4;
  double phi_start;
  double phi_end;

  if (automatic_angles) {
    // Centers are vertically stacked at the same (x,y); pick the half-circle
    // facing the origin.
    const double tmp_phi = std::atan2(centers[0][1], centers[0][0]);
    const double phi_center = tmp_phi + M_PI;
    std::cout << "PHI_CENTER: " << phi_center << '\n';
    phi_start = phi_center - M_PI_2;
    phi_end   = phi_center + M_PI_2;
  } else {
    phi_start = M_PI_2;
    phi_end   = 3 * M_PI_2;
  }

  Cylinder cylinder;
  create_cylinder(centers, radius, phi_start, phi_resolution, phi_end, cylinder);
  cylinder.print_points();
  return cylinder;
}

}  // namespace tree_mapping_geometry

#endif  // TREE_MAPPING_GEOMETRY__CYLINDER_FITTING_DATA_HPP_
