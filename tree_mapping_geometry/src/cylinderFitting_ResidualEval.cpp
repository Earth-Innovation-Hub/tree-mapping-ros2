// SPDX-License-Identifier: MIT
// ROS 2 port of tree_mapping/src/cylinderFitting_ResidualEval.cpp.
//
// Stand-alone Ceres residual *evaluator* for the cylinder-fitting cost
// function used in the DREAMS-lab tree mapping pipeline.  This is the
// regression / sanity-check binary that, given fixed parameter values
// (rho, kappa, theta, phi, alpha) and a synthetic cylinder index, prints
// the per-point and total cost.  Useful for verifying the residual math
// after porting before wiring the full Ceres optimizer in
// cylinderFitting.cpp.
//
// Cylinder parameterization (eqn 23 from the cylinders paper / eqn 4 from
// SLOAM):
//   rho   distance from origin to cylinder axis
//   kappa 1/r curvature of the cylinder cross-section
//   theta polar angle of the unit vector pointing to the closest point
//         on the axis
//   phi   azimuth of that vector
//   alpha rotation around that vector defining the axis direction

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ceres/ceres.h"
#include "ceres/jet.h"

#include "tree_mapping_geometry/cylinder_fitting_data.hpp"

namespace tree_mapping_geometry
{

struct CylinderResidual
{
  CylinderResidual(double x, double y, double z)
  : x_(x), y_(y), z_(z) {}

  static Eigen::Vector3d n_reconstr(double theta, double phi)
  {
    return {std::cos(phi) * std::sin(theta),
            std::sin(phi) * std::sin(theta),
            std::cos(theta)};
  }

  static Eigen::Vector3d n_theta(double theta, double phi)
  {
    return {+std::cos(phi) * std::cos(theta),
            +std::sin(phi) * std::cos(theta),
            -std::sin(theta)};
  }

  static Eigen::Vector3d n_phi_bar(double /*theta*/, double phi)
  {
    return {-std::sin(phi), +std::cos(phi), 0.0};
  }

  static Eigen::Vector3d a_reconstr(
    const Eigen::Vector3d & n_theta_vec,
    const Eigen::Vector3d & n_phi_bar_vec,
    double alpha)
  {
    return n_theta_vec * std::cos(alpha) + n_phi_bar_vec * std::sin(alpha);
  }

  // Cylinder distance residual.  Implements the squared-form of eqn 23
  // (no ceres::sqrt) plus a parallel "eqn22"-style residual for sanity
  // comparison.
  template <typename T>
  bool operator()(
    const T * const rho,
    const T * const kappa,
    const T * const theta,
    const T * const phi,
    const T * const alpha,
    T * residual,
    T * residual_vec,
    T * residual_eqn22) const
  {
    std::cout << "PARAMS RECEIVED in cost functor: "
              << rho[0] << ", " << kappa[0] << ", " << theta[0] << ", "
              << phi[0] << ", " << alpha[0] << '\n';

    residual[0] = (kappa[0] / 2.0) * (
      x_ * x_ + y_ * y_ + z_ * z_
      - 2.0 * rho[0] * (
          x_ * ceres::cos(phi[0]) * ceres::sin(theta[0])
        + y_ * ceres::sin(phi[0]) * ceres::sin(theta[0])
        + z_ * ceres::cos(theta[0]))
      - (
          x_ * (ceres::cos(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
                - ceres::sin(phi[0]) * ceres::sin(alpha[0]))
        + y_ * (ceres::sin(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
                + ceres::cos(phi[0]) * ceres::sin(alpha[0]))
        + z_ * (-ceres::sin(theta[0]) * ceres::cos(alpha[0]))
        )
        * (
          x_ * (ceres::cos(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
                - ceres::sin(phi[0]) * ceres::sin(alpha[0]))
        + y_ * (ceres::sin(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
                + ceres::cos(phi[0]) * ceres::sin(alpha[0]))
        + z_ * (-ceres::sin(theta[0]) * ceres::cos(alpha[0]))
        )
      + rho[0] * rho[0])
      + rho[0]
      - (
          x_ * ceres::cos(phi[0]) * ceres::sin(theta[0])
        + y_ * ceres::sin(phi[0]) * ceres::sin(theta[0])
        + z_ * ceres::cos(theta[0]));

    // Reference vector form of the same residual (sanity check).
    Eigen::Vector3d p_vec(x_, y_, z_);
    Eigen::Vector3d n_vec        = n_reconstr(theta[0], phi[0]);
    Eigen::Vector3d n_theta_vec  = n_theta(theta[0], phi[0]);
    Eigen::Vector3d n_phi_bar_v  = n_phi_bar(theta[0], phi[0]);
    Eigen::Vector3d a_vec        = a_reconstr(n_theta_vec, n_phi_bar_v, alpha[0]);

    residual_vec[0] = kappa[0] / 2.0 * (
        p_vec.norm() * p_vec.norm()
      - 2.0 * rho[0] * (p_vec.dot(n_vec))
      - p_vec.dot(a_vec) * p_vec.dot(a_vec)
      + rho[0] * rho[0])
      + rho[0]
      - p_vec.dot(n_vec);

    Eigen::Vector3d diff_vec = p_vec - (rho[0] + 1.0 / kappa[0]) * n_vec;
    Eigen::Vector3d cross_pdt = diff_vec.cross(a_vec);
    const double cross_pdt_norm = cross_pdt.norm();

    std::cout << "p_vec:\n" << p_vec << '\n'
              << "n_vec:\n" << n_vec << '\n'
              << "diff_vec:\n" << diff_vec << '\n'
              << "cross_pdt:\n" << cross_pdt << '\n'
              << "cross_pdt_norm: " << cross_pdt_norm << "\n---\n";

    residual_eqn22[0] = cross_pdt_norm - 1.0 / kappa[0];
    return true;
  }

private:
  double x_;
  double y_;
  double z_;
};

}  // namespace tree_mapping_geometry


int main(int argc, char ** argv)
{
  using tree_mapping_geometry::CylinderResidual;
  using tree_mapping_geometry::Cylinder;
  using tree_mapping_geometry::Circle;
  using tree_mapping_geometry::manual_cylinder;

  std::string file_num = "0";
  double rho = 0.0;
  double kappa = 0.0;
  double theta = 0.0;
  double phi = 0.0;
  double alpha = 0.0;

  if (argc > 1) {
    file_num = std::string(argv[1]);
    rho   = std::stod(argv[2]);
    kappa = std::stod(argv[3]);
    theta = std::stod(argv[4]);
    phi   = std::stod(argv[5]);
    alpha = std::stod(argv[6]);
  }

  std::cout << "PARAMS RECEIVED: " << file_num << ", " << rho << ", " << kappa
            << ", " << theta << ", " << phi << ", " << alpha << '\n';

  Cylinder cylinder = manual_cylinder(file_num, /*automatic_angles=*/true);

  std::cout << "== CALCULATING COST ==\n";
  int ctr = 0;
  double tot_cost = 0.0;
  double tot_cost_vec = 0.0;
  double tot_cost_eqn22 = 0.0;
  for (int ring_num = 0; ring_num < cylinder.num_rings; ++ring_num) {
    const Circle & circle = cylinder.rings[ring_num];
    for (int i = 0; i < circle.num_points; ++i) {
      double residual = 0.0;
      double residual_vec = 0.0;
      double residual_eqn22 = 0.0;
      const auto & pt = circle.points[i];
      CylinderResidual cf(pt[0], pt[1], pt[2]);

      std::cout << "---------------- " << ctr << " | "
                << pt[0] << "," << pt[1] << "," << pt[2] << '\n';

      cf(&rho, &kappa, &theta, &phi, &alpha,
         &residual, &residual_vec, &residual_eqn22);
      std::cout << "residual: " << residual << '\n'
                << "residual_vec: " << residual_vec << '\n'
                << "residual_eqn22: " << residual_eqn22 << '\n';

      tot_cost       += residual * residual;
      tot_cost_vec   += residual_vec * residual_vec;
      tot_cost_eqn22 += residual_eqn22 * residual_eqn22;
      ctr += 1;
    }
  }

  std::cout << "tot_cost: "        << tot_cost        << '\n'
            << "tot_cost/2.0: "    << tot_cost / 2.0  << '\n'
            << "tot_cost_vec: "    << tot_cost_vec    << '\n'
            << "tot_cost_vec/2.0: " << tot_cost_vec / 2.0 << '\n'
            << "tot_cost_eqn22: "  << tot_cost_eqn22  << '\n'
            << "tot_cost_eqn22/2.0: " << tot_cost_eqn22 / 2.0 << '\n';

  return 0;
}
