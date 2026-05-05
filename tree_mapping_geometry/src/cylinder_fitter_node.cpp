// SPDX-License-Identifier: MIT
//
// ROS 2 port of tree_mapping/src/cylinderFitting.cpp from the
// DREAMS-lab tree-mapping pipeline (Vishwanatha and Das, ASU).
//
// Loads a per-segment PCD point cloud (XYZ + intensity + ring index)
// representing a single trunk or branch, computes an analytic initial
// cylinder estimate (axis line via PCA, radius via mean per-ring
// distance, angles via the n_theta / n_phi_bar basis), and refines
// the parameter set (rho, kappa, theta, phi, alpha) with Ceres'
// auto-diff Levenberg-Marquardt solver against the same SLOAM-style
// residual we already validated in cylinder_fitting_residual_eval.
// Publishes the source cloud as sensor_msgs/PointCloud2 and the
// fitted-cylinder visualization as visualization_msgs/MarkerArray on
// configurable topics.
//
// Differences from upstream:
//   * Hard-coded `/home/rxth/...` paths replaced with a `base_path`
//     ROS 2 parameter. Upstream's `-f`, `-b`, `-m`, `-r`, `-a`
//     shorthand command-line flags are still parsed as a legacy
//     convenience.
//   * `ros::Rate / ros::ok` -> `rclcpp::WallRate / rclcpp::ok`.
//   * `static` parameter blobs eliminated; the IterationCallback
//     captures the parameter pointers directly.
//   * `acosDomainCheck` no longer std::exit's on out-of-domain
//     numerical roundoff -- it logs and returns NaN, so a failed
//     branch fit only loses that branch.
//   * Commented-out alternative residual forms removed (they are
//     preserved in cylinder_fitting_residual_eval.cpp for reference).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ceres/ceres.h>
#include <ceres/jet.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include "tree_mapping_geometry/pcd_custom_types.hpp"
#include "tree_mapping_geometry/cylinder_fitting_data.hpp"
#include "tree_mapping_geometry/main_utils_and_params.hpp"


namespace tree_mapping_geometry
{

using PT_XYZIR = velodyne_pointcloud::PointXYZIR;

// SLOAM-style cylinder residual, identical math to
// CylinderResidual in cylinderFitting_ResidualEval.cpp but with a
// single residual and no debug printing.  Parameters:
//   rho   distance from origin to cylinder axis minus radius
//   kappa 1 / radius
//   theta polar angle of n
//   phi   azimuth of n
//   alpha rotation of axis a in the (n_theta, n_phi_bar) basis
struct CylinderResidual
{
  CylinderResidual(double x, double y, double z)
  : x(x), y(y), z(z) {}

  template <typename T>
  bool operator()(
    const T * const rho,
    const T * const kappa,
    const T * const theta,
    const T * const phi,
    const T * const alpha,
    T * residual) const
  {
    residual[0] = (kappa[0] / 2.0) * (
      x * x + y * y + z * z
      - 2.0 * rho[0] * (
          x * ceres::cos(phi[0]) * ceres::sin(theta[0])
        + y * ceres::sin(phi[0]) * ceres::sin(theta[0])
        + z * ceres::cos(theta[0]))
      - (
          x * (ceres::cos(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
               - ceres::sin(phi[0]) * ceres::sin(alpha[0]))
        + y * (ceres::sin(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
               + ceres::cos(phi[0]) * ceres::sin(alpha[0]))
        + z * (-ceres::sin(theta[0]) * ceres::cos(alpha[0]))
        )
        * (
          x * (ceres::cos(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
               - ceres::sin(phi[0]) * ceres::sin(alpha[0]))
        + y * (ceres::sin(phi[0]) * ceres::cos(theta[0]) * ceres::cos(alpha[0])
               + ceres::cos(phi[0]) * ceres::sin(alpha[0]))
        + z * (-ceres::sin(theta[0]) * ceres::cos(alpha[0]))
        )
      + rho[0] * rho[0])
      + rho[0]
      - (
          x * ceres::cos(phi[0]) * ceres::sin(theta[0])
        + y * ceres::sin(phi[0]) * ceres::sin(theta[0])
        + z * ceres::cos(theta[0]));
    return true;
  }

  double x;
  double y;
  double z;
};


// Logs (rho, kappa, theta, phi, alpha) at every iteration of the
// Ceres solve so we can replay the convergence offline.  Holds raw
// pointers into the parameter block.
class IterationLogger : public ceres::IterationCallback
{
public:
  IterationLogger(
    rclcpp::Logger logger,
    const double * rho,
    const double * kappa,
    const double * theta,
    const double * phi,
    const double * alpha)
  : logger_(logger), rho_(rho), kappa_(kappa),
    theta_(theta), phi_(phi), alpha_(alpha) {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary & summary) override
  {
    RCLCPP_INFO(
      logger_,
      "iter %2d) rho=%.6f  r=%.6f  theta=%.4f deg  phi=%.4f deg  alpha=%.4f deg",
      summary.iteration,
      *rho_,
      (std::abs(*kappa_) > 0.0) ? 1.0 / *kappa_ : std::numeric_limits<double>::infinity(),
      utils::rad2deg(*theta_),
      utils::rad2deg(*phi_),
      utils::rad2deg(*alpha_));
    return ceres::SOLVER_CONTINUE;
  }

private:
  rclcpp::Logger logger_;
  const double * rho_;
  const double * kappa_;
  const double * theta_;
  const double * phi_;
  const double * alpha_;
};


class CylinderFitter : public rclcpp::Node
{
public:
  CylinderFitter()
  : rclcpp::Node("cylinder_fitter"),
    cloud_(new pcl::PointCloud<PT_XYZIR>),
    axis_cloud_(new pcl::PointCloud<PT_XYZIR>),
    cyl_ring_ends_(NUM_RINGS),
    cyl_ring_ends_bool_(NUM_RINGS, false),
    cyl_axis_points_(NUM_RINGS),
    cyl_axis_points_optim_(NUM_RINGS)
  {
    declare_parameter<std::string>("base_path", "");
    declare_parameter<std::string>("file_num", "0");
    declare_parameter<std::string>("branch_num", "0");
    declare_parameter<bool>("use_manual_cloud", false);
    declare_parameter<bool>("use_real_cloud", false);
    declare_parameter<bool>("invert_alpha", false);
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("cloud_topic", "/tree_mapping/cylinder_pointcloud");
    declare_parameter<std::string>("marker_topic", "/tree_mapping/cylinder_markers");
    declare_parameter<int>("max_iterations", 500);
    declare_parameter<double>("publish_rate_hz", 1.0);

    base_path_       = get_parameter("base_path").as_string();
    file_num_        = get_parameter("file_num").as_string();
    branch_num_      = get_parameter("branch_num").as_string();
    use_manual_      = get_parameter("use_manual_cloud").as_bool();
    use_real_        = get_parameter("use_real_cloud").as_bool();
    invert_alpha_    = get_parameter("invert_alpha").as_bool();
    frame_id_        = get_parameter("frame_id").as_string();
    cloud_topic_     = get_parameter("cloud_topic").as_string();
    marker_topic_    = get_parameter("marker_topic").as_string();
    max_iterations_  = static_cast<int>(get_parameter("max_iterations").as_int());
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 1);
  }

  // Override individual fields from getopt-style flags.  Booleans are
  // OR-merged so an unset CLI flag does not clobber a true value
  // already obtained from ROS 2 parameters.  Strings only override
  // when the caller passes a non-empty value.
  void apply_cli_overrides(
    const std::string & file_num,
    const std::string & branch_num,
    bool use_manual,
    bool use_real,
    bool invert_alpha)
  {
    if (!file_num.empty()) {file_num_ = file_num;}
    if (!branch_num.empty()) {branch_num_ = branch_num;}
    use_manual_   = use_manual_   || use_manual;
    use_real_     = use_real_     || use_real;
    invert_alpha_ = invert_alpha_ || invert_alpha;
  }

  void run()
  {
    std::cout.precision(std::numeric_limits<double>::max_digits10);

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% READ PCD %%%%%%%%%%%%%%");
    if (use_manual_) {
      Cylinder cylinder = manual_cylinder(file_num_, /*automatic_angles=*/true);
      convert_to_pcd(cylinder);
    } else if (use_real_) {
      const std::vector<std::string> prefixes = {
        "_base", "_base_wTrans", "_left", "_left_wOutliers",
        "_left_wOutlierRingOnly", "_right", "_rightleft",
        "_rightright", "_right_wTrans"};
      const int idx = std::stoi(file_num_);
      if (idx < 0 || static_cast<std::size_t>(idx) >= prefixes.size()) {
        RCLCPP_ERROR(get_logger(),
          "use_real_cloud=true but file_num=%s is out of range [0, %zu)",
          file_num_.c_str(), prefixes.size());
        return;
      }
      const std::string in_file = utils::create_file_name(
        base_path_, "cloud_", "0", 8, prefixes[idx], ".pcd");
      RCLCPP_INFO(get_logger(), "Reading %s", in_file.c_str());
      read_pcd_file(in_file);
    } else {
      const std::string pcd_filename =
        base_path_ + "cloud" + file_num_ + "_" + file_num_ + "_" +
        branch_num_ + ".pcd";
      read_pcd_file(pcd_filename);
    }
    if (cloud_->empty()) {
      RCLCPP_ERROR(get_logger(), "Loaded cloud is empty; aborting fit.");
      return;
    }

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% EXTRACT END POINTS %%%%%%%%%%%%%%");
    extract_ring_end_points();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% CALC AXIS POINTS %%%%%%%%%%%%%%");
    calc_axis_points();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% FIT AXIS LINE (PCA) %%%%%%%%%%%%%%");
    fit_axis_line();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% EXTRACT RADIUS %%%%%%%%%%%%%%");
    extract_radius();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% FIND NORMAL VECTOR %%%%%%%%%%%%%%");
    find_normal_vec();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% EXTRACT ANGLE PARAMS %%%%%%%%%%%%%%");
    extract_angle_params();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% OPTIMIZE CYLINDER %%%%%%%%%%%%%%");
    optimize_cylinder();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% CALC CYLINDER MARKER %%%%%%%%%%%%%%");
    calc_cylinder_marker();

    RCLCPP_INFO(get_logger(), "%%%%%%%%%%%%%% PLOT and VISUALIZE %%%%%%%%%%%%%%");
    plot_cylinder_markers();

    RCLCPP_INFO(get_logger(),
      "FIT DONE: rho=%.6f r=%.6f theta=%.4f deg phi=%.4f deg alpha=%.4f deg",
      cyl_rho_optim_, 1.0 / cyl_kappa_optim_,
      utils::rad2deg(cyl_theta_optim_),
      utils::rad2deg(cyl_phi_optim_),
      utils::rad2deg(cyl_alpha_optim_));

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = now();

    rclcpp::WallRate loop_rate(publish_rate_hz_);
    while (rclcpp::ok()) {
      cloud_msg.header.stamp = now();
      cloud_pub_->publish(cloud_msg);
      marker_pub_->publish(marker_array_);
      rclcpp::spin_some(get_node_base_interface());
      loop_rate.sleep();
    }
  }

private:
  // ---- input ------------------------------------------------------
  void read_pcd_file(const std::string & path)
  {
    if (pcl::io::loadPCDFile<PT_XYZIR>(path, *cloud_) == -1) {
      RCLCPP_ERROR(get_logger(), "Couldn't read PCD file '%s'.", path.c_str());
      cloud_->clear();
      return;
    }
    RCLCPP_INFO(get_logger(),
      "Loaded %s (%zu points)", path.c_str(), cloud_->points.size());
  }

  void convert_to_pcd(const Cylinder & cylinder)
  {
    for (int ring_num = 0; ring_num < cylinder.num_rings; ++ring_num) {
      const Circle & circle = cylinder.rings[ring_num];
      for (int i = 0; i < circle.num_points; ++i) {
        PT_XYZIR pt;
        pt.x = static_cast<float>(circle.points[i][0]);
        pt.y = static_cast<float>(circle.points[i][1]);
        pt.z = static_cast<float>(circle.points[i][2]);
        pt.intensity = 1.0f;
        pt.ring = static_cast<std::uint16_t>(ring_num);
        cloud_->push_back(pt);
      }
    }
    RCLCPP_INFO(get_logger(),
      "Synthetic cylinder: %d rings, %d points -> cloud %zu points",
      cylinder.num_rings, cylinder.num_points, cloud_->size());
  }

  // ---- ring end-points & axis -------------------------------------
  void check_q1_q4_points(double & angle_offset) const
  {
    bool q1 = false, q4 = false;
    double min_angle = std::numeric_limits<double>::infinity();
    for (const auto & pt : cloud_->points) {
      const double angle = std::atan2(pt.y, pt.x);
      if (angle >= 0.0 && angle <  M_PI_2) {q1 = true;}
      if (angle <= 0.0 && angle > -M_PI_2) {q4 = true;}
      if (angle < min_angle) {min_angle = angle;}
    }
    if (q1 && q4) {angle_offset = std::abs(min_angle);}
  }

  void extract_ring_end_points()
  {
    double angle_offset = 0.0;
    check_q1_q4_points(angle_offset);

    std::vector<double> ring_min(NUM_RINGS, std::numeric_limits<double>::infinity());
    std::vector<double> ring_max(NUM_RINGS, -std::numeric_limits<double>::infinity());

    for (const auto & pt : cloud_->points) {
      const int ring_num = static_cast<int>(pt.ring);
      double curr = std::atan2(pt.y, pt.x);
      if (angle_offset != 0.0) {
        curr += angle_offset;
      } else {
        curr = (curr < 0) ? (2.0 * M_PI - std::abs(curr)) : curr;
      }
      cyl_ring_ends_bool_[ring_num] = true;
      if (curr < ring_min[ring_num]) {
        ring_min[ring_num] = curr;
        cyl_ring_ends_[ring_num].first = pt;
      }
      if (curr > ring_max[ring_num]) {
        ring_max[ring_num] = curr;
        cyl_ring_ends_[ring_num].second = pt;
      }
    }
  }

  void calc_axis_points()
  {
    bool low_saved = false;
    for (int ring = 0; ring < NUM_RINGS; ++ring) {
      if (!cyl_ring_ends_bool_[ring]) {continue;}
      const PT_XYZIR & a = cyl_ring_ends_[ring].first;
      const PT_XYZIR & b = cyl_ring_ends_[ring].second;
      PT_XYZIR mid;
      mid.x = (a.x + b.x) / 2.0f;
      mid.y = (a.y + b.y) / 2.0f;
      mid.z = (a.z + b.z) / 2.0f;
      mid.intensity = 0.0f;
      mid.ring = static_cast<std::uint16_t>(ring);
      cyl_axis_points_[ring].x = mid.x;
      cyl_axis_points_[ring].y = mid.y;
      cyl_axis_points_[ring].z = mid.z;
      axis_cloud_->push_back(mid);
      if (!low_saved) {cyl_lowest_pt_ = mid; low_saved = true;}
      cyl_highest_pt_ = mid;
    }
    cyl_axis_len_ = std::sqrt(sq_dist(cyl_lowest_pt_, cyl_highest_pt_));
  }

  void fit_axis_line()
  {
    auto tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*axis_cloud_, *tmp);
    const std::size_t n = tmp->size();

    if (n == 0) {
      RCLCPP_ERROR(get_logger(),
        "Cannot fit cylinder for a single ring layer.  Aborting fit.");
      cyl_a_.setZero();
      cyl_a_mean_.setZero();
      cyl_a_pca_.setZero();
      return;
    } else if (n == 2) {
      pcl::PointXYZ pt1 = tmp->points[0];
      pcl::PointXYZ pt2 = tmp->points[1];
      if (pt2.z < pt1.z) {std::swap(pt1, pt2);}
      cyl_a_ = Eigen::Vector3d(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
      cyl_a_mean_ = Eigen::Vector4d(
        (pt1.x + pt2.x) / 2.0, (pt1.y + pt2.y) / 2.0,
        (pt1.z + pt2.z) / 2.0, 1.0);
      cyl_a_pca_.setZero();
      cyl_a_.normalize();
    } else {
      pcl::PCA<pcl::PointXYZ> pca(/*basis_only=*/false);
      pca.setInputCloud(tmp);
      cyl_a_pca_ = pca.getEigenVectors().cast<double>();
      cyl_a_mean_ = pca.getMean().cast<double>();
      cyl_a_ = cyl_a_pca_.col(0);
      cyl_a_.normalize();
    }
  }

  void extract_radius()
  {
    double avg_radius = 0.0;
    int n_valid = 0;
    for (const auto & pt : cloud_->points) {
      const int ring_num = static_cast<int>(pt.ring);
      if (!cyl_ring_ends_bool_[ring_num]) {continue;}
      ++n_valid;
      const pcl::PointXYZ & ctr = cyl_axis_points_[ring_num];
      avg_radius += std::sqrt(sq_dist(pt, ctr));
    }
    if (n_valid == 0) {
      cyl_kappa_ = 1.0;
      RCLCPP_WARN(get_logger(), "No valid ring points; defaulting kappa=1.0");
      return;
    }
    avg_radius /= static_cast<double>(n_valid);
    cyl_kappa_ = 1.0 / avg_radius;
    RCLCPP_INFO(get_logger(),
      "Initial radius=%.6f kappa=%.6f", avg_radius, cyl_kappa_);
  }

  void find_normal_vec()
  {
    // Solve for the foot of the perpendicular from the origin to the
    // axis line through cyl_a_mean_ in direction cyl_a_:
    //    t = (-a . cyl_a_mean_) / (a . a)
    //    cyl_n_ = a*t + cyl_a_mean_
    const double t =
      (-cyl_a_[0] * cyl_a_mean_[0]
       - cyl_a_[1] * cyl_a_mean_[1]
       - cyl_a_[2] * cyl_a_mean_[2]) /
      (cyl_a_[0] * cyl_a_[0]
       + cyl_a_[1] * cyl_a_[1]
       + cyl_a_[2] * cyl_a_[2]);

    cyl_n_[0] = cyl_a_[0] * t + cyl_a_mean_[0];
    cyl_n_[1] = cyl_a_[1] * t + cyl_a_mean_[1];
    cyl_n_[2] = cyl_a_[2] * t + cyl_a_mean_[2];

    cyl_xn_ = cyl_n_;
    cyl_n_.normalize();
    cyl_rho_ = cyl_xn_.norm() - 1.0 / cyl_kappa_;

    RCLCPP_INFO(get_logger(),
      "x1=|n|=%.6f rho=%.6f r=%.6f t=%.6f a.n=%.6f",
      cyl_xn_.norm(), cyl_rho_, 1.0 / cyl_kappa_, t,
      cyl_n_.dot(cyl_a_));
  }

  // ---- math helpers (operate on doubles, mirroring upstream) ------
  static Eigen::Vector3d n_reconstr(double theta, double phi)
  {
    return {std::cos(phi) * std::sin(theta),
            std::sin(phi) * std::sin(theta),
            std::cos(theta)};
  }
  static Eigen::Vector3d n_theta_vec(double theta, double phi)
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
    const Eigen::Vector3d & n_theta_v,
    const Eigen::Vector3d & n_phi_bar_v,
    double alpha)
  {
    return n_theta_v * std::cos(alpha) + n_phi_bar_v * std::sin(alpha);
  }

  double get_theta(const Eigen::Vector3d & n) const
  {
    const Eigen::Vector3d z(0.0, 0.0, 1.0);
    const double v = n.dot(z) / (n.norm() * z.norm());
    const double theta = std::acos(v);
    return std::isnan(theta) ? acos_domain_check(v) : theta;
  }

  double get_phi(const Eigen::Vector3d & n) const
  {
    return std::atan2(n[1], n[0]);
  }

  double get_alpha(
    const Eigen::Vector3d & a,
    const Eigen::Vector3d & n_theta_v,
    const Eigen::Vector3d & n_phi_bar_v) const
  {
    const double v = a.dot(n_theta_v) / (a.norm() * n_theta_v.norm());
    double alpha = std::acos(v);
    if (std::isnan(alpha)) {alpha = acos_domain_check(v);}
    Eigen::MatrixXd A(3, 2);
    A.col(0) = n_theta_v;
    A.col(1) = n_phi_bar_v;
    Eigen::Vector3d b = a;
    Eigen::MatrixXd A_pinv = (A.transpose() * A).inverse() * A.transpose();
    Eigen::Vector2d x = A_pinv * b;
    if (std::atan2(x(1), x(0)) < 0.0) {alpha = -alpha;}
    return alpha;
  }

  double acos_domain_check(double v) const
  {
    constexpr double eps = 1e-6;
    if (std::abs(v - 1.0) <= eps) {
      RCLCPP_WARN(get_logger(), "acos input clipped at +1");
      return 0.0;
    }
    if (std::abs(v + 1.0) <= eps) {
      RCLCPP_WARN(get_logger(), "acos input clipped at -1");
      return M_PI;
    }
    RCLCPP_ERROR(get_logger(),
      "acos input %.9f outside domain and beyond epsilon", v);
    return std::numeric_limits<double>::quiet_NaN();
  }

  void extract_angle_params()
  {
    cyl_theta_ = get_theta(cyl_n_);
    cyl_phi_   = get_phi(cyl_n_);
    cyl_n_reconstr_ = n_reconstr(cyl_theta_, cyl_phi_);
    cyl_n_theta_    = n_theta_vec(cyl_theta_, cyl_phi_);
    cyl_n_phi_bar_  = n_phi_bar(cyl_theta_, cyl_phi_);
    cyl_alpha_      = get_alpha(cyl_a_, cyl_n_theta_, cyl_n_phi_bar_);
    if (invert_alpha_) {cyl_alpha_ = -cyl_alpha_;}
    cyl_a_reconstr_ = a_reconstr(cyl_n_theta_, cyl_n_phi_bar_, cyl_alpha_);

    RCLCPP_INFO(get_logger(),
      "Initial angles theta=%.4f phi=%.4f alpha=%.4f (deg)",
      utils::rad2deg(cyl_theta_),
      utils::rad2deg(cyl_phi_),
      utils::rad2deg(cyl_alpha_));
  }

  // ---- Ceres optimization -----------------------------------------
  void optimize_cylinder()
  {
    using ceres::AutoDiffCostFunction;
    using ceres::Problem;
    using ceres::Solve;
    using ceres::Solver;

    iter_rho_   = cyl_rho_;
    iter_kappa_ = cyl_kappa_;
    iter_theta_ = cyl_theta_;
    iter_phi_   = cyl_phi_;
    iter_alpha_ = cyl_alpha_;

    Problem problem;
    for (const auto & p : cloud_->points) {
      problem.AddResidualBlock(
        new AutoDiffCostFunction<CylinderResidual, 1, 1, 1, 1, 1, 1>(
          new CylinderResidual(p.x, p.y, p.z)),
        nullptr,
        &iter_rho_, &iter_kappa_, &iter_theta_, &iter_phi_, &iter_alpha_);
    }

    Solver::Options options;
    options.max_num_iterations = max_iterations_;
    options.use_nonmonotonic_steps = false;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    options.update_state_every_iteration = true;

    IterationLogger cb(get_logger(),
      &iter_rho_, &iter_kappa_, &iter_theta_, &iter_phi_, &iter_alpha_);
    options.callbacks.push_back(&cb);

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << '\n';

    cyl_rho_optim_   = iter_rho_;
    cyl_kappa_optim_ = iter_kappa_;
    cyl_theta_optim_ = iter_theta_;
    cyl_phi_optim_   = iter_phi_;
    cyl_alpha_optim_ = iter_alpha_;
    cyl_n_reconstr_optim_       = n_reconstr(iter_theta_, iter_phi_);
    cyl_n_theta_reconstr_optim_ = n_theta_vec(iter_theta_, iter_phi_);
    cyl_n_phi_bar_reconstr_optim_ = n_phi_bar(iter_theta_, iter_phi_);
    cyl_a_reconstr_optim_ = a_reconstr(
      cyl_n_theta_reconstr_optim_, cyl_n_phi_bar_reconstr_optim_, iter_alpha_);
    cyl_xn_optim_ = (cyl_rho_optim_ + 1.0 / cyl_kappa_optim_) * cyl_n_reconstr_optim_;
  }

  // ---- visualization ----------------------------------------------
  void calc_cylinder_marker()
  {
    if (std::abs(cyl_a_reconstr_optim_[2]) < 1e-9) {
      RCLCPP_WARN(get_logger(),
        "Optimized axis is nearly horizontal; cylinder caps may be skewed.");
      cyl_lowest_pt_optim_ = cyl_lowest_pt_;
      cyl_highest_pt_optim_ = cyl_highest_pt_;
      return;
    }
    cyl_lowest_pt_optim_.z = cyl_lowest_pt_.z;
    double t = (cyl_lowest_pt_.z - cyl_xn_optim_[2]) / cyl_a_reconstr_optim_[2];
    cyl_lowest_pt_optim_.x = static_cast<float>(t * cyl_a_reconstr_optim_[0] + cyl_xn_optim_[0]);
    cyl_lowest_pt_optim_.y = static_cast<float>(t * cyl_a_reconstr_optim_[1] + cyl_xn_optim_[1]);

    cyl_highest_pt_optim_.z = cyl_highest_pt_.z;
    t = (cyl_highest_pt_.z - cyl_xn_optim_[2]) / cyl_a_reconstr_optim_[2];
    cyl_highest_pt_optim_.x = static_cast<float>(t * cyl_a_reconstr_optim_[0] + cyl_xn_optim_[0]);
    cyl_highest_pt_optim_.y = static_cast<float>(t * cyl_a_reconstr_optim_[1] + cyl_xn_optim_[1]);
  }

  int next_marker_id() {return ++marker_id_;}

  void add_sphere_marker(
    int id, double x, double y, double z,
    double sx, double sy, double sz,
    double r, double g, double b, double a)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = now();
    mk.ns = "cylinder";
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = z;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = sx;
    mk.scale.y = sy;
    mk.scale.z = sz;
    mk.color.r = static_cast<float>(r);
    mk.color.g = static_cast<float>(g);
    mk.color.b = static_cast<float>(b);
    mk.color.a = static_cast<float>(a);
    marker_array_.markers.push_back(mk);
  }

  void add_arrow_two_points(
    int id,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double shaft_diam, double head_diam, double head_len,
    double r, double g, double b, double a)
  {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = now();
    mk.ns = "cylinder";
    mk.id = id;
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.scale.x = shaft_diam;
    mk.scale.y = head_diam;
    mk.scale.z = head_len;
    mk.points.push_back(p1);
    mk.points.push_back(p2);
    mk.pose.orientation.w = 1.0;
    mk.color.r = static_cast<float>(r);
    mk.color.g = static_cast<float>(g);
    mk.color.b = static_cast<float>(b);
    mk.color.a = static_cast<float>(a);
    marker_array_.markers.push_back(mk);
  }

  static geometry_msgs::msg::Point make_point(double x, double y, double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
  }

  void plot_cylinder_markers()
  {
    // Initial axis points (yellow spheres).
    for (const auto & pt : axis_cloud_->points) {
      add_sphere_marker(next_marker_id(),
        pt.x, pt.y, pt.z, 0.1, 0.1, 0.1, 1.0, 1.0, 0.0, 1.0);
    }

    // Initial axis vector (blue arrow).
    {
      const double half = cyl_axis_len_ / 2.0;
      auto p1 = make_point(
        cyl_a_mean_[0] - half * cyl_a_[0],
        cyl_a_mean_[1] - half * cyl_a_[1],
        cyl_a_mean_[2] - half * cyl_a_[2]);
      auto p2 = make_point(
        cyl_a_mean_[0] + half * cyl_a_[0],
        cyl_a_mean_[1] + half * cyl_a_[1],
        cyl_a_mean_[2] + half * cyl_a_[2]);
      add_arrow_two_points(next_marker_id(), p1, p2,
        0.1, 0.2, cyl_axis_len_ / 3.0, 0.0, 0.0, 1.0, 0.5);
    }

    // Initial normal vector cyl_xn_ (green).
    {
      auto p1 = make_point(0.0, 0.0, 0.0);
      auto p2 = make_point(cyl_xn_[0], cyl_xn_[1], cyl_xn_[2]);
      add_arrow_two_points(next_marker_id(), p1, p2,
        0.1, 0.2, cyl_xn_.norm() / 3.0, 0.0, 1.0, 0.0, 0.5);
    }

    // Optimized axis (blue, thinner).
    {
      const double s = 10.0;
      auto p1 = make_point(0.0, 0.0, 0.0);
      auto p2 = make_point(
        cyl_a_reconstr_optim_[0] * s,
        cyl_a_reconstr_optim_[1] * s,
        cyl_a_reconstr_optim_[2] * s);
      add_arrow_two_points(next_marker_id(), p1, p2,
        0.05, 0.1, (cyl_a_reconstr_optim_.norm() * s) / 3.0,
        0.0, 0.0, 1.0, 0.8);
    }

    // Optimized normal vector (green, thinner).
    {
      const double s = cyl_rho_optim_ + 1.0 / cyl_kappa_optim_;
      auto p1 = make_point(0.0, 0.0, 0.0);
      auto p2 = make_point(
        cyl_n_reconstr_optim_[0] * s,
        cyl_n_reconstr_optim_[1] * s,
        cyl_n_reconstr_optim_[2] * s);
      add_arrow_two_points(next_marker_id(), p1, p2,
        0.05, 0.1, (cyl_n_reconstr_optim_.norm() * s) / 3.0,
        0.0, 1.0, 0.0, 0.8);
    }

    // Optimized cylinder body: arrow with diameter = 2 * radius and
    // negligible head -> renders as a capsule between
    // (cyl_lowest_pt_optim_, cyl_highest_pt_optim_).
    {
      auto p1 = make_point(
        cyl_lowest_pt_optim_.x,
        cyl_lowest_pt_optim_.y,
        cyl_lowest_pt_optim_.z);
      auto p2 = make_point(
        cyl_highest_pt_optim_.x,
        cyl_highest_pt_optim_.y,
        cyl_highest_pt_optim_.z);
      const double diameter = 2.0 / cyl_kappa_optim_;
      add_arrow_two_points(next_marker_id(), p1, p2,
        diameter, diameter, 1e-6, 1.0, 1.0, 1.0, 0.6);
    }
  }

  template <typename PtA, typename PtB>
  static double sq_dist(const PtA & a, const PtB & b)
  {
    return (a.x - b.x) * (a.x - b.x) +
           (a.y - b.y) * (a.y - b.y) +
           (a.z - b.z) * (a.z - b.z);
  }

  // ---- members ----------------------------------------------------
  static constexpr int NUM_RINGS = 16;

  std::string base_path_;
  std::string file_num_;
  std::string branch_num_;
  bool use_manual_   = false;
  bool use_real_     = false;
  bool invert_alpha_ = false;
  std::string frame_id_;
  std::string cloud_topic_;
  std::string marker_topic_;
  int max_iterations_ = 500;
  double publish_rate_hz_ = 1.0;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  pcl::PointCloud<PT_XYZIR>::Ptr cloud_;
  pcl::PointCloud<PT_XYZIR>::Ptr axis_cloud_;
  std::vector<std::pair<PT_XYZIR, PT_XYZIR>> cyl_ring_ends_;
  std::vector<bool> cyl_ring_ends_bool_;
  std::vector<pcl::PointXYZ> cyl_axis_points_;
  std::vector<pcl::PointXYZ> cyl_axis_points_optim_;

  // Initial estimate.
  Eigen::Vector3d cyl_n_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_xn_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_n_reconstr_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_n_theta_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_n_phi_bar_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_a_reconstr_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d cyl_a_mean_ = Eigen::Vector4d::Zero();
  Eigen::Matrix3d cyl_a_pca_ = Eigen::Matrix3d::Zero();
  double cyl_axis_len_ = 0.0;
  double cyl_rho_ = 0.0;
  double cyl_kappa_ = 1.0;
  double cyl_theta_ = 0.0;
  double cyl_phi_ = 0.0;
  double cyl_alpha_ = 0.0;

  // Optimized.
  double cyl_rho_optim_ = 0.0;
  double cyl_kappa_optim_ = 1.0;
  double cyl_theta_optim_ = 0.0;
  double cyl_phi_optim_ = 0.0;
  double cyl_alpha_optim_ = 0.0;
  Eigen::Vector3d cyl_n_reconstr_optim_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_a_reconstr_optim_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_n_theta_reconstr_optim_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_n_phi_bar_reconstr_optim_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d cyl_xn_optim_ = Eigen::Vector3d::Zero();
  PT_XYZIR cyl_lowest_pt_{};
  PT_XYZIR cyl_highest_pt_{};
  PT_XYZIR cyl_lowest_pt_optim_{};
  PT_XYZIR cyl_highest_pt_optim_{};

  // Ceres parameter blocks (kept as members so the IterationLogger
  // can hold pointers into them for the lifetime of the solve).
  double iter_rho_   = 0.0;
  double iter_kappa_ = 1.0;
  double iter_theta_ = 0.0;
  double iter_phi_   = 0.0;
  double iter_alpha_ = 0.0;

  visualization_msgs::msg::MarkerArray marker_array_;
  int marker_id_ = 0;
};

}  // namespace tree_mapping_geometry


int main(int argc, char ** argv)
{
  // Strip ROS args (everything after `--ros-args`) before parsing
  // our own getopt-style flags, otherwise getopt sees `--ros-args`
  // and bails on the unknown long option.
  const auto cli = rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::vector<char *> cli_argv;
  cli_argv.reserve(cli.size());
  for (const auto & s : cli) {
    cli_argv.push_back(const_cast<char *>(s.c_str()));
  }
  const int cli_argc = static_cast<int>(cli_argv.size());

  // Empty defaults so apply_cli_overrides can tell what the user
  // passed (vs. relying on ROS 2 parameter defaults).
  std::string file_num = "";
  std::string branch_num = "";
  bool use_manual = false;
  bool use_real = false;
  bool invert_alpha = false;

  optind = 1;  // reset getopt state in case of reuse
  int input_arg;
  while ((input_arg = getopt(cli_argc, cli_argv.data(), "af:b:mrh")) != -1) {
    switch (input_arg) {
      case 'a': invert_alpha = true; break;
      case 'f': file_num = std::string(optarg); break;
      case 'b': branch_num = std::string(optarg); break;
      case 'm': use_manual = true; break;
      case 'r': use_real = true; break;
      case 'h':
        std::cout << "Options: -f [file_num] -b [branch_num] "
                     "-m (manual cloud) -r (real cloud) -a (invert alpha)\n"
                  << "ROS 2 parameters: base_path, file_num, branch_num, "
                     "use_manual_cloud, use_real_cloud, invert_alpha, "
                     "frame_id, cloud_topic, marker_topic, max_iterations, "
                     "publish_rate_hz\n";
        rclcpp::shutdown();
        return 0;
      default:
        std::cerr << "Unrecognized option.\n";
        rclcpp::shutdown();
        return 1;
    }
  }

  auto node = std::make_shared<tree_mapping_geometry::CylinderFitter>();
  node->apply_cli_overrides(file_num, branch_num, use_manual, use_real, invert_alpha);
  node->run();
  rclcpp::shutdown();
  return 0;
}
