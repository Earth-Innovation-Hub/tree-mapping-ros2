// SPDX-License-Identifier: MIT
//
// ROS 2 port of tree_mapping/src/branchInstanceSegmentation.cpp from
// the DREAMS-lab tree-mapping pipeline (Vishwanatha and Das, ASU).
//
// Stage 2 of the pipeline (Trunk and Branch Separation):
//   1. Load a pre-segmented per-tree PCD.
//   2. Build a spherical range image.
//   3. Range-band filter around the tree base (+/- epsilon).
//   4. Row-by-row, left-to-right pair-wise d_thresh segmentation
//      that assigns each pixel to a branch instance index.
//   5. Publish: original cloud, filtered cloud, instance-colored
//      cloud (intensity = instance_idx + 1), and the range image.
//
// Differences from upstream:
//   * Hard-coded `/home/rxth/...` paths replaced with ROS 2
//     parameters (`base_path`, `output_dir`).
//   * Per-file `(base_xyz, epsilon_before, epsilon_after)` table
//     lifted into individual ROS 2 parameters so the node works on
//     arbitrary clouds.  For backward compat with the upstream test
//     set, providing only `file_num` looks up sensible defaults
//     (the same table the upstream binary baked in).
//   * Topic names exposed as ROS 2 parameters.
//   * `image_transport` dropped in favour of a plain
//     `rclcpp::Publisher<sensor_msgs::msg::Image>`; we never used
//     the transport plug-in features.
//   * `std::exit` removed from the core algorithm; unexpected cases
//     are logged and skipped instead.
//   * The dead/placeholder image subscriber, the `compare1/2`
//     scaffolding, and the noisy commented-out per-cloud d_thresh
//     remarks are dropped.
//   * Modern PCL / pcl_conversions / `*::msg::*` types throughout.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <unistd.h>

#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/common/common_headers.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "tree_mapping_geometry/pcd_custom_types.hpp"
#include "tree_mapping_geometry/main_utils_and_params.hpp"


namespace tree_mapping_geometry
{

using PointType = velodyne_pointcloud::PointXYZIR;


// Default per-file (base_xyz, epsilon_before, epsilon_after) values
// from the upstream binary, indexed by `file_num`.  Only used when the
// user passes -f / file_num and does not override `base_xyz`.
struct UpstreamTestEntry
{
  int file_num;
  double base_x;
  double base_y;
  double base_z;
  double eps_before;
  double eps_after;
};

constexpr UpstreamTestEntry kUpstreamTestSet[] = {
  { 0, -15.94, 20.45,  0.45, 2.0, 7.0},
  { 1, -13.62,  6.40,  0.26, 2.0, 7.0},
  { 2, -20.83,  0.27,  0.36, 2.0, 7.0},
  { 3, -13.77, -4.90,  0.25, 2.0, 7.0},
  { 4,  -9.10, -6.00, -1.72, 3.0, 2.0},
  { 5,   8.97, -7.26, -1.41, 3.0, 4.0},
  { 6,   9.07, -4.14, -1.21, 3.0, 4.0},
  { 7,   8.66, -4.44, -1.54, 3.0, 4.0},
  { 8,   8.68, -3.66, -1.49, 3.0, 4.0},
  { 9,   5.62, -0.68, -0.89, 3.0, 5.0},
  {10,   9.58, -1.75, -1.54, 3.0, 5.0},
};


class BranchSegmenter : public rclcpp::Node
{
public:
  BranchSegmenter()
  : rclcpp::Node("branch_segmenter"),
    cloud_(new pcl::PointCloud<PointType>),
    cloud_filtered_(new pcl::PointCloud<PointType>),
    cloud_instance_(new pcl::PointCloud<PointType>)
  {
    declare_parameter<std::string>("base_path", "");
    declare_parameter<std::string>("output_dir", "");
    declare_parameter<std::string>("file_num", "0");
    declare_parameter<double>("angular_resolution_x_deg", 0.1);
    declare_parameter<double>("angular_resolution_y_deg", 0.1);
    declare_parameter<double>("d_thresh", 0.3);

    // Per-tree base position + range-band tolerance.  An empty
    // base_xyz means "fall back to the upstream test-set table for
    // this file_num".  Override either or both eps_*.
    declare_parameter<std::vector<double>>("base_xyz", std::vector<double>{});
    declare_parameter<double>("eps_before", -1.0);
    declare_parameter<double>("eps_after",  -1.0);

    declare_parameter<bool>("save_range_image_pngs", true);
    declare_parameter<bool>("save_instance_mask_csv", true);

    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("topic_cloud_original",  "/tree_mapping/cloud_original");
    declare_parameter<std::string>("topic_cloud_filtered",  "/tree_mapping/cloud_filtered");
    declare_parameter<std::string>("topic_cloud_instance",  "/tree_mapping/cloud_instance");
    declare_parameter<std::string>("topic_range_image",     "/tree_mapping/range_image");
    declare_parameter<double>("publish_rate_hz", 1.0);

    base_path_ = get_parameter("base_path").as_string();
    output_dir_ = get_parameter("output_dir").as_string();
    file_num_  = get_parameter("file_num").as_string();
    angular_resolution_x_deg_ = get_parameter("angular_resolution_x_deg").as_double();
    angular_resolution_y_deg_ = get_parameter("angular_resolution_y_deg").as_double();
    d_thresh_ = static_cast<float>(get_parameter("d_thresh").as_double());

    base_xyz_  = get_parameter("base_xyz").as_double_array();
    eps_before_ = get_parameter("eps_before").as_double();
    eps_after_  = get_parameter("eps_after").as_double();

    save_pngs_ = get_parameter("save_range_image_pngs").as_bool();
    save_csv_  = get_parameter("save_instance_mask_csv").as_bool();

    frame_id_ = get_parameter("frame_id").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    pub_orig_   = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("topic_cloud_original").as_string(), qos);
    pub_filt_   = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("topic_cloud_filtered").as_string(), qos);
    pub_inst_   = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("topic_cloud_instance").as_string(), qos);
    pub_image_  = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("topic_range_image").as_string(), qos);

    if (output_dir_.empty()) {output_dir_ = base_path_;}
  }

  // CLI shorthand overrides (-f / -x / -y / -d), only applied when
  // non-default to preserve any ROS 2 parameter values.
  void apply_cli_overrides(
    const std::string & file_num, double x_deg, double y_deg, float d_thresh)
  {
    if (!file_num.empty()) {file_num_ = file_num;}
    if (x_deg > 0.0) {angular_resolution_x_deg_ = x_deg;}
    if (y_deg > 0.0) {angular_resolution_y_deg_ = y_deg;}
    if (d_thresh > 0.0f) {d_thresh_ = d_thresh;}
  }

  void run()
  {
    angular_resolution_x_deg_ = std::max(angular_resolution_x_deg_, 0.1);
    angular_resolution_y_deg_ = std::max(angular_resolution_y_deg_, 0.1);

    // Resolve base_xyz: explicit param takes priority; otherwise fall
    // back to the upstream test-set table indexed by file_num.
    if (base_xyz_.size() != 3) {
      const int idx = std::stoi(file_num_);
      const auto * entry = lookup_test_entry(idx);
      if (!entry) {
        RCLCPP_ERROR(get_logger(),
          "base_xyz parameter not provided and no upstream default for "
          "file_num=%s; cannot continue.", file_num_.c_str());
        return;
      }
      base_xyz_ = {entry->base_x, entry->base_y, entry->base_z};
      if (eps_before_ < 0.0) {eps_before_ = entry->eps_before;}
      if (eps_after_  < 0.0) {eps_after_  = entry->eps_after;}
      RCLCPP_INFO(get_logger(),
        "Using upstream defaults for file_num=%s: base=(%.3f,%.3f,%.3f) "
        "eps_before=%.2f eps_after=%.2f",
        file_num_.c_str(), base_xyz_[0], base_xyz_[1], base_xyz_[2],
        eps_before_, eps_after_);
    } else {
      if (eps_before_ < 0.0) {eps_before_ = 2.0;}
      if (eps_after_  < 0.0) {eps_after_  = 7.0;}
      RCLCPP_INFO(get_logger(),
        "Using user base=(%.3f,%.3f,%.3f) eps_before=%.2f eps_after=%.2f",
        base_xyz_[0], base_xyz_[1], base_xyz_[2], eps_before_, eps_after_);
    }
    const double approx_range = std::sqrt(
      base_xyz_[0] * base_xyz_[0] +
      base_xyz_[1] * base_xyz_[1] +
      base_xyz_[2] * base_xyz_[2]);

    // ---- load PCD ------------------------------------------------
    const std::string pcd_path = base_path_ +
      "cloud" + file_num_ + "_" + file_num_ + ".000000.pcd";
    RCLCPP_INFO(get_logger(), "Loading PCD: %s", pcd_path.c_str());
    if (pcl::io::loadPCDFile<PointType>(pcd_path, *cloud_) == -1) {
      RCLCPP_ERROR(get_logger(), "Unable to open '%s'.", pcd_path.c_str());
      return;
    }
    if (cloud_->empty()) {
      RCLCPP_ERROR(get_logger(), "Loaded cloud is empty.");
      return;
    }
    RCLCPP_INFO(get_logger(),
      "Loaded %zu 3D points; ang res (deg) x=%.3f y=%.3f; d_thresh=%.3f",
      cloud_->size(), angular_resolution_x_deg_,
      angular_resolution_y_deg_, d_thresh_);

    // ---- build spherical range image -----------------------------
    angular_resolution_x_ = pcl::deg2rad(static_cast<float>(angular_resolution_x_deg_));
    angular_resolution_y_ = pcl::deg2rad(static_cast<float>(angular_resolution_y_deg_));

    Eigen::Affine3f scene_sensor_pose =
      Eigen::Affine3f(Eigen::Translation3f(
        cloud_->sensor_origin_[0],
        cloud_->sensor_origin_[1],
        cloud_->sensor_origin_[2])) *
      Eigen::Affine3f(Eigen::Quaternion<float>(1, 0, 0, 0));

    auto range_image_ptr = std::make_shared<pcl::RangeImage>();
    pcl::RangeImage & range_image = *range_image_ptr;
    range_image.createFromPointCloud(
      *cloud_, angular_resolution_x_, angular_resolution_y_,
      pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
      scene_sensor_pose, pcl::RangeImage::LASER_FRAME,
      /*noise_level=*/0.0f, /*min_range=*/0.0f, /*border_size=*/0);

    int height = range_image.height;
    int width  = range_image.width;
    RCLCPP_INFO(get_logger(),
      "Range image (HxW) = %dx%d", height, width);

    if (save_pngs_) {
      const float * ranges = range_image.getRangesArray();
      unsigned char * rgb = pcl::visualization::FloatImageUtils::getVisualImage(
        ranges, range_image.width, range_image.height);
      const std::string out = output_dir_ +
        "range_image_" + file_num_ +
        "_x_" + std::to_string(pcl::rad2deg(angular_resolution_x_)) +
        "_y_" + std::to_string(pcl::rad2deg(angular_resolution_y_)) + ".png";
      pcl::io::saveRgbPNGFile(out, rgb, range_image.width, range_image.height);
      delete[] rgb;
    }

    // ---- range-band filter --------------------------------------
    const float lo = static_cast<float>(std::abs(approx_range - eps_before_));
    const float hi = static_cast<float>(std::abs(approx_range + eps_after_));
    RCLCPP_INFO(get_logger(),
      "Range-band filter: keep %.3f m <= range <= %.3f m (approx=%.3f)",
      lo, hi, approx_range);

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        pcl::PointWithRange pt = range_image.at(col, row);
        if (pt.range > lo && pt.range < hi) {
          PointType filt;
          filt.x = pt.x; filt.y = pt.y; filt.z = pt.z;
          filt.intensity = 10.0f; filt.ring = 10;
          cloud_filtered_->points.push_back(filt);
        } else {
          pt.x = std::numeric_limits<float>::infinity();
          pt.y = std::numeric_limits<float>::infinity();
          pt.z = std::numeric_limits<float>::infinity();
          pt.range = std::numeric_limits<float>::quiet_NaN();
          range_image.at(col, row) = pt;
        }
      }
    }
    range_image.recalculate3DPointPositions();
    height = range_image.height;
    width  = range_image.width;
    RCLCPP_INFO(get_logger(), "After filter: %dx%d", height, width);

    if (save_pngs_) {
      const float * ranges = range_image.getRangesArray();
      unsigned char * rgb = pcl::visualization::FloatImageUtils::getVisualImage(
        ranges, range_image.width, range_image.height);
      const std::string out = output_dir_ +
        "range_image_modified_" + file_num_ +
        "_x_" + std::to_string(pcl::rad2deg(angular_resolution_x_)) +
        "_y_" + std::to_string(pcl::rad2deg(angular_resolution_y_)) + ".png";
      pcl::io::saveRgbPNGFile(out, rgb, range_image.width, range_image.height);
      delete[] rgb;
    }

    // ---- instance segmentation ----------------------------------
    std::vector<std::vector<int>> instance_mask(
      height, std::vector<int>(width, -1));

    int instance_idx = 0;
    int prev_row_max_instance_idx = 0;
    int prev_row_min_instance_idx = 0;
    int row_ctr = 0;
    int prev_row_tot = 0;
    bool is_new_row = false;
    bool is_valid_row = false;
    bool instances_increased = false;

    PointType pt_instance;

    for (int row = height - 1; row >= 0; --row) {
      is_new_row = true;
      is_valid_row = false;

      for (int col = 0; col < width; ++col) {
        pcl::PointWithRange pt1 = range_image.at(col, row);
        if (utils::is_nan_or_inf(pt1.range)) {continue;}
        is_valid_row = true;

        if (is_new_row) {
          prev_row_min_instance_idx = instance_idx;
          row_ctr = 0;
          is_new_row = false;
        }

        instance_mask[row][col] = instance_idx;

        pt_instance.x = pt1.x;
        pt_instance.y = pt1.y;
        pt_instance.z = pt1.z;
        pt_instance.intensity = static_cast<float>(instance_idx + 1);
        pt_instance.ring = 10;
        cloud_instance_->points.push_back(pt_instance);

        const int next_col = col + 1;
        if (next_col == width) {break;}

        pcl::PointWithRange pt2 = range_image.at(next_col, row);
        const float d = pt_distance(pt1, pt2);

        if (!utils::is_nan_or_inf(pt2.range) && d < d_thresh_) {
          continue;  // same instance
        } else if (!utils::is_nan_or_inf(pt2.range) && d >= d_thresh_) {
          new_instance_in_row(instance_idx, row_ctr, prev_row_tot,
            instances_increased, row);
          if (instances_increased) {break;}
        } else if (utils::is_nan_or_inf(pt2.range)) {
          int probe_col = col;
          const int check = increase_idx_check(
            row, probe_col, width, range_image, pt1);
          if (check == -1) {
            continue;  // reached end of row
          } else if (check == -2) {
            col = probe_col - 1;  // resume scanning across the gap
            continue;
          } else if (check == 1) {
            new_instance_in_row(instance_idx, row_ctr, prev_row_tot,
              instances_increased, row);
            if (instances_increased) {break;}
          } else {
            RCLCPP_WARN(get_logger(),
              "Unrecognized increase_idx_check return %d at (row=%d, col=%d)",
              check, row, col);
          }
        }
      }

      if (is_valid_row) {
        prev_row_tot = row_ctr;
        prev_row_max_instance_idx = instance_idx;
        if (instances_increased) {
          instance_idx = prev_row_max_instance_idx + 1;
          instances_increased = false;
        } else {
          instance_idx = prev_row_min_instance_idx;
        }
      }
    }

    RCLCPP_INFO(get_logger(),
      "Instance segmentation: %zu cluster points, max instance idx %d",
      cloud_instance_->size(),
      max_instance_id(instance_mask));

    // ---- save instance mask CSV ---------------------------------
    if (save_csv_) {
      const std::string csv = output_dir_ +
        "branch_instances_" + file_num_ +
        "_x_" + std::to_string(angular_resolution_x_deg_) +
        "_y_" + std::to_string(angular_resolution_y_deg_) +
        "_d_" + std::to_string(d_thresh_) + ".txt";
      std::ofstream out(csv);
      for (const auto & row : instance_mask) {
        for (int v : row) {
          if (v < 0) {
            out << v << ", ";
          } else if (v < 10) {
            out << "0" << v << ", ";
          } else {
            out << v << ", ";
          }
        }
        out << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote instance mask: %s", csv.c_str());
    }

    // ---- prepare publishable artifacts --------------------------
    const float * ranges_after = range_image.getRangesArray();
    unsigned char * rgb_after = pcl::visualization::FloatImageUtils::getVisualImage(
      ranges_after, range_image.width, range_image.height);
    cv::Mat range_image_cv(height, width, CV_8UC3);
    {
      const unsigned char * p = rgb_after;
      for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
          range_image_cv.at<cv::Vec3b>(h, w) = cv::Vec3b(p[0], p[1], p[2]);
          p += 3;
        }
      }
    }
    delete[] rgb_after;

    sensor_msgs::msg::Image::SharedPtr range_image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", range_image_cv).toImageMsg();
    range_image_msg->header.frame_id = frame_id_;

    sensor_msgs::msg::PointCloud2 msg_orig, msg_filt, msg_inst;
    pcl::toROSMsg(*cloud_, msg_orig);
    pcl::toROSMsg(*cloud_filtered_, msg_filt);
    pcl::toROSMsg(*cloud_instance_, msg_inst);
    msg_orig.header.frame_id = frame_id_;
    msg_filt.header.frame_id = frame_id_;
    msg_inst.header.frame_id = frame_id_;

    rclcpp::WallRate loop_rate(publish_rate_hz_);
    while (rclcpp::ok()) {
      const auto now_stamp = now();
      msg_orig.header.stamp = now_stamp;
      msg_filt.header.stamp = now_stamp;
      msg_inst.header.stamp = now_stamp;
      range_image_msg->header.stamp = now_stamp;
      pub_orig_->publish(msg_orig);
      pub_filt_->publish(msg_filt);
      pub_inst_->publish(msg_inst);
      pub_image_->publish(*range_image_msg);
      rclcpp::spin_some(get_node_base_interface());
      loop_rate.sleep();
    }
  }

private:
  static const UpstreamTestEntry * lookup_test_entry(int file_num)
  {
    for (const auto & e : kUpstreamTestSet) {
      if (e.file_num == file_num) {return &e;}
    }
    return nullptr;
  }

  template <typename T>
  static float pt_distance(T pt1, T pt2)
  {
    if (std::isnan(pt1.x)) {pt1.x = std::numeric_limits<float>::infinity();}
    if (std::isnan(pt1.y)) {pt1.y = std::numeric_limits<float>::infinity();}
    if (std::isnan(pt1.z)) {pt1.z = std::numeric_limits<float>::infinity();}
    if (std::isnan(pt2.x)) {pt2.x = std::numeric_limits<float>::infinity();}
    if (std::isnan(pt2.y)) {pt2.y = std::numeric_limits<float>::infinity();}
    if (std::isnan(pt2.z)) {pt2.z = std::numeric_limits<float>::infinity();}
    return std::sqrt(
      (pt1.x - pt2.x) * (pt1.x - pt2.x) +
      (pt1.y - pt2.y) * (pt1.y - pt2.y) +
      (pt1.z - pt2.z) * (pt1.z - pt2.z));
  }

  static int max_instance_id(const std::vector<std::vector<int>> & mask)
  {
    int m = -1;
    for (const auto & r : mask) {
      for (int v : r) {if (v > m) {m = v;}}
    }
    return m;
  }

  // When pt2 is NaN, walk forward until we either find a valid pt2,
  // hit the row's end, or determine that the gap was just a brief
  // dropout vs. a true new instance.
  //   -1 : reached end of row, no further branches in row
  //   -2 : short break in same instance, resume from probe_col
  //    1 : found a new instance further along the row
  int increase_idx_check(
    int row,
    int & probe_col,
    int width,
    const pcl::RangeImage & range_image,
    const pcl::PointWithRange & pt1)
  {
    // Iterative version of upstream's recursive helper; same
    // semantics, no risk of stack overflow on wide range images.
    while (true) {
      probe_col += 1;
      if (probe_col == width) {return -1;}
      const pcl::PointWithRange pt2 = range_image.at(probe_col, row);
      if (utils::is_nan_or_inf(pt2.range)) {continue;}
      const float d = pt_distance(pt1, pt2);
      if (d < d_thresh_) {return -2;}
      return 1;
    }
  }

  void new_instance_in_row(
    int & instance_idx, int & row_ctr, int prev_row_tot,
    bool & instances_increased, int & row)
  {
    instance_idx += 1;
    row_ctr += 1;
    if (row_ctr > prev_row_tot) {
      instances_increased = true;
      instance_idx -= 1;
      row += 1;  // restart this row in the outer loop
    }
  }

  // members
  std::string base_path_;
  std::string output_dir_;
  std::string file_num_;
  std::string frame_id_;
  double angular_resolution_x_deg_ = 0.1;
  double angular_resolution_y_deg_ = 0.1;
  float angular_resolution_x_ = 0.0f;
  float angular_resolution_y_ = 0.0f;
  float d_thresh_ = 0.3f;
  std::vector<double> base_xyz_;
  double eps_before_ = -1.0;
  double eps_after_  = -1.0;
  bool save_pngs_ = true;
  bool save_csv_ = true;
  double publish_rate_hz_ = 1.0;

  pcl::PointCloud<PointType>::Ptr cloud_;
  pcl::PointCloud<PointType>::Ptr cloud_filtered_;
  pcl::PointCloud<PointType>::Ptr cloud_instance_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_orig_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filt_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_inst_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
};

}  // namespace tree_mapping_geometry


int main(int argc, char ** argv)
{
  const auto cli = rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::vector<char *> cli_argv;
  cli_argv.reserve(cli.size());
  for (const auto & s : cli) {
    cli_argv.push_back(const_cast<char *>(s.c_str()));
  }
  const int cli_argc = static_cast<int>(cli_argv.size());

  std::string file_num;
  double x_deg = -1.0;
  double y_deg = -1.0;
  float d_thresh = -1.0f;

  optind = 1;
  int opt;
  while ((opt = getopt(cli_argc, cli_argv.data(), "f:x:y:d:h")) != -1) {
    switch (opt) {
      case 'f': file_num = optarg; break;
      case 'x': x_deg = std::stod(optarg); break;
      case 'y': y_deg = std::stod(optarg); break;
      case 'd': d_thresh = std::stof(optarg); break;
      case 'h':
        std::cout
          << "Options: -f [file_num] -x [angular_resolution_x_deg]\n"
          << "         -y [angular_resolution_y_deg] -d [d_thresh]\n"
          << "ROS 2 parameters: base_path, output_dir, file_num,\n"
          << "  angular_resolution_{x,y}_deg, d_thresh, base_xyz,\n"
          << "  eps_before, eps_after, save_range_image_pngs,\n"
          << "  save_instance_mask_csv, frame_id,\n"
          << "  topic_cloud_{original,filtered,instance},\n"
          << "  topic_range_image, publish_rate_hz\n";
        rclcpp::shutdown();
        return 0;
      default:
        std::cerr << "Unrecognized option.\n";
        rclcpp::shutdown();
        return 1;
    }
  }

  auto node = std::make_shared<tree_mapping_geometry::BranchSegmenter>();
  node->apply_cli_overrides(file_num, x_deg, y_deg, d_thresh);
  node->run();
  rclcpp::shutdown();
  return 0;
}
