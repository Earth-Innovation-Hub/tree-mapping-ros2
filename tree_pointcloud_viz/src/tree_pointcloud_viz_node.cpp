// SPDX-License-Identifier: MIT
//
// ROS 2 port of DREAMS-lab tree-mapping/tree_pointcloud_viz/src/testPcl.cpp.
//
// Loads one or more XYZ point-cloud text files (whitespace-separated
// "x y z" per line, e.g. the WUR tropical-tree terrestrial-lidar set
// http://lucid.wur.nl/datasets/terrestrial-lidar-of-tropical-forests),
// merges them, and publishes a single sensor_msgs/PointCloud2 on
// ``~/cloud`` with TRANSIENT_LOCAL durability so RViz subscribers attach
// late and still see the cloud.
//
// Improvements over the upstream ROS 1 version:
//   * Configurable ``input_dir`` and ``file_list`` via ROS 2 parameters
//     (no hardcoded /root/catkin_ws/... path).
//   * Configurable ``frame_id``, ``topic``, ``max_points_per_file``,
//     and progress logging cadence.
//   * Modern PCL / pcl_conversions + sensor_msgs::msg::PointCloud2.
//   * Uses an rclcpp::Node subclass and a normal spin loop (no busy waits).

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


namespace tree_pointcloud_viz
{

class TreePointCloudVizNode : public rclcpp::Node
{
public:
  TreePointCloudVizNode()
  : rclcpp::Node("tree_pointcloud_viz_node")
  {
    declare_parameter<std::string>("input_dir", "");
    declare_parameter<std::vector<std::string>>(
      "file_list", std::vector<std::string>{});
    declare_parameter<std::string>("frame_id", "origin_tree");
    declare_parameter<std::string>("topic", "~/cloud");
    declare_parameter<int>("max_points_per_file", 2'000'000);
    declare_parameter<int>("progress_log_steps", 100);

    const std::string input_dir = get_parameter("input_dir").as_string();
    const std::vector<std::string> file_list =
      get_parameter("file_list").as_string_array();
    const std::string frame_id = get_parameter("frame_id").as_string();
    const std::string topic = get_parameter("topic").as_string();
    const std::int64_t max_points =
      static_cast<std::int64_t>(get_parameter("max_points_per_file").as_int());
    const int progress_log_steps =
      static_cast<int>(get_parameter("progress_log_steps").as_int());

    if (file_list.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter ``file_list`` is empty -- nothing to load. "
        "Pass file_list:='[\"GUY01_000.txt\", ...]' on launch.");
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(file_list.size() * static_cast<std::size_t>(
        std::max<std::int64_t>(max_points, 1)));

    for (const auto & file_name : file_list) {
      const std::filesystem::path full_path =
        input_dir.empty() ? std::filesystem::path(file_name)
                          : std::filesystem::path(input_dir) / file_name;
      RCLCPP_INFO(get_logger(), "Adding tree: %s", full_path.c_str());

      std::ifstream fin(full_path);
      if (!fin) {
        RCLCPP_ERROR(
          get_logger(), "Failed to open '%s'; skipping.", full_path.c_str());
        continue;
      }

      const std::int64_t log_every =
        std::max<std::int64_t>(max_points / std::max(progress_log_steps, 1), 1);

      std::int64_t loaded = 0;
      pcl::PointXYZ point;
      while (loaded < max_points && (fin >> point.x >> point.y >> point.z)) {
        if ((loaded % log_every) == 0) {
          RCLCPP_DEBUG(get_logger(), "  loaded %ld points", loaded);
        }
        cloud->push_back(point);
        ++loaded;
      }
      RCLCPP_INFO(
        get_logger(), "  -> %ld points from %s",
        loaded, full_path.filename().c_str());
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    pcl::toROSMsg(*cloud, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id;
    cloud_msg_.header.stamp = now();

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .transient_local()
      .reliable();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);

    // Re-publish on a slow timer so subscribers that connect later (and
    // for some reason do not get the latched message) still receive a
    // periodic refresh. The TRANSIENT_LOCAL QoS handles the common case.
    publish_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        cloud_msg_.header.stamp = now();
        publisher_->publish(cloud_msg_);
      });

    RCLCPP_INFO(
      get_logger(),
      "Published %zu points on '%s' (frame=%s).",
      cloud->size(), topic.c_str(), frame_id.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

}  // namespace tree_pointcloud_viz


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tree_pointcloud_viz::TreePointCloudVizNode>());
  rclcpp::shutdown();
  return 0;
}
