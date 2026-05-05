"""Launch the branch segmenter on a single per-tree PCD.

Examples
--------
ros2 launch tree_mapping_geometry branch_segmenter.launch.py \
    base_path:=$HOME/Downloads/TreeMapping-master/tree_mapping/pointclouds/pcd_xyzir/ \
    file_num:=0 d_thresh:=0.33

ros2 launch tree_mapping_geometry branch_segmenter.launch.py \
    base_path:=/path/to/clouds/ output_dir:=/tmp/branch_seg/ \
    file_num:=0 \
    base_xyz:='[-15.94, 20.45, 0.45]' eps_before:=2.0 eps_after:=7.0 \
    angular_resolution_x_deg:=0.1 angular_resolution_y_deg:=0.1 d_thresh:=0.33
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "base_path", default_value="",
            description="Directory containing 'cloudN_N.000000.pcd' files."),
        DeclareLaunchArgument(
            "output_dir", default_value="",
            description="Where to write range image PNGs and instance "
                        "mask CSVs (defaults to base_path)."),
        DeclareLaunchArgument(
            "file_num", default_value="0",
            description="Cloud index used in the filename and as a key "
                        "into the upstream test-set defaults."),
        DeclareLaunchArgument(
            "angular_resolution_x_deg", default_value="0.1"),
        DeclareLaunchArgument(
            "angular_resolution_y_deg", default_value="0.1"),
        DeclareLaunchArgument(
            "d_thresh", default_value="0.33",
            description="Pair-wise pixel distance threshold (m) for "
                        "splitting branches in the range image row."),
        DeclareLaunchArgument(
            "base_xyz", default_value="[]",
            description="Tree base (x, y, z) in lidar frame; empty list "
                        "falls back to upstream defaults for file_num."),
        DeclareLaunchArgument("eps_before", default_value="-1.0"),
        DeclareLaunchArgument("eps_after",  default_value="-1.0"),
        DeclareLaunchArgument("save_range_image_pngs",
                              default_value="true"),
        DeclareLaunchArgument("save_instance_mask_csv",
                              default_value="true"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("publish_rate_hz", default_value="1.0"),
    ]

    parameters = [{
        "base_path": ParameterValue(
            LaunchConfiguration("base_path"), value_type=str),
        "output_dir": ParameterValue(
            LaunchConfiguration("output_dir"), value_type=str),
        "file_num": ParameterValue(
            LaunchConfiguration("file_num"), value_type=str),
        "angular_resolution_x_deg": ParameterValue(
            LaunchConfiguration("angular_resolution_x_deg"),
            value_type=float),
        "angular_resolution_y_deg": ParameterValue(
            LaunchConfiguration("angular_resolution_y_deg"),
            value_type=float),
        "d_thresh": ParameterValue(
            LaunchConfiguration("d_thresh"), value_type=float),
        "base_xyz": ParameterValue(
            LaunchConfiguration("base_xyz"),
            value_type=list[float]),
        "eps_before": ParameterValue(
            LaunchConfiguration("eps_before"), value_type=float),
        "eps_after": ParameterValue(
            LaunchConfiguration("eps_after"), value_type=float),
        "save_range_image_pngs": ParameterValue(
            LaunchConfiguration("save_range_image_pngs"), value_type=bool),
        "save_instance_mask_csv": ParameterValue(
            LaunchConfiguration("save_instance_mask_csv"), value_type=bool),
        "frame_id": ParameterValue(
            LaunchConfiguration("frame_id"), value_type=str),
        "publish_rate_hz": ParameterValue(
            LaunchConfiguration("publish_rate_hz"), value_type=float),
    }]

    return LaunchDescription([
        *declared,
        Node(
            package="tree_mapping_geometry",
            executable="branch_segmenter_node",
            name="branch_segmenter",
            output="screen",
            parameters=parameters,
        ),
    ])
