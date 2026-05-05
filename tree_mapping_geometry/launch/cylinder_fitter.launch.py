"""Launch a cylinder_fitter_node configured for the upstream test PCDs.

Examples:

    # Synthetic cylinder #0 (manual cloud).
    ros2 launch tree_mapping_geometry cylinder_fitter.launch.py mode:=manual file_num:=0

    # Real-tree fit on the upstream test set.
    ros2 launch tree_mapping_geometry cylinder_fitter.launch.py \\
        mode:=real \\
        file_num:=0 \\
        base_path:=/home/jdas/Downloads/TreeMapping-master/tree_mapping/pointclouds/test_cylinders_simulation/from_bags_live_2021-09-01_16-30-17/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode = LaunchConfiguration('mode')                # 'manual' | 'real' | 'pcd'
    file_num = LaunchConfiguration('file_num')
    branch_num = LaunchConfiguration('branch_num')
    base_path = LaunchConfiguration('base_path')
    frame_id = LaunchConfiguration('frame_id')
    cloud_topic = LaunchConfiguration('cloud_topic')
    marker_topic = LaunchConfiguration('marker_topic')
    max_iterations = LaunchConfiguration('max_iterations')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    invert_alpha = LaunchConfiguration('invert_alpha')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='manual',
            description="One of 'manual', 'real', or 'pcd'."),
        DeclareLaunchArgument('file_num', default_value='0'),
        DeclareLaunchArgument('branch_num', default_value='0'),
        DeclareLaunchArgument('base_path', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('cloud_topic',
            default_value='/tree_mapping/cylinder_pointcloud'),
        DeclareLaunchArgument('marker_topic',
            default_value='/tree_mapping/cylinder_markers'),
        DeclareLaunchArgument('max_iterations', default_value='500'),
        DeclareLaunchArgument('publish_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('invert_alpha', default_value='false'),
        Node(
            package='tree_mapping_geometry',
            executable='cylinder_fitter_node',
            name='cylinder_fitter',
            output='screen',
            parameters=[{
                'base_path': ParameterValue(base_path, value_type=str),
                'file_num': ParameterValue(file_num, value_type=str),
                'branch_num': ParameterValue(branch_num, value_type=str),
                'use_manual_cloud': ParameterValue(
                    PythonExpression(["'", mode, "' == 'manual'"]),
                    value_type=bool),
                'use_real_cloud': ParameterValue(
                    PythonExpression(["'", mode, "' == 'real'"]),
                    value_type=bool),
                'invert_alpha': ParameterValue(invert_alpha, value_type=bool),
                'frame_id': ParameterValue(frame_id, value_type=str),
                'cloud_topic': ParameterValue(cloud_topic, value_type=str),
                'marker_topic': ParameterValue(marker_topic, value_type=str),
                'max_iterations': ParameterValue(max_iterations, value_type=int),
                'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
            }],
        ),
    ])
