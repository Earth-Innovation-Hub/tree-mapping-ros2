#!/usr/bin/env python3
"""Launch the tree_pointcloud_viz loader and (optionally) RViz.

Examples
--------
    ros2 launch tree_pointcloud_viz tree_pointcloud_viz.launch.py \
        input_dir:=/data/wur_trees \
        file_list:='["GUY01_000.txt","GUY02_000.txt","GUY03_000.txt"]'

    ros2 launch tree_pointcloud_viz tree_pointcloud_viz.launch.py \
        rviz:=false topic:=/tree_dataset/cloud
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('tree_pointcloud_viz')
    default_rviz = os.path.join(pkg_share, 'rviz', 'tree_pointcloud_viz.rviz')

    args = [
        DeclareLaunchArgument(
            'input_dir', default_value='',
            description='Directory holding the XYZ tree text files.'),
        DeclareLaunchArgument(
            'file_list', default_value='[]',
            description='YAML list of file names (relative to input_dir). '
                        "Example: '[\"GUY01_000.txt\",\"GUY02_000.txt\"]'."),
        DeclareLaunchArgument(
            'frame_id', default_value='origin_tree',
            description='frame_id stamped on the published cloud.'),
        DeclareLaunchArgument(
            'topic', default_value='~/cloud',
            description='Topic to publish on.'),
        DeclareLaunchArgument(
            'max_points_per_file', default_value='2000000'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Also start RViz with a tree-mapping config.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz config file.'),
    ]

    viz_node = Node(
        package='tree_pointcloud_viz',
        executable='tree_pointcloud_viz_node',
        name='tree_pointcloud_viz_node',
        output='screen',
        parameters=[{
            'input_dir': LaunchConfiguration('input_dir'),
            'file_list': LaunchConfiguration('file_list'),
            'frame_id': LaunchConfiguration('frame_id'),
            'topic': LaunchConfiguration('topic'),
            'max_points_per_file': LaunchConfiguration('max_points_per_file'),
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='origin_tree_static_tf',
        output='log',
        arguments=['1', '0', '0', '0', '0', '0', '1',
                   LaunchConfiguration('frame_id'), 'child_frame1'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='tree_pointcloud_rviz',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [viz_node, static_tf, rviz])
