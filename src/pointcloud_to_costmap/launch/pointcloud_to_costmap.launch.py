#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数文件路径
    default_param_file = PathJoinSubstitution([
        FindPackageShare('pointcloud_to_costmap'),
        'param',
        'param.yaml'
    ])
    
    # Launch参数声明
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to parameter file'
    )
    
    # 点云转地图节点
    pointcloud_to_costmap_node = Node(
        package='pointcloud_to_costmap',
        executable='pointcloud_to_costmap_node',
        name='pointcloud_to_costmap_node',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
        remappings=[]
    )
    
    return LaunchDescription([
        param_file_arg,
        pointcloud_to_costmap_node
    ])
