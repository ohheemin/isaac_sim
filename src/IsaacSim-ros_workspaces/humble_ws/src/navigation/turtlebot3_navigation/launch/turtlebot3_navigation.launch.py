# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    
    # Package Directory
    pkg_nav = get_package_share_directory("turtlebot3_navigation")
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    # Configuration Paths
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(pkg_nav, "maps", "turtlebot3_warehouse.yaml"),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(pkg_nav, "params", "turtlebot3_nav2_params.yaml"),
    )

    rviz_config_dir = os.path.join(pkg_nav, "rviz2", "turtlebot3_nav2.rviz")

    return LaunchDescription(
        [
            # 1. Launch Arguments Declaration
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument("params_file", default_value=param_dir, description="Full path to param file to load"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Omniverse Isaac Sim) clock if true"),
            
            # 2. RViz Launcher (Visualization)
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_dir], 
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            
            # 3. NAV2 Bringup (AMCL, Planner, Controller, BT)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={
                    "map": map_dir, 
                    "use_sim_time": use_sim_time, 
                    "params_file": param_dir,
                    "autostart": "True"
                }.items(),
            ),

            # 4. Pointcloud to Laserscan Node
            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', ['/turtlebot3_burger/point_cloud']),
                    ('scan', ['/scan'])
                ],
                parameters=[{
                    'target_frame': 'base_scan',
                    'transform_tolerance': 0.01,
                    'min_height': -0.1, 
                    'max_height': 0.3, 
                    'angle_min': -3.14159, 
                    'angle_max': 3.14159, 
                    'angle_increment': 0.0087,
                    'scan_time': 0.3333,
                    'range_min': 0.05,
                    'range_max': 5.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0,
                }],
                name='pointcloud_to_laserscan_node'
            )
        ]
    )

