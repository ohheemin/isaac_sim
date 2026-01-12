# SPDX-FileCopyrightText: Copyright (c) 2025
# SPDX-License-Identifier: Apache-2.0

"""
Multi-TurtleBot3 Navigation Launch File

- Spawns two TurtleBot3 robots into a shared environment
- Each robot runs its own Nav2 stack, RViz, and pointcloud_to_laserscan node
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory("turtlebot3_navigation")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    rviz_config_dir = os.path.join(pkg_nav, "rviz2", "turtlebot3_nav2.rviz")

    robots = [
        {"name": "turtlebot3_burger_1"},
        {"name": "turtlebot3_burger_2"},
    ]

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    autostart = LaunchConfiguration("autostart", default="True")
    use_rviz = LaunchConfiguration("use_rviz", default="True")
    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(pkg_nav, "maps", "turtlebot3_warehouse_multiple_shelves.yaml"),
    )
    rviz_config_file = LaunchConfiguration("rviz_config", default=rviz_config_dir)

    declare_map_cmd = DeclareLaunchArgument(
        "map", default_value=map_yaml_file, description="Full path to map YAML file"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation clock"
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Autostart Nav2 stack"
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Launch RViz for each robot"
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_file,
        description="Full path to the RViz config file to use",
    )

    nav_instances_cmds = []
    for robot in robots:
        robot_name = robot["name"]

        params_arg_name = robot_name + "_params_file" # 동적 인자 이름
        declare_params_file_cmd = DeclareLaunchArgument(
            params_arg_name,
            default_value=os.path.join(
                pkg_nav,
                "params",
                f"turtlebot3_nav2_params_{robot_name[-1]}.yaml"
            ),
            description=f"Full path to params file for {robot_name}",
        )

        params_file = LaunchConfiguration(params_arg_name)

        group = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_launch_dir, "rviz_launch.py")
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot_name),
                        "use_namespace": "True",
                        "rviz_config": TextSubstitution(
                            text=os.path.join(
                                pkg_nav,
                                "rviz2",
                                f"turtlebot3_nav2_{robot_name[-1]}.rviz"
                            )
                        ),
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_launch_dir, "bringup_launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot_name,
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": use_sim_time,
                        "params_file": params_file,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),

                LogInfo(msg=[f"Launching {robot_name} with namespace {robot_name}"]),
                LogInfo(msg=[f"{robot_name} using map: ", map_yaml_file]),
                LogInfo(msg=[f"{robot_name} params file: ", params_file]), 
            ]
        )

        nav_instances_cmds.append(declare_params_file_cmd)
        nav_instances_cmds.append(group)

    ld = LaunchDescription()

    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)

    for cmd in nav_instances_cmds:
        ld.add_action(cmd)

    return ld
