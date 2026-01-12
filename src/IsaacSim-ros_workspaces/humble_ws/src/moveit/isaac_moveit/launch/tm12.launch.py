import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type"
    )

    package_name = "isaac_moveit"
    pkg_share = get_package_share_directory(package_name)

    moveit_config = (
        MoveItConfigsBuilder("tm12", package_name=package_name)
        .robot_description(
            file_path=os.path.join(pkg_share, "tm_description", "xacro", "tm12.urdf.xacro"),
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(pkg_share, "tm_config", "tm12.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(pkg_share, "tm_config", "moveit_controllers.yaml")
        )
        .robot_description_kinematics(
            file_path=os.path.join(pkg_share, "tm_config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, "tm_config", "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"default_planning_pipeline": "ompl"},
        ],
        remappings=[("joint_states", "isaac_joint_states")],
        arguments=["--ros-args", "--log-level", "info"],
    )

    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "world",
            "base"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    pick_place_node = Node(
        package="isaac_moveit",
        executable="pick_and_place_tm12",
        name="pick_and_place_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("joint_states", "isaac_joint_states")],
    )

    return LaunchDescription([
        use_sim_time,
        ros2_control_hardware_type,
        move_group_node,
        robot_state_publisher,
        world2robot_tf_node,
        pick_place_node
    ])

