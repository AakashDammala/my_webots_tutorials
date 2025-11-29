import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the paths
    pkg_share = FindPackageShare(
        "my_webots_tutorials").find("my_webots_tutorials")
    webots_ros2_epuck_share = FindPackageShare(
        "webots_ros2_epuck").find("webots_ros2_epuck")

    # Declare launch arguments
    use_rviz = LaunchConfiguration("rviz")
    use_rosbag = LaunchConfiguration("rosbag")

    declare_use_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz visualization"
    )

    declare_use_rosbag = DeclareLaunchArgument(
        "rosbag",
        default_value="false",
        description="Enable rosbag recording (excludes /camera/* topics)"
    )

    # Launch Webots simulator with ePuck
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [webots_ros2_epuck_share, "launch", "robot_launch.py"])
        ),
        launch_arguments={"rviz": use_rviz}.items()
    )

    walker_node = Node(
        package="my_webots_tutorials",
        executable="walker",
        name="walker",
        output="screen",
    )

    # Rosbag recording (if enabled)
    # Records all topics except /camera/* to avoid excessive disk usage
    # Saves to results/ directory (created relative to launch execution directory)
    rosbag_record = ExecuteProcess(
        condition=launch.conditions.IfCondition(use_rosbag),
        cmd=['ros2', 'bag', 'record', '-x',
             '/camera/.*', '-a', '-o', 'results'],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declare arguments
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_rosbag)

    # Add Webots launch
    ld.add_action(webots_launch)

    # Add walker node
    ld.add_action(walker_node)

    # Add rosbag recording
    ld.add_action(rosbag_record)

    return ld
