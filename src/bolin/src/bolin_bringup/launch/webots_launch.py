from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    webots_launch_file_dir = FindPackageShare(package="webots_ros2_turtlebot").find(
        "webots_ros2_turtlebot"
    )
    webots_launch_file_path = os.path.join(
        webots_launch_file_dir, "launch", "robot_launch.py"
    )

    return LaunchDescription(
        [
            Node(
                package="bolin_camera",
                executable="camera",
                name="camera",
                output="screen",
            ),
            Node(
                package="bolin_lidar",
                executable="lidar",
                name="lidar",
            ),
            Node(
                package="bolin_teleop",
                executable="teleop",
                name="teleop",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(webots_launch_file_path)
            ),
        ]
    )