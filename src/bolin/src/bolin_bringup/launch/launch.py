from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    turtlebot_launch_file_dir = FindPackageShare(package="turtlebot3_bringup").find(
        "turtlebot3_bringup"
    )
    turtlebot_launch_file_path = os.path.join(
        turtlebot_launch_file_dir, "launch", "robot.launch.py"
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
            ),
            Node(
                package="bolin_lidar",
                executable="lidar",
                name="lidar",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot_launch_file_path)
            ),
        ]
    )
