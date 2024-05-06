from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]),
            launch_arguments={
                "enable_rgbd": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "enable_color": "true",
                "enable_depth": "true",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "-0.056",
                "--y", "0",
                "--z", "0.10095",
                "--roll", "0",
                "--pitch", "-1.5708",
                "--yaw", "1.5708",
                "--frame-id", "end_effector_link",
                "--child-frame-id", "camera_link",
            ],
        ),
    ])
