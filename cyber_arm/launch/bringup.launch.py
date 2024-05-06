from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = LaunchConfiguration("camera", default="false")

    urdf_launch_package = FindPackageShare('urdf_launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera",
            default_value=camera,
            choices=["true", "false"],
            description="set to true to bringup D435i camera",
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
            launch_arguments={
                'urdf_package': 'cyber_arm',
                'urdf_package_path': 'models/model.urdf'}.items()
        ),
        Node(
            executable="cyberarm_node",
            package="cyber_arm",
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("cyber_arm"), "launch", "camera.launch.py"]),
            condition=IfCondition(camera),
        ),
    ])
