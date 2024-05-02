from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_launch_package = FindPackageShare('urdf_launch')

    return LaunchDescription([
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
    ])
