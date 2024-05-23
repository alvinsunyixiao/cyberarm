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
            executable="cybergear_node",
            package="cyber_arm",
            namespace="cybergear0",
            parameters=[{
                "can_id": 127,
                "mode": 1,
            }],
        ),
        Node(
            executable="cybergear_node",
            package="cyber_arm",
            namespace="cybergear1",
            parameters=[{
                "can_id": 126,
                "mode": 1,
                "position_kp": 10.0,
                "velocity_kp": 4.0,
                "velocity_ki": 0.16,
            }],
        ),
        Node(
            executable="cybergear_node",
            package="cyber_arm",
            namespace="cybergear2",
            parameters=[{
                "can_id": 125,
                "mode": 1,
                "position_kp": 15.0,
                "velocity_kp": 3.0,
                "velocity_ki": 0.06,
            }],
        ),
        Node(
            executable="cybergear_node",
            package="cyber_arm",
            namespace="cybergear3",
            parameters=[{
                "can_id": 124,
                "mode": 1,
            }],
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("cyber_arm"), "launch", "camera.launch.py"]),
            condition=IfCondition(camera),
        ),
    ])
