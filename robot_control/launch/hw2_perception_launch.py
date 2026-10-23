"""HW2 perception: rectified camera + AprilTag detector + base_link -> camera_frame TF.

ros2 launch robot_control hw2_perception_launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Camera position in base_link (m), measured by hand.
    extrinsic_args = [
        DeclareLaunchArgument('cam_x', default_value='0.075'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.125'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_vision_camera'), 'launch', 'robot_vision_camera.launch.py'])),
        # apriltag uses only P from camera_info, so it needs the rectified image.
        launch_arguments={'image_rectify': 'true'}.items(),
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('robot_control'), 'config', 'hw2', 'apriltag.yaml'])],
    )

    # Level camera: optical axes (x right, y down, z forward) -> base (x fwd, y left, z up).
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_frame',
        ],
    )

    return LaunchDescription([*extrinsic_args, camera, apriltag, camera_tf])
