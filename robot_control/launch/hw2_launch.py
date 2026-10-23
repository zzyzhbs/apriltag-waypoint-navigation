"""HW2 full run: perception + motor_control + hw2_nav.

ros2 launch robot_control hw2_launch.py [floor:=study_wood2] [dry_run:=true] [tag_z_sign:=1]
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('floor', default_value='study_wood2'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('tag_z_sign', default_value='1'),
    ]
    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('robot_control'), 'launch', 'hw2_perception_launch.py'])))
    motor_node = Node(
        package='robot_control', executable='motor_control', name='motor_controller_node',
        output='screen', emulate_tty=True)
    nav_node = Node(
        package='robot_control', executable='hw2_nav', name='hw2_nav',
        output='screen', emulate_tty=True,
        parameters=[{
            'floor': LaunchConfiguration('floor'),
            'dry_run': LaunchConfiguration('dry_run'),
            'tag_z_sign': LaunchConfiguration('tag_z_sign'),
        }])
    # Bring everything down when the run ends or a node dies.
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[Shutdown(reason=f'{name} exited')]))
        for node, name in ((nav_node, 'hw2_nav'), (motor_node, 'motor_control'))
    ]
    return LaunchDescription([*args, perception, motor_node, nav_node, *shutdown_handlers])
