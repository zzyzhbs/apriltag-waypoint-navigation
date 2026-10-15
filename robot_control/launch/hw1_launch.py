from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    floor = DeclareLaunchArgument('floor', description='calibration file, e.g. wood')
    motor_node = Node(
        package='robot_control',
        executable='motor_control',
        name='motor_controller_node',
        output='screen',
        emulate_tty=True,
    )
    velocity_node = Node(
        package='robot_control',
        executable='velocity_control',
        name='velocity_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'floor': LaunchConfiguration('floor')}],
    )
    # Bring everything down when either node exits.
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[Shutdown(reason=f'{name} exited')]))
        for node, name in ((velocity_node, 'velocity_control'), (motor_node, 'motor_control'))
    ]
    return LaunchDescription([floor, motor_node, velocity_node, *shutdown_handlers])
