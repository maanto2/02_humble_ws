from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # define launch arguments



    # define launch configurations

    # joy is the node to read from the joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[os.path.join(get_package_share_directory('bumperbot_controller'), 'config', 'joy_config.yaml')],)

    # joy_teleop is the node to map from joy -- to cmd_vel
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        parameters=[os.path.join(get_package_share_directory('bumperbot_controller'), 'config', 'joy_teleop.yaml')],
    )


    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])
    
