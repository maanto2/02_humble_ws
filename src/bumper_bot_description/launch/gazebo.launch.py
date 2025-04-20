from launch import launch_description
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        name = 'model',
        default_value= os.path.join( get_package_share_directory('bumper_bot_description'), 'urdf', 'bumperbot.urdf.xacro'),
        description='Absolute path to robot urdf file'

    )
    robot_description = ParameterValue( Command(["xacro ", LaunchConfiguration('model')]), value_type=str)
    
    

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description }])

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value= [str(Path(get_package_share_directory('bumper_bot_description')).parent.resolve())])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments= [ 
                ("gz_args", [" -v 4"," -r"," empty.sdf"])
                ] )
    
    gz_spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=["-topic", "robot_description",  
                       "-name", "bumperbot"],
            output='screen'
    )

    return launch_description.LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn
    ])