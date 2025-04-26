from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():


    # define launch arguments
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='False',
        description='Use Python launch file'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Wheel radius'
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Wheel separation'
    )


    #argument to choose own different controller or default controller
    use_simple_controller_arg = DeclareLaunchArgument(
        'use_simple_controller',
        default_value='True',
        description='Use simple controller'
    )

    # Declare the launch configurations
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_simple_controller = LaunchConfiguration('use_simple_controller')



    # Declare the launch arguments
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   "--controller-manager", '/controller_manager']
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions = [
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['simple_velocity_controller',
                        "--controller-manager", '/controller_manager']),

            Node(
                package='bumperbot_controller',
                executable='simple_controller.py',
                name='simple_controller_py',
                parameters=[{
                    'wheel_radius': wheel_radius,
                    'wheel_separation': wheel_separation
                }], condition = IfCondition(use_python), ),

            Node(
                package='bumperbot_controller',
                executable='simple_controller',
                name='simple_controller_cpp',
                parameters=[{
                    'wheel_radius': wheel_radius,
                    'wheel_separation': wheel_separation
                }], condition= UnlessCondition(use_python),), ])
  

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bumperbot_controller',
                   "--controller-manager", '/controller_manager'],
                   condition = UnlessCondition(use_simple_controller), )



    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,])