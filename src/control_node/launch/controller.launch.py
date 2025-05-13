from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare optional launch arguments
    controller_params_arg = DeclareLaunchArgument(
        'controller_params_file',
        default_value='',
        description='Optional path to controller_node parameter file'
    )

    input_params_arg = DeclareLaunchArgument(
        'input_params_file',
        default_value='',
        description='Optional path to input_node parameter file'
    )

    return LaunchDescription([
        controller_params_arg,
        input_params_arg,

        # Launch controller_node with params only if provided
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[LaunchConfiguration('controller_params_file')],
        ),

        # Launch input_node with params only if provided
        Node(
            package='input_node',
            executable='input_node',
            name='input_node',
            output='screen',
            parameters=[LaunchConfiguration('input_params_file')],
        ),
    ])
