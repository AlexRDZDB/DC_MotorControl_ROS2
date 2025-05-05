import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='input_node',  # Replace with your actual package name
            executable='input_node',       # This should match the entry point in setup.py
            name='input_node',
            output='screen',
            parameters=[
                {"signal_type": "step"},    # Default parameter values
                {"set_point": 10.0}
            ]
        )
    ])
