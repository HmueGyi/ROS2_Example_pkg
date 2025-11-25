from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First turtlesim instance
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim1'
        ),

        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim2'
        ),

        # Your custom draw_square node controlling turtlesim2
        Node(
            package='homework_pkg',
            namespace='turtlesim2',
            executable='draw_square',
            name='square'
        ),

        Node(
            package='homework_pkg',
            namespace='turtlesim1',
            executable='draw_cyc',
            name='cycle'
        ),

        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])
