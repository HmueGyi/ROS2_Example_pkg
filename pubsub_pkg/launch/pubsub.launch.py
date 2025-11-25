from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_pkg',
            # namespace='turtlesim1',
            executable='talker',
            # name='sim1'

        ),
        Node(
            package='pubsub_pkg',
            # namespace='turtlesim2',
            executable='listener',
            # name='sim2'
        )
    ])