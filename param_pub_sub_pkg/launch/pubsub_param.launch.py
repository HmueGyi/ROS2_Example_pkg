from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='param_pub_sub_pkg',
            namespace='publisher',
            executable='param_talker',
            name='pub',
            parameters=[{"my_parameter" : 10}]

        ),
        Node(
            package='param_pub_sub_pkg',
            namespace='publisher',
            executable='param_listener',
            name='sub',
            parameters=[{"my_parameter1" : 10}]
        )
    ])