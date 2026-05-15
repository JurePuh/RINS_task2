from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task2',
            executable='speak.main',
            name='speak',
            output='screen',
        ),
        Node(
            package='task2',
            executable='map_query.main',
            name='map_query',
            output='screen',
        ),
        Node(
            package='task2',
            executable='movement.main',
            name='movement',
            output='screen',
        ),
    ])
