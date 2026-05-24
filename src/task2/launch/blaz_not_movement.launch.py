from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task2',
            executable='speak',
            name='speak',
            output='screen',
        ),
        Node(
            package='task2',
            executable='map_query',
            name='map_query',
            output='screen',
        ),
        Node(
            package='task2',
            executable='line_fit',
            name='line_fit',
            output='screen',
        ),
        Node(
            package='task2',
            executable='blue_line',
            name='blue_line',
            output='screen',
        ),
    ])
