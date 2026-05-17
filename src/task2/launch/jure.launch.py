from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task2',
            executable='detect_faces',
            name='detect_faces',
            output='screen',
        ),
        Node(
            package='task2',
            executable='classify_face',
            name='classify_face',
            output='screen',
        ),
    ])
