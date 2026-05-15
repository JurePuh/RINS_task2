from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task2',
            executable='detect_faces.main',
            name='detect_faces',
            output='screen',
        ),
        Node(
            package='task2',
            executable='detect_rings.main',
            name='detect_rings',
            output='screen',
        ),
        Node(
            package='task2',
            executable='speak.main',
            name='speak',
            output='screen',
        ),
        Node(
            package='task2',
            executable='classify_face.main',
            name='classify_face',
            output='screen',
        ),
    ])
