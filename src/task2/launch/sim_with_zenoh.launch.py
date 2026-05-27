from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Available worlds (from dis_tutorial3/worlds):
#   task2_blue_demo, task2_green_demo, task2_yellow_demo,
WORLD = 'task2'


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory('dis_tutorial7'),
        'launch',
        'sim_turtlebot_nav.launch.py',
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments=[('world', WORLD)],
        ),
    ])
