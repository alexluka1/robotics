import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('chaser')), '/twist_mux_launch.py'])
    )
    spin = Node(
        package='chaser',
        executable='spin',
        output='screen'
    )
    collision = Node(
        package='chaser',
        executable='collision',
        output='screen'
    )
    chaser = Node(
        package='chaser',
        executable='chaser',
        output='screen'
    )

    return LaunchDescription([
        twist_mux,
        spin,
        collision,
        chaser
    ])