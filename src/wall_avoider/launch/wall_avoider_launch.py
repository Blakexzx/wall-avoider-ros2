# Wall avoider launch - used to launch everything together.
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    epuck_pkg_share = get_package_share_directory('webots_ros2_epuck')

    epuck_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(epuck_pkg_share, 'launch', 'robot_launch.py')
        ),
        launch_arguments={}.items(),
    )

    wall_avoider_node = Node(
        package='wall_avoider',
        executable='wall_avoider_node',
        name='wall_avoider',
        output='screen',
        parameters=[{
            'left_sensor_topic': '/ps6',
            'right_sensor_topic': '/ps7',
            'cmd_vel_topic': '/cmd_vel',
            'threshold': 0.05,
            'escape_threshold': 0.03,
            'forward_speed': 0.08,
            'turn_speed': 2.0,
            'control_period': 0.064,
        }]
    )

    return LaunchDescription([
        epuck_launch,
        wall_avoider_node,
    ])