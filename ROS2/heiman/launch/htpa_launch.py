# htpa_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='heiman',
            executable='controlandpublishfromHTPA',
            name='controlandpublishfromHTPA',
            output='screen'
        ),
        Node(
            package='heiman',
            executable='convertimagefromHTPApublished',
            name='convertimagefromHTPApublished',
            output='screen',
            parameters=[
                {'zoom': 20},
                {'lower_limit': 25},
                {'upper_limit': 35}
            ]
        ),
        Node(
            package='heiman',
            executable='showHTPAimage',
            name='showHTPAimage',
            output='screen'
        ),
    ])

