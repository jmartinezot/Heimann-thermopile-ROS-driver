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
            output='screen'
        ),
    ])
