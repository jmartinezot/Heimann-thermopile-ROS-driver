# htpa_launch.py

import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    zoom = LaunchConfiguration('zoom').perform(context)
    lower_limit = LaunchConfiguration('lower_limit').perform(context)
    upper_limit = LaunchConfiguration('upper_limit').perform(context)

    return [
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
            parameters=[{
              'zoom': LaunchConfiguration('zoom'),
              'lower_limit': LaunchConfiguration('lower_limit'),
              'upper_limit': LaunchConfiguration('upper_limit'),
            }]
        ),
        Node(
            package='heiman',
            executable='showHTPAimage',
            name='showHTPAimage',
            output='screen'
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'zoom', default_value='20'
        ),
        DeclareLaunchArgument(
            'lower_limit', default_value='25'
        ),
        DeclareLaunchArgument(
            'upper_limit', default_value='35'
        ),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    args = sys.argv[1:]
    zoom = args[args.index('--zoom')+1] if '--zoom' in args else '20'
    lower_limit = args[args.index('--lower_limit')+1] if '--lower_limit' in args else '25'
    upper_limit = args[args.index('--upper_limit')+1] if '--upper_limit' in args else '35'

    generate_launch_description(zoom, lower_limit, upper_limit)