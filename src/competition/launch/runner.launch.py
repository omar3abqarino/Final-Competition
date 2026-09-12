import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    detection = Node(
        package='competition',
        executable='ScrollDetectionNode',
        name='ScrollDetectionNode',
        output= 'screen',
        parameters=[{'numOfdetections': 2,
                     'framesNeeded': 5,
                     'cooldown': 2.0
                     }]
    )

    controller = Node(
        package='competition',
        executable='pilotNode.py',
        name='pilotNode.py',
        output= 'screen',
        parameters=[{
            'linear_target': 0.2,
            'angular_target': 1.0,
        }]
    )

    return LaunchDescription([
        detection,
        controller
        
    ])
