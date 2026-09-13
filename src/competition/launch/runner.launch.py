
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    detection = Node(
        package='competition',
        executable='detection',
        name='detection',
        output= 'screen',
        parameters=[{'numOfdetections': 2,
                    'framesNeeded': 5,
                    'cooldown': 2.0
                    }]
    )

    controller = Node(
        package='competition',
        executable='pilot',
        name='pilot',
        output= 'screen',
        parameters=[{
            'linear_target': 0.2,
            'angular_target': 1.0,
        }],
        emulate_tty=True,   # <-- CRITICAL: Simulates a real terminal interface
        prefix='xterm -e',
    )

    return LaunchDescription([
        detection,
        controller 
    ])
