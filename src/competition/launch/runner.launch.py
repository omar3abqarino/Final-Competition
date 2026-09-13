
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    detection = Node(
        package='competition',
        executable='detection',
        name='ScrollDetectionNode',
        output= 'screen',
        parameters=[{'numOfdetections': 2,
                     'framesNeeded': 5,
                     'cooldown': 2.0
                     }]
    )

    controller = Node(
        package='competition',
        executable='pilot',
        name='pilotNode',
        output= 'screen',
        parameters=[{
            'linear_target': 0.2,
            'angular_target': 1.0,
        }]
    )

    ultrasonic_node = Node(
        package= 'competition',
        executable='ultrasonic',
        name='ultrasonic_node',
        output= 'screen',
        parameters=[{
            'input_vel_topic': '/cmd_vel_requested',
            'output_vel_topic': '/cmd_vel',
            'ultrasonic_topic': '/ultrasonic_distance',
        }]

    )
    return LaunchDescription([
        detection,
        controller,
        ultrasonic_node
    ])
