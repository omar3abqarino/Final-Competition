from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_kinematics_pkg',
            executable='mecanum_invers', 
            output='screen'
        ),
       Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),
    ])
