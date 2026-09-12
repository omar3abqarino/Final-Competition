from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_drivers",
            executable="calib",
            # parameters=[PathJoinSubstitution([
            #     FindPackageShare('camera_drivers'), 'python_stereo_camera_calibrate', 'calibration_settings.yaml'])
            # ],
        ),
    ])