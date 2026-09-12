from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_kinematics_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My robot kinematics package',
    license='Apache License 2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # kinematics1 (ROS node with omni kinematics)
            'kinematics = robot_kinematics_pkg.kinematics1.kinematics:main',

            # scripts - diff (4 wheel)
            'diff_forward = robot_kinematics_pkg.scripts.diff.diff_forward:main',
            'diff_invers = robot_kinematics_pkg.scripts.diff.diff_invers:main',

            # scripts - diff_2wheels
            'diff_2wheels_forward = robot_kinematics_pkg.scripts.diff_2wheels.diff_2wheels_forward:main',
            'diff_2wheels_invers = robot_kinematics_pkg.scripts.diff_2wheels.diff_2wheels_invers:main',

            # scripts - mecanum
            'mecanum_forward = robot_kinematics_pkg.scripts.mecanum.mecanum_forward:main',
            'mecanum_invers = robot_kinematics_pkg.scripts.mecanum.mecanum_invers:main',

            # scripts - omni
            'omni3_forward = robot_kinematics_pkg.scripts.omni.omni3_forward:main',
            'omni3_invers = robot_kinematics_pkg.scripts.omni.omni3_invers:main',
            'omni4_forward = robot_kinematics_pkg.scripts.omni.omni4_forward:main',
            'omni4_invers = robot_kinematics_pkg.scripts.omni.omni4_invers:main',
            'omni6_invers = robot_kinematics_pkg.scripts.omni.omni6_inverse:main',

            # scripts - r1
            'diff_forward_r1 = robot_kinematics_pkg.scripts.diff.diff_forward_r1:main',
            'diff_invers_r1 = robot_kinematics_pkg.scripts.diff.diff_invers_r1:main',
            'diff_invers_new_r1 = robot_kinematics_pkg.scripts.diff.diff_invers_new_r1:main',

            # scripts - r2
            'mecanum_invers_r2 = robot_kinematics_pkg.scripts.mecanum.mecanum_invers_r2:main',
        ],
    },
)

