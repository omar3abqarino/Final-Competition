from setuptools import setup
import os
from glob import glob

package_name = 'camera_drivers'
package_mono = 'camera_drivers.examples.mono'
package_stereo = 'camera_drivers.examples.stereo'
package_kinect_v1 = 'camera_drivers.examples.kinect_v1'
package_kinect_v2 = 'camera_drivers.examples.kinect_v2'
calib = 'camera_drivers.python_stereo_camera_calibrate'




setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_mono, package_stereo, package_kinect_v1, package_kinect_v2, calib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'),
        # glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sherbiny',
    maintainer_email='elsherbiny2023@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mono_publisher =camera_drivers.examples.mono.mono_publisher:main',
            'mono_subscriber =camera_drivers.examples.mono.mono_subscriber:main',
            'mono_getter =camera_drivers.examples.mono.mono_getter:main',
            'mono_client =camera_drivers.examples.mono.mono_client:main',
            'mono_server =camera_drivers.examples.mono.mono_server:main',
    
            'stereo_publisher =camera_drivers.examples.stereo.stereo_publisher:main',
            'stereo_getter =camera_drivers.examples.stereo.stereo_getter:main',
            'stereo_subscriber =camera_drivers.examples.stereo.stereo_subscriber:main',
            'stereo_server =camera_drivers.examples.stereo.stereo_server:main',
            'stereo_client =camera_drivers.examples.stereo.stereo_client:main',

            'kinect_v1_subscriber =camera_drivers.examples.kinect_v1.kinect_v1_subscriber:main',
            'kinect_v1_subscriber2 =camera_drivers.examples.kinect_v1.kinect_v1_subscriber2:main',

            'index_tester =camera_drivers.camera_index_tester:main',

            'kinect_v2_subscriber =camera_drivers.examples.kinect_v2.kinect_v2_subscriber:main',
            'kinect_v2_subscriber2 =camera_drivers.examples.kinect_v2.kinect_v2_subscriber2:main',
            

        ],
    },
)
