import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'competition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'opencv-python', 'torch', ],
    zip_safe=True,
    maintainer='a',
    maintainer_email='omar3abqarino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "pilot = competition.pilotNode:main",
            "detection = competition.ScrollDetectionNode:main",
        ],
    },
)
