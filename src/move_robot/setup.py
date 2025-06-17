from setuptools import find_packages, setup
import os
import glob

package_name = 'move_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdul Shahzeb',
    maintainer_email='z5311131@ad.unsw.edu.au.com',
    description='ROS2 Package to control UR10e movement and Filament Extrusion using microROS',
    license='OpenSSL',
    entry_points={
        'console_scripts': [
            'move_ur = move_robot.move_ur:main',
            'calibrate_extruder = move_robot.calibrate_extruder:main',
        ],
    },
)
