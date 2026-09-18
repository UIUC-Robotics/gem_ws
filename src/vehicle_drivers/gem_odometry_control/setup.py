from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gem_odometry_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gem',
    maintainer_email='gem@todo.todo',
    description='Wheel+IMU+GPS fused odometry and odometry-based pure pursuit for GEM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry = gem_odometry_control.wheel_odometry_node:main',
            'pure_pursuit_odom = gem_odometry_control.pure_pursuit_odom:main',
            'record_waypoints_odom = gem_odometry_control.record_waypoints_odom:main',
        ],
    },
)
