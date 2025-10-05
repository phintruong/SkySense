from setuptools import setup
import os
from glob import glob

package_name = 'obstacle_avoidance_lidar'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='LiDAR-based obstacle detection and avoidance for quadcopters',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_adapter = obstacle_avoidance_lidar.frame_adapter:main',
            'avoidance_node = obstacle_avoidance_lidar.avoidance_node:main',
            'fake_scan_publisher = obstacle_avoidance_lidar.fake_scan_publisher:main',
            'plot_histogram = obstacle_avoidance_lidar.plot_histogram:main',
        ],
    },
)

