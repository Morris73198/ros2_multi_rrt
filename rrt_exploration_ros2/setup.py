from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rrt_exploration_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Morris',
    maintainer_email='your@email.com',
    description='RRT-based multi-robot exploration for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boundary_node = rrt_exploration_ros2.boundary_node:main',
            'global_detector_node = rrt_exploration_ros2.boundary_node:main',
        ],
    },
)
