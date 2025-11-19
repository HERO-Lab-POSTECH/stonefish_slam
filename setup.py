from setuptools import setup
from glob import glob
import os

package_name = 'stonefish_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.utils'],
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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Sonar-based SLAM for underwater robots using Stonefish simulator',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning_node = stonefish_slam.dead_reckoning_node:main_wrapper',
            'feature_extraction_node_sim = stonefish_slam.feature_extraction_node_sim:main_wrapper',
            'slam_node = stonefish_slam.slam_node:main_wrapper',
            'kalman_node = stonefish_slam.kalman_node:main_wrapper',
        ],
    },
)
