from setuptools import setup
from glob import glob
import os

package_name = 'stonefish_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        package_name + '.nodes',
        package_name + '.core',
        package_name + '.sensors',
        package_name + '.utils',
        package_name + '.cpp',
    ],
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
            'dead_reckoning_node = stonefish_slam.nodes.dead_reckoning_node:main_wrapper',
            'feature_extraction_node_sim = stonefish_slam.nodes.feature_extraction_node_sim:main_wrapper',
            'slam_node = stonefish_slam.nodes.slam_node:main_wrapper',
            'kalman_node = stonefish_slam.nodes.kalman_node:main_wrapper',
        ],
    },
)
