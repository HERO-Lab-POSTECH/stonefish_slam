from setuptools import find_packages
from setuptools import setup

setup(
    name='stonefish_slam',
    version='0.1.0',
    packages=find_packages(
        include=('stonefish_slam', 'stonefish_slam.*')),
)
