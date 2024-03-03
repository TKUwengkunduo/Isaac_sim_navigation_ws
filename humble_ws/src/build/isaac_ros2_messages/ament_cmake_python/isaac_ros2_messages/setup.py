from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros2_messages',
    version='0.1.0',
    packages=find_packages(
        include=('isaac_ros2_messages', 'isaac_ros2_messages.*')),
)
