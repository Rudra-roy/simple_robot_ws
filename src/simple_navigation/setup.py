from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'simple_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/simple_navigation']),
        ('share/simple_navigation', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Simple navigation package with custom costmap generation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator=simple_navigation.waypoint_navigator:main',
            'adaptive_planner=simple_navigation.adaptive_planner_node:main',
        ],
    },
)
