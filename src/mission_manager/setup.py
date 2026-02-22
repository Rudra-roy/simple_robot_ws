from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'missions'), glob(os.path.join('missions', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahir',
    maintainer_email='mahir@todo.todo',
    description='Mission management and coordination for autonomous navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_coordinator_node = mission_manager.mission_coordinator_node:main',
            'pattern_generator_node = mission_manager.pattern_generator_node:main',
        ],
    },
)
