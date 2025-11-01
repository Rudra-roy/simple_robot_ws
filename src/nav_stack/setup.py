from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mt',
    maintainer_email='hironmoy.roy.rudra@g.bracu.ac.bd',
    description='Professional Navigation Stack with Global and Local Planners',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_planner_node = nav_stack.global_planner_node:main',
        ],
    },
)
