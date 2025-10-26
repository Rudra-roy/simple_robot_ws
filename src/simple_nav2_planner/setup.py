from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simple_nav2_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kage',
    maintainer_email='hironmoy.roy.rudra@g.bracu.ac.bd',
    description='Nav2 planner wrapper using custom costmap',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_planner_wrapper = simple_nav2_planner.nav2_planner_wrapper:main',
            'static_tf_publisher = simple_nav2_planner.static_tf_publisher:main',
        ],
    },
)
