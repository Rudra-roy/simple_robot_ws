from setuptools import setup
import os
from glob import glob

package_name = 'simple_gps_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Point-to-point GPS navigation system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_navigation_node = simple_gps_nav.gps_navigation_node:main',
            'target_publisher_node = simple_gps_nav.target_publisher_node:main',
        ],
    },
)
