from setuptools import setup
import os
from glob import glob

package_name = 'mt_unity_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MT',
    maintainer_email='user@todo.todo',
    description='Unity simulation bringup package with point cloud transformation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_transform_node = mt_unity_sim.pointcloud_transform_node:main',
        ],
    },
)
