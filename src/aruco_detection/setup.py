from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_detection'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Hironmoy Roy Rudra',
    maintainer_email='hironmoy.roy.rudra@g.bracu.ac.bd',
    description='ArUco marker detection and tracking for Unity simulation',
    license='GPL-3.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detection_node = aruco_detection.aruco_detection_node:main',
        ],
    },
)
