from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahir',
    maintainer_email='mahir@todo.todo',
    description='YOLO-based object detection nodes for mallet and bottle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mallet_detector_node = object_detection.mallet_detector_node:main',
            'bottle_detector_node = object_detection.bottle_detector_node:main',
        ],
    },
)
