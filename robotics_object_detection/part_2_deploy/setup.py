from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robotics_object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', ['weights/yolo11n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bartosz',
    maintainer_email='bartptak@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_results_publisher = robotics_object_detection.detect_results_publisher:main',
        ],
    },
)
