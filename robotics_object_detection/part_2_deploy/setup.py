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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), # Add this line

        ('share/' + package_name + '/calibration_results', ['calibration_results/left.yaml', 'calibration_results/right.yaml']),
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
            'stereo_publisher = robotics_object_detection.stereo_publisher:main',
            'stereo_info_publisher = robotics_object_detection.stereo_info_publisher:main',
        ],
    },
)
