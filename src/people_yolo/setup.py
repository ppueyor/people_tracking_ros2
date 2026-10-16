import os
from glob import glob
from setuptools import setup

package_name = 'people_yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RGB-D people detector using YOLOv4-tiny via OpenCV DNN.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = people_yolo.yolo_detector_node:main',
        ],
    },
)
