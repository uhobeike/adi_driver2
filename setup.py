from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'adis_driver2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [
            # 'scripts_main = '+ package_name +'.scripts_main:ros_main',
            'adis16465_node = bringme_service.adis16465_node:main'        ],
    }
)