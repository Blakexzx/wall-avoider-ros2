from setuptools import setup
import os
from glob import glob

package_name = 'wall_avoider'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Blake',
    maintainer_email='20144113@tafe.wa.edu.au',
    description='ROS2 wall-avoidance controller for Webots e-puck',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable name = package.module:function
            'wall_avoider_node = wall_avoider.wall_avoider_node:main',
        ],
    },
)