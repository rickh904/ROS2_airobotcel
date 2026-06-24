from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'manipulator_unit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='ROS 2 manipulator unit met Robot en GoHome functionaliteit',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Robot = manipulator_unit.Robot:main',
            'GoHome = manipulator_unit.GoHome:main',
        ],
    },
)
