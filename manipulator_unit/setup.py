from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'manipulator_unit'

setup(
    name=package_name,
    version='0.0.0',
    packages=['manipulator_unit'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = manipulator_unit.demo:main',
        ],
    },
)
