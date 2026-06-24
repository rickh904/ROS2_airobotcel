from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'positie_transformatie'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Zorgt ervoor dat eventuele launch- of configbestanden later ook werken
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='ROS 2 node voor het omrekenen van camera- naar robotcoördinaten',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Dit linkt het terminal-commando aan jouw Python script:
            'transformatie_node = positie_transformatie.positie_transformatie:main',
        ],
    },
)
