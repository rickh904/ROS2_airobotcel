from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'transformatie_pakket'

launch_files = glob('launch/*') if os.path.exists('launch') else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Dit linkt nu netjes naar het bestand in je fysieke resource map:
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='ROS 2 node voor het omrekenen van camera- naar robotcoördinaten',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # executable_naam = binnenste_map.scriptnaam:functie
            'positie_transformatie.py = transformatie_pakket.positie_transformatie:main',
        ],
    },
)
