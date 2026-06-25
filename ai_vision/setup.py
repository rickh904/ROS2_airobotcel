from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ai_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files installeren
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Config files installeren
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Model files installeren
        # Let op: glob('resources/*') pakt ook mappen mee en dat geeft build errors.
        # Daarom pakken we expliciet de bestanden in resources/models.
        (os.path.join('share', package_name, 'resources', 'models'),
            glob('resources/models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='ma.slewe@student.avans.nl',
    description='AI vision package for DepthAI YOLO OBB detection',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = ai_vision.vision_node:main',
        ],
    },
)
