from setuptools import find_packages, setup

package_name = 'controller'

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
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='Centrale Controller voor de Airobotcel',
    license='Apache License 2.0',
    tests_require=['pytest'],  # Schoner dan extras_require voor standaard ROS2 templates
    entry_points={
        'console_scripts': [
            'main_controller = controller.main:main',
        ],
    },
)