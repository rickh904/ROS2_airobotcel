from setuptools import find_packages, setup

package_name = 'hmi_unit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hmi.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='GerardAnneHarkema@gmail.com',
    description='HMI node voor de sorteercel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # RECHTGEZET: hmi_unit (map) -> main (bestand) -> main (functie)
            'hmi_node = hmi_unit.main:main',
        ],
    },
)