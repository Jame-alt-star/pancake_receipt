import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_sensorik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal Franz',
    maintainer_email='pascalfranz02@gmail.com',
    description='Sensorvorverarbeitung: Punktwolke -> 2D-Scan und Hinderniswolke',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'punktwolke_zu_scan = roboter_sensorik.punktwolke_zu_scan:main',
    ]},
)
