import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pascal',
    maintainer_email='pascal@todo.todo',
    description='Layer 2: generische Nav2-Konfiguration (Platzhalter)',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'punktwolke_zu_scan = roboter_navigation.punktwolke_zu_scan:main',
    ]},
)
