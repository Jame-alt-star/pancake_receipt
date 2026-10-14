import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_steuerung'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien mit ins Installationsverzeichnis kopieren - sonst
        # findet "ros2 launch roboter_steuerung ..." sie nach colcon build
        # nicht, obwohl sie im Quellcode (launch/) existieren.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'modelle'), glob('modelle/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal Franz',
    maintainer_email='pascalfranz02@gmail.com',
    description='G1-Humanoid-Pipeline (siehe README.md)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'steuereinheit_node = roboter_steuerung.steuereinheit_node:main',
            'balance_dienst_node = roboter_steuerung.balance_dienst_node:main',
            'beinodometrie_node = roboter_steuerung.beinodometrie_node:main',
        ],
    },
)
