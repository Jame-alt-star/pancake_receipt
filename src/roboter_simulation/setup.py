import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_simulation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal Franz',
    maintainer_email='pascalfranz02@gmail.com',
    description=(
        'Layer 4: MuJoCo-Simulation des Unitree G1 (29 DOF) mit Sensorik.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mujoco_node = roboter_simulation.mujoco_node:main',
        ],
    },
)
