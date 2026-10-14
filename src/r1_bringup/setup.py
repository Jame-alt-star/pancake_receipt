import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'r1_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models/policies'), glob('models/policies/*')),
        (os.path.join('share', package_name, 'models/robots/g1_description'), glob('models/robots/g1_description/*.*')),
        (os.path.join('share', package_name, 'models/robots/g1_description/meshes'), glob('models/robots/g1_description/meshes/*.*')),
        (os.path.join('share', package_name, 'models/robots/g1_description/images'), glob('models/robots/g1_description/images/*.*')),
        (os.path.join('share', package_name, 'models/robots/g1_description/labels'), glob('models/robots/g1_description/labels/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal',
    maintainer_email='pascalfranz02@gmail.com',
    description=(
        'Layer 4: MuJoCo-Simulation des Unitree G1 (29 DOF) mit Sensorik.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_node = r1_bringup.mujoco_node:main',
        ],
    },
)
