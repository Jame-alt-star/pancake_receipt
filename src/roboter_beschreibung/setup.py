import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_beschreibung'
G1 = 'modelle/g1_description'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, G1), glob(G1 + '/*.*')),
        (os.path.join('share', package_name, G1, 'meshes'), glob(G1 + '/meshes/*.*')),
        (os.path.join('share', package_name, G1, 'images'), glob(G1 + '/images/*.*')),
        (os.path.join('share', package_name, G1, 'labels'), glob(G1 + '/labels/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal Franz',
    maintainer_email='pascalfranz02@gmail.com',
    description='Unitree-G1-Modell und Robotergeometrie',
    license='Apache-2.0',
)
