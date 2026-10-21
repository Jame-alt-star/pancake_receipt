import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_welten'

# Jede Welt als eigener Ordner unter share/roboter_welten/welten/<name>/ (szene.xml, orte.yaml).
welten = [(os.path.join('share', package_name, d), glob(os.path.join(d, '*.*')))
          for d in sorted(glob('welten/*')) if os.path.isdir(d)]

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ] + welten,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pascal Franz',
    maintainer_email='pascalfranz02@gmail.com',
    description='Welten des G1-Projekts: Szene (Simulation) und Orte (Missionen) je Umgebung',
    license='Apache-2.0',
)
