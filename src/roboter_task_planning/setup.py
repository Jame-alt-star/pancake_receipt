import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roboter_task_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*.pddl')),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yassi',
    maintainer_email='yassi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_node = roboter_task_planning.mission_node:main',
            'nav_testdouble_node = roboter_task_planning.testing.nav_testdouble_node:main',
        ],
    },
)
