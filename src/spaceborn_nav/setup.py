from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'spaceborn_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gharablyy',
    maintainer_email='gharably@todo.todo',
    description='Autonomous navigation and visualisation for the Husky A200 with RTAB-Map.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtabmap_dashboard = spaceborn_nav.rtabmap_dashboard:main',
            'frontier_explorer = spaceborn_nav.frontier_explorer:main',
            'signal_jammer = spaceborn_nav.signal_jammer:main',
            'jammer_controller = spaceborn_nav.jammer_controller:main',
        ],
    },
)
