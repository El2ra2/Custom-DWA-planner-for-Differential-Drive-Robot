import os
from setuptools import find_packages, setup

package_name = 'dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            ['launch/display.launch.py']),
        (os.path.join('share', package_name, 'rviz'),
            ['rviz/config.rviz']),

        (os.path.join('share', package_name, 'worlds'),
            ['worlds/my_world.sdf']),
        (os.path.join('share', package_name, 'worlds'),
            ['worlds/my_world.world']),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivek',
    maintainer_email='vivek@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'dwa_planner = dwa_planner.dwa_planner:main',
        ],
    },
)
