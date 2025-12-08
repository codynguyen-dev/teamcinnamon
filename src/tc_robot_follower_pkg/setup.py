import os
from glob import glob
from setuptools import setup

package_name = 'tc_robot_follower_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot follower package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = tc_robot_follower_pkg.fsm_node:main',
            'image_processor_node = tc_robot_follower_pkg.image_processor_node:main',
            'dual_pid_controller_node = tc_robot_follower_pkg.dual_pid_controller_node:main',
            'keyboard_mode_switch = tc_robot_follower_pkg.keyboard_mode_switch:main',
        ],
    },
)
