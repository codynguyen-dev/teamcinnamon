from setuptools import find_packages, setup

package_name = 'mk_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'launch/mvturtle_launch.py',
            'launch/launch_distance.py',
            'launch/converter_launch.py',
            'launch/mk_mode_select_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammadkhan',
    maintainer_email='mohammadkhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'moveturtle = mk_pkg.moveturtle:main',
            'distance = mk_pkg.distance:main',
            'lab1_4 = mk_pkg.lab_14:main',
            'mk_fsm_mode = mk_pkg.mk_fsm_mode:main',
            'mk_mode_select = mk_pkg.mk_mode_select:main',
            'mk_estop_reset = mk_pkg.mk_estop_reset:main',
        ],
    },
)