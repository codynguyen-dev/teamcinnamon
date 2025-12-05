from setuptools import find_packages, setup

package_name = 'mk_pid_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mk_pid_control_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammadkhan',
    maintainer_email='mohammadkhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_control = mk_pid_control.mk_pid:main',       
        ],
    },
)
