from setuptools import find_packages, setup

package_name = 'mk_color_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'launch/mk_color_filter_launch.py',
        ]),
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
            'color_filter_node = mk_color_filter.mk_color_filter:main',
            'measurement_data = mk_color_filter.measurement_data:main',
        ],
    },
)
