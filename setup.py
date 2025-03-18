from setuptools import find_packages, setup

package_name = 'vehicle_control_using_waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nithish',
    maintainer_email='nithish@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['waypoint_visualizer = vehicle_control_using_waypoints.waypoint_visualizer:main',
        'vehicle_control_using_waypoints = vehicle_control_using_waypoints.vehicle_control_using_waypoints:main',
        ],
    },
)
