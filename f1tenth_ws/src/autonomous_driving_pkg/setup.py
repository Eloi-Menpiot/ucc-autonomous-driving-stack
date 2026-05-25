from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_driving_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'waypoints'), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='F1TENTH autonomous driving package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pp_node = autonomous_driving_pkg.controller_node:main',
            'follow_the_gap = autonomous_driving_pkg.follow_the_gap:main',
            'waypoint_follower = autonomous_driving_pkg.waypoint_follower:main',
            'return_to_origin = autonomous_driving_pkg.return_to_origin:main',
            'person_follow = autonomous_driving_pkg.person_follow:main',
            'camera_publisher = autonomous_driving_pkg.camera_publisher:main',
        ],
    },
)
