import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kaya_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rajat',
    maintainer_email='atomicrajat619@gmail.com',
    description='Launch files composing the Kaya robot stack.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'cuvslam_origin_anchor = kaya_bringup.origin_anchor_node:main',
        'nvblox_map_loader = kaya_bringup.nvblox_map_loader_node:main',
        'jetson_stats = kaya_bringup.jetson_stats_node:main',
        'map_server_guard = kaya_bringup.map_server_guard_node:main',
        'pose_guard = kaya_bringup.pose_guard_node:main',
        'perception_throttle = kaya_bringup.perception_throttle_node:main',
    ]},
)
