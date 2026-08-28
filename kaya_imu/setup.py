import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kaya_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Tells the ROS 2 index this package exists. Without it `ros2 run` cannot find you.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Anything you want installed alongside the code has to be listed here.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rajat',
    maintainer_email='atomicrajat619@gmail.com',
    description='MPU6050 IMU driver for the Kaya robot.',
    license='Apache-2.0',
    entry_points={
        # 'name = module:function' -> creates the executable `ros2 run kaya_imu <name>`
        'console_scripts': [
            'imu_node = kaya_imu.imu_node:main',
            'calibrate = kaya_imu.calibrate:main',
            'camera_imu_node = kaya_imu.camera_imu_node:main',
        ],
    },
)
