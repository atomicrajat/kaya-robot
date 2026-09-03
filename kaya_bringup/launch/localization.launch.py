"""Phase 3 bringup: fuse wheel odometry with the camera's built-in IMU.

    ros2 launch kaya_bringup localization.launch.py
    ros2 launch kaya_bringup localization.launch.py rviz:=true

Brings up the full sensing + drive + localization stack: camera, conditioned
IMU, URDF/TF, kaya_drive, and an EKF (robot_localization) that fuses wheel
velocity with gyro yaw rate. The EKF -- not drive_node -- publishes
odom -> base_footprint; drive_node is launched with publish_tf:=false so
there is exactly one authority for that transform.

See kaya_bringup/config/ekf.yaml for what each source is trusted for and why.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaya_bringup')
    drive_share = get_package_share_directory('kaya_drive')

    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('enable_color', default_value='false',
                              description='Needed for Phase 4 RGB-D SLAM, not for plain localization.'),
        DeclareLaunchArgument('depth_registration', default_value='false',
                              description='Align depth to colour -- ditto, Phase 4 only.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'imu.launch.py')),
            launch_arguments={
                'real_joint_states': 'true',  # kaya_drive publishes the real ones below
                'rviz': use_rviz,
                'enable_color': LaunchConfiguration('enable_color'),
                'depth_registration': LaunchConfiguration('depth_registration'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drive_share, 'launch', 'drive.launch.py')),
            launch_arguments={
                'publish_tf': 'false',  # the EKF below owns odom -> base_footprint instead
            }.items(),
        ),

        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml')],
        ),
    ])
