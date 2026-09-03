"""Phase 1 bringup: camera IMU -> conditioned IMU -> TF tree.

    ros2 launch kaya_bringup imu.launch.py
    ros2 launch kaya_bringup imu.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('kaya_description')
    orbbec_share = get_package_share_directory('orbbec_camera')

    use_rviz = LaunchConfiguration('rviz')
    auto_bias = LaunchConfiguration('auto_bias')

    # Command(...) runs xacro AT LAUNCH TIME and captures its stdout. This is
    # why you never commit a generated .urdf: the .xacro is the source, and it
    # is expanded fresh every run so parameter changes cannot go stale.
    # ParameterValue(..., value_type=str) is MANDATORY here. Without it launch
    # tries to YAML-parse the xacro output; URDF is XML, so it fails with
    # "Unable to parse the value of parameter robot_description as yaml"
    # and tears down the entire launch.
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc_share, 'urdf', 'kaya.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('auto_bias', default_value='true',
                              description='Estimate gyro bias at startup (robot must be still)'),
        DeclareLaunchArgument('real_joint_states', default_value='false',
                              description='Set true when kaya_drive drive_node is also running: '
                                          'it publishes real encoder-derived /joint_states, so the '
                                          'zero-filling placeholder below must not also publish '
                                          'there or the two fight over the same topic.'),

        # ------------------------------------------------ 1. the camera driver
        # Including another package's launch file rather than re-declaring its
        # nodes: the Orbbec launch handles ~80 parameters and a component
        # container. Re-implementing that would rot the moment they update it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_share, 'launch', 'gemini_330_series.launch.py')),
            launch_arguments={
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'enable_sync_output_accel_gyro': 'true',   # -> /camera/gyro_accel/sample
                'enable_color': 'false',                   # Phase 1 needs no colour
            }.items(),
        ),

        # ---------------------------------------- 2. condition the IMU stream
        Node(
            package='kaya_imu', executable='camera_imu_node', name='camera_imu_node',
            output='screen',
            parameters=[{
                'input_topic': '/camera/gyro_accel/sample',
                'output_topic': '/imu/data_raw',
                'auto_bias_on_start': auto_bias,
                'bias_samples': 600,
            }],
        ),

        # ------------------------------------ 3. URDF -> TF (base_link -> ...)
        # robot_state_publisher walks the URDF and publishes every FIXED joint
        # on /tf_static and every movable joint on /tf, driven by /joint_states.
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # The three wheel joints are `continuous`, so TF needs their angles.
        # Until kaya_drive's drive_node is running, this supplies zeros so the
        # tree is complete. Disabled via real_joint_states when it is.
        Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            name='joint_state_publisher', output='log',
            condition=UnlessCondition(LaunchConfiguration('real_joint_states')),
        ),

        # ------------------------------------------------------------ 4. RViz
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(desc_share, 'rviz', 'kaya.rviz')],
        ),
    ])
