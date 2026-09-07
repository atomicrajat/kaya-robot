"""Phase 4, route 2: Isaac ROS Visual SLAM (cuVSLAM), stereo IR + IMU.

    ros2 launch kaya_bringup slam_cuvslam.launch.py
    ros2 launch kaya_bringup slam_cuvslam.launch.py tracking_mode:=0   # stereo only, no IMU

The real target for Phase 4. Unlike the RTAB-Map route this consumes the
camera's STEREO IR pair, not registered RGB-D -- which structurally avoids
every failure mode that route hit. Verified on this camera:

    left/right IR   848x480, mono8          <- cuVSLAM's native input format,
                                               no format conversion needed
    camera_info D   0.00000 on both         <- ALREADY RECTIFIED. The RGB-D
                                               path had real plumb_bob
                                               distortion and needed the
                                               driver's undistortion.
    baseline        P[3]/fx = 38.72/408.2
                    = 94.9 mm               <- matches the documented 95 mm
    stereo sync     0.178 ms median,
                    0.198 ms max            <- genuine hardware sync

There is no depth registration step, no software aligner, and no resolution
mismatch possible here, because both images come off the same sensor pair at
the same resolution with the same clock.

*** enable_laser MUST stay false. *** The IR projector's dot pattern is fixed
to the CAMERA, not the world, so feature matchers lock onto it and pose
estimation degrades silently. The driver's default is true. This is the one
setting most likely to be lost if someone edits the camera arguments.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('kaya_description')
    orbbec_share = get_package_share_directory('orbbec_camera')
    drive_share = get_package_share_directory('kaya_drive')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([desc_share, 'urdf', 'kaya.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('tracking_mode', default_value='1',
                              description='0=stereo only, 1=VIO (IMU fusion), 2=RGBD. '
                                          '1 is the point of using this camera -- its IMU is '
                                          'rigidly co-located with the imagers and shares their '
                                          'clock, which is what VIO requires.'),
        DeclareLaunchArgument('enable_drive', default_value='true',
                              description='Bring up kaya_drive so the robot can be teleoperated.'),
        DeclareLaunchArgument('depth_for_nvblox', default_value='false',
                              description='cuVSLAM itself needs only the stereo IR pair, so depth '
                                          'and colour are off by default to save USB bandwidth and '
                                          'GPU memory. Phase 5 sets this true because nvblox fuses '
                                          'DEPTH -- without it nvblox has nothing to build a '
                                          'costmap from. See nav.launch.py.'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='Let cuVSLAM own odom -> base_link. See the TF '
                                          'ownership note below before changing.'),
        DeclareLaunchArgument('auto_reset', default_value='true',
                              description='Re-zero the tracker once after startup settles. '
                                          'See the ORIGIN note at the bottom of this file.'),
        DeclareLaunchArgument('auto_reset_settle_s', default_value='5.0',
                              description='How long cuVSLAM must track healthily before its '
                                          'origin is re-zeroed. This is a settle window on the '
                                          'TRACKER, not a delay from launch -- see the node.'),

        # ------------------------------------------------------- camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_share, 'launch', 'gemini_330_series.launch.py')),
            launch_arguments={
                'enable_left_ir': 'true',
                'enable_right_ir': 'true',
                # See the module docstring -- non-negotiable for stereo VSLAM.
                'enable_laser': 'false',
                'enable_frame_sync': 'true',
                'time_domain': 'global',
                # Colour and depth are not used by this route. Leaving them off
                # keeps USB bandwidth and GPU memory for SLAM, which matters on
                # an 8 GB board (curriculum Sec. 03).
                'enable_color': LaunchConfiguration('depth_for_nvblox'),
                'enable_depth': LaunchConfiguration('depth_for_nvblox'),
                'depth_registration': LaunchConfiguration('depth_for_nvblox'),
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'enable_sync_output_accel_gyro': 'true',
            }.items(),
        ),

        # ------------------------------------------------------------ URDF/TF
        # cuVSLAM needs base_frame -> camera_optical_frames from TF to place the
        # cameras on the robot. The URDF supplies base_link -> camera_link and
        # the driver supplies camera_link -> the IR optical frames.
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drive_share, 'launch', 'drive.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_drive')),
            launch_arguments={
                # ---------------------------- TF OWNERSHIP, read before editing
                # cuVSLAM publishes odom -> base_link itself, so drive_node must
                # NOT also publish that transform (TF allows exactly one parent
                # per frame; two publishers produce a transform that flickers
                # between them). drive_node still publishes /odom and
                # /joint_states, which stay useful as a cross-check against the
                # visual estimate and for the wheel-slip comparison.
                #
                # This deliberately REPLACES the Phase 3 EKF for this route
                # rather than feeding it: cuVSLAM's VIO estimates gyro bias
                # online as part of its own state, which is the proper fix for
                # the drift that robot_localization structurally cannot solve
                # (it has no IMU bias states). Fusing a drifting wheel+gyro EKF
                # into a VIO that already handles bias would add error, not
                # remove it.
                #
                # The longer-term production shape is the other way round: take
                # cuVSLAM's odometry as ONE input to a filter that also has
                # wheel odometry, so wheel encoders carry you through visual
                # dropout (blank walls, sudden lighting change, motion blur).
                # That is worth doing once cuVSLAM alone is proven here.
                'publish_tf': 'false',
            }.items(),
        ),

        ComposableNodeContainer(
            name='cuvslam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    name='visual_slam_node',
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    parameters=[{
                        # Confirmed from camera_info: D = 0 on both IR streams.
                        'rectified_images': True,
                        'enable_image_denoising': False,
                        'tracking_mode': LaunchConfiguration('tracking_mode'),

                        # base_footprint, NOT base_link. The URDF roots the robot
                        # at base_footprint and parents base_link beneath it, so
                        # publishing odom -> base_link here gives base_link TWO
                        # parents. TF permits exactly one, and the tree splits:
                        # "Could not find a connection between 'odom' and
                        # 'base_footprint' ... Tf has two or more unconnected
                        # trees", after which Nav2's costmaps cannot activate at
                        # all. Anchoring here restores the REP-105 chain
                        # map -> odom -> base_footprint -> base_link.
                        #
                        # This also puts the map origin ON THE FLOOR rather than
                        # one wheel radius above it, which is what makes nvblox's
                        # slice heights genuinely floor-referenced. See
                        # config/nvblox.yaml -- the two must be reasoned about
                        # together.
                        'base_frame': 'base_footprint',
                        'camera_optical_frames': [
                            'camera_left_ir_optical_frame',
                            'camera_right_ir_optical_frame',
                        ],
                        'imu_frame': 'camera_accel_gyro_optical_frame',

                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'publish_tf': LaunchConfiguration('publish_odom_tf'),
                        'publish_map_tf': True,

                        # IMU noise, derived from THIS camera rather than copied
                        # from NVIDIA's RealSense example. camera_imu_node
                        # measures the noise floor at every startup; converting
                        # its reported standard deviations at 200 Hz to spectral
                        # densities (sd * sqrt(dt)):
                        #   gyro  sd ~5.0e-4 rad/s   -> 5.0e-4 * sqrt(1/200)
                        #                            = 3.5e-5 rad/s/sqrt(Hz)
                        #   accel sd ~1.5e-2 m/s^2   -> 1.1e-3 m/s^2/sqrt(Hz)
                        # Random walk is an ESTIMATE, not an Allan-variance
                        # measurement: the yaw-axis bias moved 2.03e-3 rad/s
                        # over ~1800 s of warm-up, so ~4.8e-5 rad/s/sqrt(s).
                        # Worth refining with a proper overnight Allan plot if
                        # VIO ever looks over- or under-confident in the IMU.
                        'gyro_noise_density': 3.5e-5,
                        'gyro_random_walk': 4.8e-5,
                        'accel_noise_density': 1.1e-3,
                        'accel_random_walk': 3.0e-3,
                        'calibration_frequency': 200.0,

                        'enable_slam_visualization': True,
                        'enable_landmarks_view': True,
                        'enable_observations_view': True,
                    }],
                    remappings=[
                        # Already mono8 and already rectified, so these go
                        # straight in -- no ImageFormatConverterNode and no
                        # rectify node, unlike NVIDIA's RGB examples.
                        ('visual_slam/image_0', '/camera/left_ir/image_raw'),
                        ('visual_slam/camera_info_0', '/camera/left_ir/camera_info'),
                        ('visual_slam/image_1', '/camera/right_ir/image_raw'),
                        ('visual_slam/camera_info_1', '/camera/right_ir/camera_info'),
                        # RAW IMU on purpose, not kaya_imu's bias-corrected
                        # /imu/data_raw: cuVSLAM estimates gyro bias itself as
                        # part of the VIO state, and two independent bias
                        # corrections fighting each other is worse than one
                        # done properly inside the estimator.
                        ('visual_slam/imu', '/camera/gyro_accel/sample'),
                    ],
                ),
            ],
        ),

        # ------------------------------------------------- ORIGIN: re-zero once
        # cuVSLAM's origin is wherever the tracker thought it was when it
        # initialised, which is when it is least trustworthy -- measured here
        # as a (0.285, -0.061, -0.360) m / -9 deg offset on a robot that never
        # moved. The full reasoning, and why this is a node watching the
        # tracker rather than a fixed launch delay, is in origin_reset_node.py.
        # Short version: a fixed delay guesses at a machine whose startup time
        # is not predictable, and the first version of this guessed wrong and
        # reset the tracker mid-initialisation.
        Node(
            package='kaya_bringup', executable='cuvslam_origin_reset',
            name='cuvslam_origin_reset', output='screen',
            condition=IfCondition(LaunchConfiguration('auto_reset')),
            parameters=[{'settle_s': LaunchConfiguration('auto_reset_settle_s')}],
        ),
    ])
