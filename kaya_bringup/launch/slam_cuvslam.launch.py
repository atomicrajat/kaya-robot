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
        DeclareLaunchArgument('time_domain', default_value='system',
                              description='Camera timestamp domain: system | global | device. '
                                          'system is required for navigation -- see below.'),
        # The driver defaults to 'global', and that default silently breaks
        # Nav2. cuVSLAM stamps every transform it publishes with the CAMERA's
        # frame time, so the camera's clock becomes the clock for map -> odom
        # and odom -> base_footprint. Measured lag behind ROS time:
        #
        #     time_domain=global   0.879 s median  (image), 0.897 s (map->odom)
        #     time_domain=system   0.030 s median  (image), 0.048 s (map->odom)
        #
        # The offset under 'global' is nearly constant (min 0.876, median
        # 0.879), so it is a clock-sync problem, not jitter -- relative timing
        # is fine, which is why VIO itself never complained. But Nav2 compares
        # stamps against now, so every SLAM transform failed its tolerance as
        # "Transform data too old", and that failure surfaced as the controller
        # announcing "Reached the goal!" without the robot moving.
        #
        # Measured cost to VIO of switching: none detectable. vo_state stayed 1
        # and track time was 10.8-15.3 ms, against 11.1 ms on 'global'. The
        # left/right stereo pair is hardware-synced on the sensor, so both
        # images still share a stamp whichever domain is used.
        DeclareLaunchArgument('enable_laser', default_value='false',
                              description='IR dot projector. Off by default -- the pattern is '
                                          'fixed to the CAMERA, so a tracker locks onto dots that '
                                          'never move relative to it and concludes the robot is '
                                          'stationary. Exposed only because in a dark room there '
                                          'may be no real features at all, which is worse. See '
                                          'the note at enable_laser below.'),
        DeclareLaunchArgument('override_stamp', default_value='false',
                              description='Publish TF stamped with current time instead of the '
                                          'image time. Only needed if camera latency regresses '
                                          '-- see the note at the parameter.'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='Load this saved cuVSLAM map and localize in it, '
                                          'instead of starting a fresh coordinate system. '
                                          'Empty means map-less. See the anchor node below.'),
        DeclareLaunchArgument('hint_x', default_value='0.0',
                              description='Rough starting pose in the saved map. Only a hint -- '
                                          'verified to recover from one 1.8 m off.'),
        DeclareLaunchArgument('hint_y', default_value='0.0'),
        DeclareLaunchArgument('hint_yaw', default_value='0.0'),
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
                # OFF by default, and the docstring above explains why. Made
                # switchable to answer one question empirically: this robot's
                # VIO tracks on the IR pair, so in a dark room it has almost
                # nothing to see. Measured with every room light on --
                #
                #     projector off   brightness 47   texture  4.6
                #     projector on    brightness 42   texture 11.8
                #
                # -- and a 20 m loop at texture 4.6 drifted 3.03 m, against
                # 2.2 cm over 16.4 m in daylight. The theoretical objection to
                # the projector assumes there are real features to corrupt.
                # Whether 11.8 of camera-fixed texture beats 4.6 of real
                # texture is a question about this room, not about theory, so
                # it gets measured rather than argued.
                'enable_laser': LaunchConfiguration('enable_laser'),
                'enable_frame_sync': 'true',
                'time_domain': LaunchConfiguration('time_domain'),
                # Colour and depth are not used by this route. Leaving them off
                # keeps USB bandwidth and GPU memory for SLAM, which matters on
                # an 8 GB board (curriculum Sec. 03).
                # MATCH COLOUR TO DEPTH. Both 848x480, and colour pinned to 30 fps.
                #
                # Left at the driver's 1280x720 default this quietly costs ~15%
                # of every depth frame. depth_registration does the depth->colour
                # alignment in SOFTWARE, and across a resolution mismatch it
                # cannot keep up. Measured, camera alone with nothing else
                # running:
                #
                #     colour 1280x720   depth 26.1 Hz   IR 29.9 Hz
                #     colour  848x480   depth 30.0 Hz   IR 29.9 Hz
                #
                # That 15% loss is what drove the 32-minute mapping run to
                # accumulate 8797 dropped-frame warnings and 45 cm of vertical
                # pose drift, which put nvblox's obstacle band 55 cm above the
                # floor the robot was standing on.
                #
                # This is the SECOND time this exact mismatch has bitten this
                # project -- see slam_rtabmap.launch.py, where the SW aligner
                # silently passed ~15% of frames through UNALIGNED at the same
                # 1280x720/848x480 pairing. The lesson did not transfer to this
                # route because the resolutions are set per-launch-file. If a
                # third route appears, set them there too.
                #
                # colour_fps is pinned because at 848x480 the driver otherwise
                # selects a 60 fps profile -- measured 59.4 Hz -- which is twice
                # the frames nvblox consumes, for nothing.
                'color_width': '848',
                'color_height': '480',
                'color_fps': '30',
                'depth_width': '848',
                'depth_height': '480',
                'depth_fps': '30',
                'enable_color': LaunchConfiguration('depth_for_nvblox'),
                'enable_depth': LaunchConfiguration('depth_for_nvblox'),
                'depth_registration': LaunchConfiguration('depth_for_nvblox'),
                # The depth POINT CLOUD exists for exactly one consumer:
                # collision_monitor's safety layer (config/nav2.yaml). cuVSLAM
                # uses the IR pair and nvblox uses the depth IMAGE, so nothing
                # else reads it -- which makes publishing it at full 848x480
                # (407k points, ~9 Hz) an expensive way to serve one subscriber
                # on a board already sharing 8 GB between cuVSLAM and nvblox.
                #
                # It showed up as latency, not CPU%. Measured stamp lag on
                # /camera/depth/points: 0.508 s median while stationary, spiking
                # to 1.5-1.8 s under motion -- past collision_monitor's timeout,
                # so it kept declaring its only source invalid and STOPPING the
                # robot mid-goal.
                #
                # Decimating 4x in each axis leaves ~25k points, which is ample
                # for "is something in the way" and costs a fraction as much.
                # The alternative was raising source_timeout again, but a
                # safety layer reasoning about 1.8 s-old data is decorative --
                # better to make the data fresh than to widen the window.
                'enable_point_cloud': LaunchConfiguration('depth_for_nvblox'),
                'point_cloud_decimation_filter_factor': '4',
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
                        # This robot is a wheeled base on a flat floor: it has
                        # three degrees of freedom, not six. Left unconstrained
                        # the estimator is free to drift in the other three, and
                        # it does -- a dark-room run put the robot at
                        # Z = -1.795 m, 1.8 metres below the floor it was
                        # standing on, after 2 m of driving.
                        #
                        # Constraining odometry to the ground plane removes that
                        # freedom. It is not a tuning fudge; it is telling the
                        # estimator a true fact about the robot that it
                        # otherwise has to infer from noisy data.
                        #
                        # Only valid while the floor really is flat. If Kaya
                        # ever has to cross a ramp or a threshold of any height,
                        # this has to come back off.
                        'enable_ground_constraint_in_odometry': True,
                        'base_frame': 'base_footprint',
                        'camera_optical_frames': [
                            'camera_left_ir_optical_frame',
                            'camera_right_ir_optical_frame',
                        ],
                        'imu_frame': 'camera_accel_gyro_optical_frame',

                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        # Publish TF stamped with the current time rather than the
                        # image acquisition time.
                        #
                        # HISTORY, because the default flipped once: cuVSLAM
                        # stamps map -> odom and odom -> base_footprint with the
                        # camera's frame time. That lag was originally 0.406 s
                        # median under load, which Nav2 rejected as "Transform
                        # data too old" -- and surfaced as the controller
                        # announcing "Reached the goal!" without the robot
                        # moving. Turning this ON fixed navigation, at the cost
                        # of relabelling a stale pose as current (~6 cm of
                        # unmodelled latency at 0.15 m/s).
                        #
                        # That was a workaround for a latency problem we have
                        # since actually FIXED: decimating the depth point cloud
                        # 4x (see the camera arguments above) took the whole
                        # camera pipeline from 0.464 s median stamp lag to
                        # 0.030 s. With real latency an order of magnitude
                        # lower, the workaround is no longer earning its keep,
                        # so it defaults OFF and the stamps are honest again.
                        #
                        # Set override_stamp:=true if camera latency ever
                        # regresses and "Transform data too old" comes back.
                        'override_publishing_stamp': LaunchConfiguration('override_stamp'),
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

        # ------------------------------------------------ MAP FRAME: anchor it
        # Establishes where "map" is, exactly once, after tracking is healthy.
        # With map_path set it LOCALIZES in that saved map, so the coordinate
        # system matches the session that built it and any goal recorded
        # against it is still valid after a reboot. Without map_path it
        # re-zeroes, pinning a fresh origin to wherever the robot stands.
        #
        # Doing both would be a bug: a reset after a successful localization
        # throws the localization away. That is why one node owns the choice
        # rather than it being two flags to remember. Full reasoning, and the
        # measurements behind waiting on the tracker instead of a timer, are in
        # origin_anchor_node.py.
        Node(
            package='kaya_bringup', executable='cuvslam_origin_anchor',
            name='cuvslam_origin_anchor', output='screen',
            condition=IfCondition(LaunchConfiguration('auto_reset')),
            parameters=[{
                'settle_s': LaunchConfiguration('auto_reset_settle_s'),
                'map_path': LaunchConfiguration('map_path'),
                'hint_x': LaunchConfiguration('hint_x'),
                'hint_y': LaunchConfiguration('hint_y'),
                'hint_yaw': LaunchConfiguration('hint_yaw'),
            }],
        ),
    ])
