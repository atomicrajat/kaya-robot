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
                              description='cuVSLAM itself needs only the stereo IR pair, so '
                                          'depth and colour are off by default to save USB '
                                          'bandwidth and GPU memory. Phase 5 sets this true '
                                          'because nvblox fuses '
                                          'DEPTH -- without it nvblox has nothing to build a '
                                          'costmap from. See nav.launch.py.'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true',
                              description='Let cuVSLAM own odom -> base_link. See the TF '
                                          'ownership note below before changing.'),
        DeclareLaunchArgument('point_cloud', default_value='true',
                              description='Publish /camera/depth/points. It is '
                                          'collision_monitor and the local '
                                          'costmap voxel_layer source, so false '
                                          'leaves the robot without live '
                                          'obstacle data -- diagnostic use.'),
        DeclareLaunchArgument('time_domain', default_value='global',
                              description='Camera timestamp domain: global | system | device. '
                                          'global is REQUIRED for VIO accuracy -- read the note '
                                          'below before changing it.'),
        # 'global' is the camera's HARDWARE clock, and this must stay global.
        #
        # time_domain looks like a timestamp-freshness knob. It is also, and far
        # more importantly, what decides whether the IMU and the images share a
        # time base -- and cuVSLAM is a VISUAL-INERTIAL estimator that integrates
        # a ~200 Hz IMU between 30 Hz frames. Measured on this camera:
        #
        #                image interval std   IMU interval std   IMU max deviation
        #     global               0.001 ms           0.004 ms            0.015 ms
        #     system               0.107 ms           5.474 ms           11.374 ms
        #
        # The IMU samples every 4.93 ms. Under 'system' its timestamps deviate by
        # up to 11.4 ms -- more than TWO whole sampling periods. Integrating that
        # produces rotation and velocity error proportional to how much the robot
        # is actually moving: invisible at rest, ruinous under motion.
        #
        # Measured exactly that way, same robot, same room, same night:
        #
        #     global   16.4 m loop   raw VIO drift  1.2%   returned 2.2 cm
        #     system   8.5-24.6 m    raw VIO drift 14.5-32%
        #
        # and stationary under 'system' the robot held 1.4 mm over 94 s. That is
        # why every static check passed while every driven loop failed.
        #
        # WHY IT WAS SET TO 'system' AND HOW THAT WENT WRONG. Under load,
        # 'global' stamps lagged ROS time by ~0.9 s -- a near-constant CLOCK
        # OFFSET, not jitter -- and Nav2 rejects stale transforms, reporting the
        # failure as "Reached the goal!" without moving. Switching to 'system'
        # fixed Nav2 and silently destroyed VIO accuracy. The commit that made
        # the change claimed "measured cost to VIO: none detectable, vo_state
        # stayed 1 and track time was 10.8-15.3 ms" -- but vo_state and track
        # time measure PERFORMANCE, not ACCURACY. A drifting estimator reports
        # vo_state=1 at full speed right up until it is metres wrong.
        #
        # The right tool for the Nav2 staleness problem is override_stamp:=true,
        # which republishes TF with the current time and leaves IMU timing alone.
        # (Since then the point-cloud decimation fix cut real latency from 0.46 s
        # to 0.028 s, so the offset may no longer bite at all -- verify before
        # reaching for the override.)
        DeclareLaunchArgument('enable_laser', default_value='false',
                              description='IR dot projector. Off by default -- the pattern is '
                                          'fixed to the CAMERA, so a tracker locks onto dots that '
                                          'never move relative to it and concludes the robot is '
                                          'stationary. Exposed only because in a dark room there '
                                          'may be no real features at all, which is worse. See '
                                          'the note at enable_laser below.'),
        DeclareLaunchArgument('color_fps', default_value='5',
                              description='Colour frame rate. 5, not 30: at 30 the colour stream '
                                          'starved cuVSLAM\'s left eye (24.5 Hz / 66.6 ms p90 '
                                          'against the right\'s 28.8 Hz / 35.0 ms), and cuVSLAM '
                                          'needs matched pairs. Nothing consuming colour needs '
                                          '30 Hz.'),
        DeclareLaunchArgument('enable_color', default_value='true',
                              description='Colour stream. Costs USB bandwidth and CPU '
                                          'navigation does not need: measured 15 Hz, 133 ms '
                                          'p90 gap, the worst stream on this camera. nvblox '
                                          'needs it only for the mesh; the costmap comes from '
                                          'depth. False gives the stereo pair more of the pipe.'),
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
                # PIN THE IR PAIR. These are the only streams cuVSLAM runs
                # on, and they were the one pair left unconfigured: depth got
                # an explicit 848x480 at 30, colour was deliberately pinned
                # because the driver otherwise picked a 60 fps profile, and the
                # IR pair was left to whatever default the driver chose.
                #
                # MEASURED, unpinned, on the robot at idle:
                #
                #   /camera/left_ir/image_raw    20.1 Hz   gap median 51.2 ms
                #   /camera/right_ir/image_raw   20.2 Hz   gap median 51.4 ms
                #
                # cuVSLAM expects 30 Hz and warns above a 34 ms gap, so at
                # 51 ms EVERY frame is late and the log fills with "Delta
                # between current and previous frame is above threshold".
                # That was read as a camera stall under load for most of a
                # session. It is not load: the link is USB 3.2 at 5000 Mbps
                # carrying about 24 MB/s, and the board has bandwidth to spare.
                # The camera was simply never asked for 30.
                #
                # It matters well past the warnings. Late IR frames make
                # cuVSLAM's pose output late, which makes TF late, and
                # collision_monitor then cannot transform its depth cloud into
                # the base frame:
                #
                #   Lookup would require extrapolation into the future.
                #   Requested ...494.668 but latest data is ...494.472
                #   Robot to stop due to invalid source
                #
                # A safety layer doing exactly its job on a pipeline that is
                # 0.2 s behind, which aborted two goals partway across the flat.
                # A SENSOR CANNOT EXPOSE FOR LONGER THAN ITS FRAME PERIOD.
                # At 30 fps that period is 33.3 ms. The driver shipped with
                # `enable_ir_long_exposure` TRUE and `ir_ae_max_exposure` at
                # -1, meaning auto-exposure had no upper bound -- so in
                # ordinary indoor light it settled around 50 ms and the sensor
                # simply could not produce 30 fps. Measured, before and after,
                # nothing else changed:
                #
                #                     before          after
                #   left_ir           18.0 Hz         28.4 Hz
                #   right_ir          20.9 Hz         30.8 Hz
                #   cuVSLAM warnings  ~1278 / 40 s    228 / 40 s
                #
                # The 51 ms median gap between IR frames WAS the exposure time.
                # This was read as a camera stall under CPU load for most of a
                # session, and it is not load: the link is USB 3.2 at 5000 Mbps
                # carrying ~24 MB/s, memory was 4.1 GB free, and pinning the IR
                # rate to 30 changed nothing because the rate was never the
                # constraint -- the exposure was.
                #
                # It also explains why the PAIR desynchronised, left running
                # 3 Hz slower than right: two sensors metering slightly
                # different scenes choose slightly different exposures, and a
                # stereo pair that drifts apart is exactly what cuVSLAM cannot
                # use. The cap makes both deterministic.
                #
                # 20 ms leaves headroom under the 33.3 ms period for readout.
                # If tracking degrades in a genuinely dark room, raise gain --
                # not this. Trading frame rate for brightness is what broke it.
                'enable_ir_long_exposure': 'false',
                'ir_ae_max_exposure': '20000',
                # SYNC THE CAMERA CLOCK TO THE HOST.
                # time_domain is 'global', the camera's own hardware clock, and
                # it has to stay that way -- see the note above, where 'system'
                # was measured corrupting IMU intervals by up to 11.4 ms. But
                # the driver ships with enable_sync_host_time FALSE, so that
                # hardware clock free-runs against the host's, and MEASURED it
                # sits about 0.5 s behind:
                #
                #   map -> odom              24.8 Hz   stamp lag 0.545 s
                #   odom -> base_footprint   24.7 Hz   stamp lag 0.547 s
                #   base_link -> wheel_1     15.0 Hz   stamp lag 0.005 s
                #
                # The wheel transforms come off the same board on the system
                # clock and are current; only the camera-derived ones are half
                # a second in the past. That is a CLOCK OFFSET, not latency,
                # and cuVSLAM inherits it into every transform it publishes.
                #
                # It is what makes Nav2 fail to transform map -> odom, and a
                # goal checker that cannot compute a distance falls through to
                # "Reached the goal!" -- the robot stopping 23 cm into a 5 m
                # drive and reporting success, reproducibly.
                #
                # Syncing to host time keeps the hardware timebase (so image
                # and IMU stay consistent, which is the whole reason for
                # 'global') while removing the offset against the host.
                'enable_sync_host_time': 'true',
                # DO NOT MAKE THE IR PAIR WAIT FOR DEPTH.
                # enable_frame_sync aligns every enabled stream to a common
                # frame, which means the driver holds the fast ones until the
                # slow one is ready. Depth is the slow one, so the IR pair --
                # the only input cuVSLAM uses -- was being buffered behind it.
                # NOTE: until 2026-09-27 a stale 'enable_frame_sync': 'true'
                # sat further down this dict, and a later key wins - so this
                # 'false' never took effect and the IR pair DID wait for depth.
                'enable_frame_sync': 'false',
                'enable_left_ir': 'true',
                'left_ir_width': '848',
                'left_ir_height': '480',
                'left_ir_fps': '30',
                'enable_right_ir': 'true',
                'right_ir_width': '848',
                'right_ir_height': '480',
                'right_ir_fps': '30',
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
                # project -- the RTAB-Map route (removed 2026-09-27; in git history
                # as kaya_bringup/launch/slam_rtabmap.launch.py), where the SW aligner
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
                # 5 fps, not 30, and this is the fix for a MEASURED fault
                # rather than a saving. At 30 fps the colour stream starved
                # cuVSLAM's LEFT eye: measured with the full nav stack, left IR
                # ran 24.5 Hz at a 66.6 ms p90 gap against the right's 28.8 Hz
                # and 35.0 ms. cuVSLAM needs matched PAIRS, so the slow eye set
                # the true tracking rate, and stereo_sync.py showed 0% unmatched
                # throughout -- correctly, because every surviving left frame
                # does have a right. Only comparing the two RATES exposes it.
                #
                # Turning colour off entirely balanced the pair and took a
                # 3.5 m traverse from ~20 cm to 6.3 cm, but that is not a fix:
                # the dashboard view, open-vocabulary detection and any
                # reasoning model all need RGB. Nothing needs it at 30 Hz --
                # the dashboard polls ~6 Hz, and the roadmap's own rule for
                # detection is 1-2 Hz because "a room does not change fast".
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': '848',
                'depth_height': '480',
                'depth_fps': '30',
                # Colour is separable from depth on purpose. It costs USB
                # bandwidth and CPU that navigation does not need: measured at
                # 15 Hz with a 133 ms p90 gap, the worst-behaved stream on the
                # camera, while nvblox only needs it for the mesh and the
                # costmap comes from the depth-derived ESDF slice. Turn it off
                # to give cuVSLAM's stereo pair more of the pipe; turn it on
                # for mapping runs, the dashboard view and anything that has to
                # see in colour.
                'enable_color': LaunchConfiguration('enable_color'),
                'enable_depth': LaunchConfiguration('depth_for_nvblox'),
                # REGISTRATION FOLLOWS COLOUR, NOT DEPTH.
                # depth_registration reprojects the depth image into the COLOUR
                # camera's frame, so it is meaningless with colour disabled --
                # and it was wired to depth_for_nvblox, which is hard-coded true
                # in nav.launch.py. So every depth frame was being reprojected
                # to match a stream that was not running.
                #
                # It is not free, and what it costs is LATENCY ON THE IR PAIR.
                # Measured on the left IR stream, the input cuVSLAM runs on:
                #
                #   depth pipeline ON    IR stamp lag  0.459 s
                #   depth pipeline OFF   IR stamp lag  0.019 s
                #
                # 440 ms, on a stream that had nothing to do with depth. That
                # lag is inherited by every transform cuVSLAM publishes, and it
                # is why Nav2 could not convert map -> odom, why the goal
                # checker fell through to "Reached the goal!" 23 cm into a 5 m
                # drive, and why collision_monitor kept declaring its source
                # invalid and stopping the robot.
                'depth_registration': LaunchConfiguration('enable_color'),
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
                'enable_point_cloud': LaunchConfiguration('point_cloud'),
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
                        # TRIED AND REJECTED, kept documented so it is not
                        # tried again. Kaya is a wheeled base on a flat floor,
                        # so constraining odometry to the ground plane looks
                        # obviously correct -- three degrees of freedom instead
                        # of six, and it directly targets a dark-room run that
                        # put the robot at Z = -1.795 m.
                        #
                        # Measured on a there-and-back loop, night, projector
                        # on, same conditions both times:
                        #
                        #     no constraint    8.5 m path   raw VIO drift 14.5%
                        #     constraint      24.6 m path   raw VIO drift 32%
                        #
                        # It also failed at its own stated job. Raw VIO Z was
                        # pinned to 0.000 exactly as intended, but map -> odom
                        # then absorbed 0.790 m of Z, leaving the FINAL pose at
                        # Z = 0.875. The constraint does not flatten the
                        # trajectory; it relocates the error into the SLAM layer
                        # where the constraint does not reach.
                        #
                        # The likely mechanism for the harm: this robot really
                        # does pitch and roll slightly -- omni rollers, real
                        # floors, acceleration -- so forcing those to zero tells
                        # the estimator something false, and the inconsistency
                        # has nowhere to go but planar motion. Over-constraining
                        # injects error instead of removing it.
                        #
                        # Caveat: one run each and the routes differed in
                        # length, so the percentages are not a perfectly clean
                        # comparison. But there is no evidence it helps, a
                        # mechanism by which it hurts, and it demonstrably did
                        # not flatten the map.
                        'enable_ground_constraint_in_odometry': False,
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
