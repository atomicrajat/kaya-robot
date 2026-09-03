"""Phase 4, route 1: RTAB-Map RGB-D SLAM, using our own wheel+IMU odometry.

    ros2 launch kaya_bringup slam_rtabmap.launch.py
    ros2 launch kaya_bringup slam_rtabmap.launch.py delete_db:=true   # fresh map

Curriculum's recommended starting point for Phase 4: forgiving, well
documented, gives a working map in days and a known-good reference to check
cuVSLAM against later. cuVSLAM (Isaac ROS Visual SLAM) is the real target --
GPU-accelerated stereo IR + IMU visual-inertial odometry -- and comes after
this route is working.

Brings up the full stack: camera (colour + depth, registered/aligned to the
colour frame), conditioned IMU, URDF/TF, kaya_drive, the Phase 3 EKF, and
RTAB-Map's mapping node consuming that EKF odometry via TF -- NOT RTAB-Map's
own visual odometry. Curriculum: "map -> odom is a correction published on
top of odom -> base_link, never a replacement for it" -- we already have a
real, calibrated odom -> base_footprint from Phase 2/3; RTAB-Map's job here
is only the map -> odom loop-closure correction on top of it.

Topic names are Orbbec's actual ones (verified live, not the rtabmap_launch
defaults which assume a different camera driver's naming):
  rgb   /camera/color/image_raw       + /camera/color/camera_info
  depth /camera/depth/image_raw       (registered to colour when
                                        depth_registration:=true is set below
                                        -- there is a separate, unaligned
                                        /camera/depth/image_unaligned)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup_share = get_package_share_directory('kaya_bringup')
    rtabmap_share = get_package_share_directory('rtabmap_launch')

    # rtabmap.launch.py's own 'args'/'rtabmap_args' is a raw CLI-flag string;
    # translate our boolean delete_db arg into it rather than exposing that
    # raw-flag interface directly.
    delete_db_flag = PythonExpression(
        ["'--delete_db_on_start' if '", LaunchConfiguration('delete_db'), "' == 'true' else ''"])

    return LaunchDescription([
        DeclareLaunchArgument('delete_db', default_value='false',
                              description='Start a fresh map instead of continuing '
                                          '~/.ros/rtabmap.db from the last run.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='RTAB-Map\'s own Qt viz UI -- needs a display; '
                                          'off by default on a headless robot.'),
        DeclareLaunchArgument('rviz', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'localization.launch.py')),
            launch_arguments={
                'enable_color': 'true',
                'depth_registration': 'true',
                # RTAB-Map expects RECTIFIED images -- rtabmap_launch's own
                # default topic name is literally "image_rect_color". The
                # Gemini 335L's factory camera_info carries real plumb_bob
                # distortion (k1 -0.034, k2 0.038), and the raw stream is
                # distorted, so feeding image_raw as if it were rectified made
                # 3D reprojection systematically wrong: 2D appearance matching
                # still worked (both frames distorted identically, ~105
                # matches) but every point failed geometric verification, i.e.
                # the "Not enough inliers 0/20" loop-closure rejections. This
                # makes the driver publish genuinely undistorted colour, and
                # its camera_info then reports D = all zeros.
                'enable_color_undistortion': 'true',
                # Match colour to the depth sensor's NATIVE 848x480 so the
                # driver's depth->colour registration is 1:1 pixel work.
                # At the 1280x720 default the SW aligner could not keep up and
                # silently passed ~15% of frames through UNALIGNED at native
                # 848x480 -- while camera_info still advertised 1280x720, so
                # those frames' depth got projected into 3D with intrinsics off
                # by ~1.5x. Measured 483/483 consistent after matching; it was
                # 263/309 (85.1%) before.
                # HW alignment would avoid the CPU cost entirely but the device
                # rejects it for this profile pair ("Current stream profile is
                # not support hardware d2c process"), so SW + matched
                # resolutions is the working combination.
                # 848x480 is also the right call for SLAM on an 8 GB board --
                # curriculum Sec. 03: resolution and frame rate are budget
                # decisions, not defaults.
                'color_width': '848',
                'color_height': '480',
                'depth_width': '848',
                'depth_height': '480',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_share, 'launch', 'rtabmap.launch.py')),
            launch_arguments={
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'approx_sync': 'true',    # driver does not pre-sync rgb/depth/info itself
                # Guard rail, not a fix for anything currently broken: measured
                # colour-vs-depth stamp gap is 0.3 ms median / 0.4 ms max, so
                # this drops nothing in normal operation. It bounds the damage
                # if a stream ever stalls, since the default (0.0) means an
                # INFINITE permitted gap -- pairing a colour frame with
                # arbitrarily stale depth would put the 3D points somewhere the
                # robot no longer is.
                'approx_sync_max_interval': '0.04',

                'frame_id': 'base_footprint',
                # TF, not a topic: we already publish a real odom -> base_footprint
                # (Phase 3's EKF) and want RTAB-Map to read it the same way every
                # other consumer in this project does, not add a second path.
                'odom_frame_id': 'odom',
                'visual_odometry': 'false',   # use ours, not RTAB-Map's own VO
                'publish_tf_map': 'true',     # RTAB-Map publishes map -> odom

                'args': delete_db_flag,

                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
