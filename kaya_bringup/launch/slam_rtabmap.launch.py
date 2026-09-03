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

    # Vis/MinInliers: how many feature correspondences must survive geometric
    # (RANSAC) verification before a loop closure is accepted. RTAB-Map's
    # default of 20 suits wide-FOV cameras with generous scene overlap; a
    # 848x480 depth camera indoors legitimately yields fewer.
    #
    # Measured on a real 3.84 m-out / 10.7 m loop returning to within 5 cm and
    # 3 deg: two correct candidates scored 10/20 and 13/20 inliers out of
    # ~112 matches. Those same candidates scored 0/20 before the rectified-
    # colour and depth-alignment fixes, so the geometry is now sound and 20 is
    # simply the wrong bar for this sensor.
    #
    # 12 keeps a real geometric-consistency requirement -- it is not "accept
    # anything" -- while admitting genuine revisits this camera can actually
    # produce. Raise it if false positives ever appear (a loop closure that
    # visibly teleports the map is the symptom).
    min_inliers = LaunchConfiguration('min_inliers')

    # Vis/InlierDistance: how far a reprojected 3D correspondence may sit from
    # its observed position and still count as an inlier. RTAB-Map's 0.1 m
    # default assumes better depth than this sensor has at range: the Gemini
    # 335L's stereo depth error grows with distance squared, so at the 3-4 m
    # where these loop closures actually occur a single point is easily 3-5 cm
    # off, and the relative error across the two frames being matched can
    # exceed 10 cm on its own. That rejects genuine correspondences as
    # outliers. 0.15 m reflects the sensor's real accuracy at loop-closure
    # range rather than loosening the test arbitrarily.
    inlier_distance = LaunchConfiguration('inlier_distance')

    # Feature type, for BOTH the bag-of-words loop detector
    # (Kp/DetectorStrategy) and the geometric verification that follows
    # (Vis/FeatureType). They are separate parameters and are set together
    # here on purpose -- detecting a revisit with one descriptor and then
    # verifying it with another wastes the detection.
    #
    # RTAB-Map defaults both to 8 = GFTT/ORB. ORB's binary descriptors are
    # fast but noticeably less robust to viewpoint and illumination change
    # than SIFT -- and loop closure is exactly the viewpoint-change case:
    # the same place seen later, from a slightly different pose, under
    # whatever the lighting is by then. 1 = SIFT.
    #
    # The usual objection to SIFT is cost, which does not apply here:
    # Rtabmap/DetectionRate is 1 Hz, so this runs once a second, not per
    # frame. Measured feature supply is not the constraint either -- ~970
    # keypoints/frame either way; what we are short of is MATCHES that
    # survive geometric verification (34-113 raw matches, 0-13 inliers).
    feature_type = LaunchConfiguration('feature_type')

    return LaunchDescription([
        DeclareLaunchArgument('delete_db', default_value='false',
                              description='Start a fresh map instead of continuing '
                                          '~/.ros/rtabmap.db from the last run.'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='RTAB-Map\'s own Qt viz UI -- needs a display; '
                                          'off by default on a headless robot.'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('feature_type', default_value='1',
                              description='Kp/DetectorStrategy and Vis/FeatureType. '
                                          '1=SIFT (default here), 8=GFTT/ORB (RTAB-Map\'s own '
                                          'default) -- see the note above.'),
        DeclareLaunchArgument('inlier_distance', default_value='0.15',
                              description='Vis/InlierDistance (m) -- see the note above; '
                                          'RTAB-Map\'s 0.1 default assumes better depth '
                                          'than this sensor delivers at 3-4 m.'),
        DeclareLaunchArgument('min_inliers', default_value='12',
                              description='Vis/MinInliers -- see the note above; RTAB-Map\'s '
                                          'default of 20 is tuned for wider-FOV sensors.'),

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

                'args': [delete_db_flag, ' --Vis/MinInliers ', min_inliers,
                         ' --Vis/InlierDistance ', inlier_distance,
                         ' --Kp/DetectorStrategy ', feature_type,
                         ' --Vis/FeatureType ', feature_type],

                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
