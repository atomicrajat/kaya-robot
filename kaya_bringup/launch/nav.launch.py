"""Phase 5: autonomous navigation -- SLAM + nvblox + Nav2.

    ros2 launch kaya_bringup nav.launch.py
    ros2 launch kaya_bringup nav.launch.py slam:=rtabmap    # RGB-D route

Mapping vs. returning to a known map:

    # first visit -- build a map, then save it
    ros2 launch kaya_bringup nav.launch.py
    ros2 run kaya_bringup kaya_save_map ~/kaya_maps/home

    # every visit after -- reuse it, so goals keep their meaning
    ros2 launch kaya_bringup nav.launch.py map_path:=/home/orin-nano/kaya_maps/home

Without map_path the origin is wherever the robot happens to be switched on,
so a goal recorded yesterday means nothing today. With it, cuVSLAM localizes
into the saved coordinate system -- measured on this robot as recovering to
within 7 mm from a pose hint that was 1.8 m wrong.

Then send a goal (RViz "2D Goal Pose", or):
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \\
        orientation: {w: 1.0}}}'

Composition, and why each piece is here:

  SLAM      supplies map -> odom (and, on the cuVSLAM route, odom -> base_link)
  nvblox    turns depth into a TSDF -> ESDF -> a 2D costmap slice. This robot
            has NO LIDAR, so nvblox is the sole source of obstacles.
  Nav2      behaviour tree, Smac planner, MPPI controller

Two settings in the configs matter far more than the rest, and both are
places the stock defaults are wrong for THIS robot:

  nav2.yaml    motion_model: "Omni"  (stock is "DiffDrive", which forbids
               lateral velocity and would waste the omni base entirely)
  nvblox.yaml  ESDF slice 0.05-0.20 m  (stock is 0.0-1.0 m sliced at 1.0 m,
               which is aimed a metre over Kaya's head)

NOTE on startup time: nvblox allocates its voxel layers on the GPU at boot
and this takes roughly 2 minutes on this board -- measured, not estimated.
It is not hung. Wait for the costmap to appear before sending a goal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaya_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    nvblox_cfg = os.path.join(bringup_share, 'config', 'nvblox.yaml')
    nav2_cfg = os.path.join(bringup_share, 'config', 'nav2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='cuvslam',
                              description='cuvslam (stereo IR + IMU, the real target) or '
                                          'rtabmap (RGB-D reference route).'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Set false to bring up SLAM + nvblox only, e.g. to '
                                          'inspect the costmap before letting the robot plan.'),
        DeclareLaunchArgument('map_path', default_value='',
                              description='Load a saved cuVSLAM map and localize in it, so goals '
                                          'keep their meaning across restarts. Empty starts a '
                                          'fresh coordinate system. Save one with kaya_save_map.'),
        DeclareLaunchArgument('hint_x', default_value='0.0',
                              description='Rough starting pose in the saved map (a hint only).'),
        DeclareLaunchArgument('hint_y', default_value='0.0'),
        DeclareLaunchArgument('hint_yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # --------------------------------------------------------------- SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'slam_cuvslam.launch.py')),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('slam'), 'cuvslam')),
            launch_arguments={
                'depth_for_nvblox': 'true',
                'map_path': LaunchConfiguration('map_path'),
                'hint_x': LaunchConfiguration('hint_x'),
                'hint_y': LaunchConfiguration('hint_y'),
                'hint_yaw': LaunchConfiguration('hint_yaw'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'slam_rtabmap.launch.py')),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('slam'), 'rtabmap')),
        ),

        # ------------------------------------------------------------- nvblox
        # The cuVSLAM route runs the camera with depth OFF (it only needs the
        # stereo IR pair), so nvblox needs depth re-enabled to have anything to
        # fuse. That is handled in slam_cuvslam.launch.py via its own args --
        # see the depth_for_nvblox argument there.
        Node(
            package='nvblox_ros', executable='nvblox_node', name='nvblox_node',
            output='screen',
            parameters=[nvblox_cfg],
            # The camera_0/ prefix is NOT decoration: nvblox namespaces its
            # inputs per camera, and num_cameras is 1 here, so camera index 0
            # is the only one it subscribes to. Remapping the un-prefixed
            # 'depth/image' silently connects NOTHING -- the node starts, runs,
            # reports healthy, and logs "Last view not set for sensor type.
            # Decaying all voxels" forever while the costmap stays empty.
            # Confirmed against nvblox_ros's own integration tests, which
            # remap into exactly these names.
            remappings=[
                ('camera_0/depth/image', '/camera/depth/image_raw'),
                ('camera_0/depth/camera_info', '/camera/depth/camera_info'),
                ('camera_0/color/image', '/camera/color/image_raw'),
                ('camera_0/color/camera_info', '/camera/color/camera_info'),
            ],
        ),

        # --------------------------------------------------------------- Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            condition=IfCondition(LaunchConfiguration('nav2')),
            launch_arguments={
                'params_file': nav2_cfg,
                'use_sim_time': 'false',
                # SLAM already provides map -> odom; Nav2 must not start its own
                # localisation or map server on top of it.
                'autostart': 'true',
            }.items(),
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='log',
        ),
    ])
