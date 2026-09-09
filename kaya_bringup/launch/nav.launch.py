"""Phase 5: autonomous navigation -- SLAM + nvblox + Nav2.

    ros2 launch kaya_bringup nav.launch.py
    ros2 launch kaya_bringup nav.launch.py slam:=rtabmap    # RGB-D route

Building a map (nothing forgotten) vs. navigating (people not fused in):

    ros2 launch kaya_bringup nav.launch.py mapping:=true nav2:=false   # map it
    ros2 launch kaya_bringup nav.launch.py map_path:=...               # use it

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
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('kaya_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    # Two nvblox configs, because building a map and navigating with one want
    # opposite behaviour. Navigation uses dynamic mapping so people are not
    # fused in as walls; that works by DECAYING voxels, which also deletes
    # everywhere the robot drove away from. Mapping therefore uses static_tsdf
    # with decay off. See config/nvblox_mapping.yaml for the measurements.
    nvblox_cfg = PythonExpression([
        "'", os.path.join(bringup_share, 'config', 'nvblox_mapping.yaml'),
        "' if '", LaunchConfiguration('mapping'), "' == 'true' else '",
        os.path.join(bringup_share, 'config', 'nvblox.yaml'), "'"])
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
        DeclareLaunchArgument('mapping', default_value='false',
                              description='Set true for a deliberate MAPPING run: static_tsdf '
                                          'with decay disabled, so nothing is forgotten behind '
                                          'the robot. Leave false to navigate, which uses '
                                          'dynamic mapping so people are not fused in as walls. '
                                          'See config/nvblox_mapping.yaml.'),
        DeclareLaunchArgument('nvblox_map', default_value='',
                              description='A saved .nvblx volume to reload, so the costmap holds '
                                          'the whole space instead of only what the camera has '
                                          'seen this run. This is SEPARATE from map_path: that '
                                          "loads cuVSLAM's relocalisation database (where the "
                                          'robot is), this loads what the space looks like. They '
                                          'live side by side in a saved map directory, e.g. '
                                          'map_path:=~/kaya_maps/home_final/cuvslam with '
                                          'nvblox_map:=~/kaya_maps/home_final/home.nvblx. Giving '
                                          'only map_path leaves the robot correctly localized on '
                                          'a map that has forgotten the building.'),
        DeclareLaunchArgument('map_2d', default_value='',
                              description='A saved 2D grid (.yaml from save_2d_map) to serve as '
                                          'the global costmap static layer, INSTEAD of reloading '
                                          'the 3.3 GB nvblox volume. The same 202 m^2 costs '
                                          '115 KB this way, and nvblox still supplies live '
                                          'obstacles on '
                                          'top, so furniture moved since the map was built is '
                                          'still avoided. Pair with nav2_params pointing at '
                                          'nav2_static_map.yaml, and leave nvblox_map empty.'),
        DeclareLaunchArgument('nav2_params', default_value=nav2_cfg,
                              description='Nav2 parameter file. Override to nav2_static_map.yaml '
                                          'in this package to add the static_layer that map_2d '
                                          'feeds.'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Serve the stack to Foxglove Studio over the LAN, for '
                                          'a live map and robot position without an X session on '
                                          'the robot. Connect the desktop app to '
                                          'ws://192.168.1.28:8765. Off by default until the cost '
                                          'to VIO of a subscribed client is measured on a driven '
                                          'loop -- see the note at the node.'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

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
                'params_file': LaunchConfiguration('nav2_params'),
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

        # Reload the saved nvblox volume once nvblox is up. See the node for
        # why this is a node rather than a delay, and why its success flag can
        # be trusted where cuVSLAM's cannot.
        Node(
            package='kaya_bringup', executable='nvblox_map_loader',
            name='nvblox_map_loader', output='screen',
            parameters=[{'nvblox_map': LaunchConfiguration('nvblox_map')}],
        ),

        # ------------------------------------------------- saved 2D grid
        # Serves the mapping run's occupancy grid as /map for the global
        # costmap's static_layer. map_server is a lifecycle node and does
        # nothing until something activates it, hence its own manager --
        # nav2's navigation_launch.py manages only the navigation nodes.
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('map_2d'), "' != ''"])),
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map_2d'),
                         'use_sim_time': False}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('map_2d'), "' != ''"])),
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server'],
                         'use_sim_time': False}],
        ),

        # Cheap enough to leave on: sysfs reads at 1 Hz, no tegrastats process.
        Node(
            package='kaya_bringup', executable='jetson_stats',
            name='jetson_stats', output='log',
        ),

        # ----------------------------------------------------------- Foxglove
        # A websocket view of the running stack, so the map and the robot's
        # position can be watched from the workstation. RViz on the robot needs
        # an X session and pushes rendering onto the same GPU cuVSLAM and nvblox
        # are using; this pushes serialised messages over the LAN instead.
        #
        # The bridge only serialises topics a connected client has actually
        # SUBSCRIBED to, so its cost is set by which panels are open, not by
        # what is whitelisted. Measured on this board, stationary, full nav
        # stack up, against a client subscribed to the worst case at once --
        # nvblox mesh, depth point cloud, colour image and map slice, 15.4 MB/s
        # over the LAN:
        #
        #     track_execution_time_mean   6.79 ms -> 7.14 ms
        #     bridge CPU                  3.7%    -> 6.0%
        #     memory used                 3657 MB -> 3881 MB
        #
        # Cheap enough to leave connected, and an idle bridge costs ~3.7% CPU.
        # But that is a STATIONARY measurement, and this project has been caught
        # by exactly that gap before: vo_state and frame times reported perfect
        # health all through the time_domain regression while raw drift went to
        # 15-32%. The honest test is raw VIO drift on a driven loop with a
        # client connected, and it has not been run yet -- which is why this
        # argument still defaults to false.
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge',
            condition=IfCondition(LaunchConfiguration('foxglove')),
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('foxglove_port'),
                # 0.0.0.0 so the workstation can reach it; the robot is on a
                # home LAN behind a router, not an open network.
                'address': '0.0.0.0',
                'use_sim_time': False,
            }],
        ),
    ])
