# Dashboard

Live map, live robot position, click to navigate, and the place the Phase 10
question box will live.

    source /opt/ros/jazzy/setup.bash
    python3 dashboard/kaya_dashboard.py      # http://localhost:8080

## It runs on the workstation, not the robot

Both machines run Jazzy on `ROS_DOMAIN_ID` 0 on the same subnet, so the
workstation sees all 151 of the robot's topics directly over DDS. The dashboard
subscribes from here and costs the Jetson nothing beyond traffic it already
publishes. That matters: the roadmap's rule is that navigation is real-time and
nothing else may compete with it.

## Why not Foxglove

`foxglove_bridge` is wired into `nav.launch.py` as `foxglove:=true` and it works
for TF, images and standard messages. It cannot show the map. Foxglove Studio
3.0.0 reports

    Failed to parse channel schema on /nvblox_node/static_map_slice
    Unsupported message encoding

for every nvblox topic and for `/visual_slam/status`, because those carry custom
`nvblox_msgs` / Isaac types. No panel setting fixes that.

## What it shows, and what it deliberately does not

| layer | source | why |
|---|---|---|
| base map | `robot_maps/home_final/home_2d.pgm` | the whole 202 m², static, free |
| live obstacles | `/global_costmap/costmap` | what Nav2 is planning on **now** |
| robot pose | TF `map -> base_footprint` | |
| goal | publishes `/goal_pose` | click to place, drag to aim heading |
| 3D | walls extruded from the base map | same source as 2D, so they cannot disagree |

Both map layers are plain `nav_msgs/OccupancyGrid` or a file. `nvblox_msgs` is
not installed on the workstation, so the custom slice topics are unavailable
here at all — which is also why the 3D view extrudes the 2D grid rather than
shipping the 42 MB `.ply` mesh to a browser.

**The VIO figure reads "n/a here" on purpose.** `isaac_ros_visual_slam_
interfaces` is installed only on the robot, so `track_execution_time_mean`
cannot be read from the workstation. It would be easy to show something else
and call it VIO health; this project has been burned enough by numbers that
report health instead of truth. Read it on the robot.

`/api/ask` returns 501 for the same reason — Phase 9 (keyframes) and Phase 10
(the reasoning model) are not built, and an endpoint that invents an answer is
worse than one that says it is not built.
