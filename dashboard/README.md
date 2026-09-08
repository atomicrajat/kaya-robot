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

## The robot model

`static/kaya_robot.glb` is the Fusion CAD model, shown in the Robot panel and
used as the marker in the 3D map view. It is generated from
`~/work_projects/kaya_robot/kaya_robot_ttb.usdz`:

    cd ~/isaac-sim && ./python.sh <repo>/dashboard/usd_to_glb.py

Nothing on this machine could read the source formats — no blender, assimp,
usdcat or pxr, and neither `.f3d` (a Fusion archive) nor `.usdz` loads in a
browser. The USD Python bundled inside Isaac Sim can, so the converter reads
the stage with that and writes the glTF binary itself.

Three things it has to get right, each of which is silently wrong otherwise:

- **Units.** The stage is in centimetres (`metersPerUnit` 0.01). Everything else
  in this project is metres.
- **Up axis.** USD is Z-up, three.js is Y-up: `(x, y, z) -> (x, z, -y)`.
- **Materials.** The stage carries 20 real materials — AluminumSatin,
  BrassPolished, PlasticMatteYellow, RubberSoft. A first pass ignored them and
  emitted one flat grey, which throws away the only thing that makes the model
  readable at 200 px.

And one that bites in the browser rather than the converter: those materials are
genuine PBR, with metalness 1.0 on the aluminium and brass. **A metal with
nothing to reflect renders black in three.js**, so both renderers set
`scene.environment` from a PMREM'd `RoomEnvironment`. Without it the model is a
silhouette and looks like a failed load.

`three.min.js`, `GLTFLoader.js`, `RoomEnvironment.js` and the `.glb` are all
vendored under `static/` rather than pulled from a CDN, so the dashboard works
on a network with no route out — which is the normal case for a robot.
