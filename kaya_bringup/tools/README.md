# Diagnostic tools

Small standalone scripts, kept because each one was written to answer a
question that cost real time, and every one of them will be needed again.
None depend on anything but rclpy and numpy. Run them with the stack up:

    python3 tools/<script>.py

`scene_quality.py` -- brightness and texture of the IR image, logged against
pose drift. THE tool for "why is VIO drifting when everything looks healthy".
Texture, not brightness, is what predicts drift: a bright blank wall is
untrackable. Measured on this robot: ~4.6 in a dark room (VIO diverged 1.8 m
in 2 m of driving), ~11.8 with the IR projector on.

`ir_stats.py` -- the same brightness/texture numbers standalone, for comparing
camera configurations without driving.

`stereo_sync.py` -- verifies the IR pair delivers MATCHED frames. Unpaired
frames wreck triangulation while vo_state still reports success. Known-good on
this camera: 419/419 matched, 0.048 ms median.

`drift_log.py` -- pose drift over a there-and-back loop, with Z called out
separately. Z is the honest one: on a flat floor it should read zero, so any
departure is drift you can trust.

`stamp_lag.py` -- how far behind ROS time each transform and image is stamped.
Nav2 rejects stale transforms and reports the failure as "Reached the goal!",
so this is worth checking before believing a navigation result.

`kaya_run.sh` -- start and stop the stack so that stopping ACTUALLY stops it.
`ros2 launch` spawns the composable-node containers as separate processes, so
killing the launch process orphans them, silently. Ten orphaned nvblox_node
processes at ~800 MB each filled this 7.5 GB board, took available memory to
54 MB and load average to 49 -- at which point cuVSLAM stopped reporting
localization outcomes at all and relocalisation returned a confident pose 2.4 m
from truth. Both looked like cuVSLAM bugs and neither was. Launches into its own
session and signals the whole process group.

    ./tools/kaya_run.sh start map_path:=... hint_x:=0.0
    ./tools/kaya_run.sh stop

`slice_render.py` -- renders nvblox's costmap slice as ASCII around the
ROBOT'S ACTUAL POSE, looked up from TF. An earlier version assumed the robot
sat at map (0,0) and reported "0 observed cells" -- indistinguishable from a
broken costmap -- the moment it did not.

`blur_check.py` -- SHARPNESS (variance of the Laplacian) of the IR image, not
just brightness. Motion blur is invisible at rest and destroys tracking the
moment the robot drives: 1.4 mm drift standing still, 15-32% under motion.

`imu_cam_sync.py` -- image-timestamp jitter and IMU-vs-image clock skew.
cuVSLAM fuses a 200 Hz IMU with 30 Hz images; jitter between them is
unmodelled rotation, zero at rest and proportional to error once moving.

`save_2d_map.py` -- writes nvblox's occupancy grid as a standard PGM + YAML
map. nav2's map_saver_cli refuses it (nvblox publishes volatile, the saver
expects latched); subscribing with matching QoS is simpler.

`kaya.sh` -- bring the whole robot up from the workstation: robot stack, then
the dashboard (`./kaya.sh`, `./kaya.sh stop`, `./kaya.sh status`). Put the
robot on the mark first - relocalisation depends on it.

## The measurement lesson worth keeping

Several of these were written after a wrong diagnosis, and the pattern repeats:
a tool that reports a number is not the same as a tool that reports the truth.
`ros2 topic hz` showed the IR pair at 27.9/29.9 Hz and suggested dropped
frames; subscribing properly showed 419/419 matched. cuVSLAM's dropped-frame
warning fires at 34 ms against a 33.3 ms frame period, so half its warnings are
noise. Check what a number is actually measuring before building a theory on it.
