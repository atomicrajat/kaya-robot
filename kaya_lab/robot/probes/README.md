# One-off probes

Measurement scripts from the ball-demo nights (2026-09-28/29). None is needed
to run the demo; each answered one question, and the answer is in
`kaya_lab/LEARNING_LOG.md`. Run them on the robot from this folder with the
ball stack up (`../run_stack.sh start`); they find the node's modules by path.

| script | question it answered | moves the robot? |
|---|---|---|
| `probe_detect.py` | what does the detector see in ONE frame: every YOLO candidate, every colour blob and its sphere fit (found the ball YOLO scored at exactly 0.30, run 12) | no |
| `probe_depth.py` | how much of the depth image is valid, row by row, and the 16-ray scan built from it (found depth blind to plain cardboard < 0.3 m) | no |
| `probe_ir.py` | saves the IR image and a depth picture, for "why is depth empty" | no |
| `probe_marker.py` | the goal box pose from each visible ArUco marker, three frames | no |
| `spin_test.py` | gyro vs wheels vs cuVSLAM over a 0.5 rad/s spin: the gyro's sign and scale (+170.7 / +171.7 / +178.7 deg against ~172 commanded) | yes, 6 s |
| `stall_probe.py` | what each signal does when the robot drives into a wall: the IMU's -1.27 m/s^2 spike is the only honest one | yes, drives at a wall |
| `nudge.py` | a fixed small move, then stop: `python3 nudge.py vx vy w seconds` | yes |
