# Servo tools

Run on the robot after `source ~/ros2_ws/install/setup.bash` (they import the
driver's `kaya_drive.feetech_bus`). Neither is installed by colcon; run them
from the source tree.

| script | what | status |
|---|---|---|
| `wheel_probe.py` | pulses ONE servo (`<id> <signed_speed> <seconds>`), printing every write's result, the Mode read-back and the encoder during the pulse, so you can see which wheel each ID drives | the script that resolved the mapping on 2026-09-03; one import line changed to use the vendored SDK |
| `servo_set_id.py` | writes a new bus ID to one servo's EEPROM, with read-back | untested on hardware; new servos ship as ID 1, connect one at a time |

The mapping the kinematics assume: servo 1 = front-right (300 deg), 2 =
front-left (60 deg), 3 = rear (180 deg), forward being where the camera looks.
See `GETTING_STARTED.md` step 4.
