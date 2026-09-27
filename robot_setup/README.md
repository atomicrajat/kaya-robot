# Robot setup

Everything needed to take a freshly flashed Jetson Orin Nano to a working
Kaya - kept in the repo so a clone is enough to rebuild the robot.

| file | what |
|---|---|
| `SETUP_LOG.md` | the full JetPack 6.2.1 -> 7.2.1 + ROS 2 Jazzy + Isaac ROS 4.6 bring-up, every failure and its fix. Read first. |
| `kaya_provision.sh` | runs that log as stages on a fresh flash (`--list`, `--only`, `--skip`); skips what is done |
| `kaya_check.sh` | health check of the board, power mode, camera, servos, wifi |
| `backup_before_flash.sh` | captures what a reflash would destroy and git/apt cannot restore |

Run on the robot. The scripts refer to each other and to `SETUP_LOG.md`
sections by name, so keep them together. They hold no credentials - wifi
details are `<SSID>`/`<PASS>` placeholders, or read from the live system.

Note: `SETUP_LOG.md` still calls the servos Feetech ST3215; they are Waveshare
ST3020 (see `kaya_description/cad/CAD_CONTEXT.md`).
