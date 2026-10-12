#!/bin/bash
# Bring the NAVIGATION stack up on the robot with the configuration that worked
# on 2026-09-10 (Phase 10 runs: marked start, kitchen reached to 14 cm), and the
# reason for every flag. Run ON THE ROBOT; kaya.sh calls it over ssh.
#
#     kaya_nav.sh [map_name] [--relocalise]        default map_name: home_final
#     kaya_nav.sh stop
#
# Maps live in ~/kaya_maps/<map_name>/ as written by kaya_save_map:
#     <map_name>_2d.pgm/.yaml   the 2D grid Nav2 plans on
#     cuvslam/                  the relocalisation database (only with --relocalise)
#
# Without --relocalise the map frame is anchored wherever the robot stands at
# launch, so PUT THE ROBOT ON THE MARK the map and the keyframe store were made
# from. With it, cuVSLAM localises into the saved map instead; this failed in
# the project's own flat once the furniture had moved, which is why the
# default is the mark. See ROADMAP.md, "Hard-won settings".
#
# This is the robot's ~/kaya_drive.sh (the file kaya.sh used to call, pulled into
# the repo 2026-10-12), with the map name as an argument and the MAXN_SUPER check
# reading the clock table. The flags and their reasons are unchanged.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
MAPS=${KAYA_MAPS:-$HOME/kaya_maps}

if [ "${1:-}" = "stop" ]; then exec "$HERE/kaya_run.sh" stop; fi
NAME=home_final; RELOC=0
for a in "$@"; do case "$a" in --relocalise|--relocalize) RELOC=1;; -h|--help) sed -n '2,20p' "$0"; exit 0;; *) NAME=$a;; esac; done
MAP2D="$MAPS/$NAME/${NAME}_2d.yaml"
[ -f "$MAP2D" ] || { echo "no 2D map at $MAP2D (save one with kaya_save_map, see GETTING_STARTED.md step 7)"; exit 1; }

# MAXN_SUPER: at 25W cuVSLAM ran at 15 Hz and its input queue grew without bound,
# turning a 0.47 s camera lag into 4 s spikes (a820bbd). Check the clock table,
# not nvpmodel's own report, which lies after the symlink workaround.
GPU_MAX=$(cat /sys/devices/platform/bus@0/*.gpu/devfreq/*.gpu/available_frequencies 2>/dev/null | tr ' ' '\n' | sort -n | tail -1)
if [ "${GPU_MAX:-0}" -lt 1000000000 ]; then
  echo "GPU clock table tops out at ${GPU_MAX:-?} Hz: the board is not flashed with the -super config (robot_setup/SETUP_LOG.md section 3)"; exit 1
fi
MODE=$(sudo nvpmodel -q 2>/dev/null | sed -n 2p)
if [ "$MODE" != "2" ]; then
  echo "switching to MAXN_SUPER (nvpmodel -m 2)"; sudo nvpmodel -m 2 || { echo "nvpmodel -m 2 failed"; exit 1; }
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
ARGS=(
  # the saved 2D grid is the GLOBAL costmap; nvblox is not a costmap source (82ba82b, f31b8a7)
  "nav2_params:=$(ros2 pkg prefix kaya_bringup)/share/kaya_bringup/config/nav2_static_map.yaml"
  "map_2d:=$MAP2D"
  # that config references nvblox nowhere, so it would spend 22% CPU feeding nobody (3aa071a)
  "nvblox:=false"
  # every colour frame is bandwidth taken from the IR pair cuVSLAM tracks on; even 5 fps
  # halved the left eye's rate under load. The dashboard shows the IR stream instead (482c450)
  "enable_color:=false"
)
[ $RELOC = 1 ] && ARGS+=("map_path:=$MAPS/$NAME/cuvslam")
echo "launching: ${ARGS[*]}"
exec "$HERE/kaya_run.sh" start "${ARGS[@]}"
