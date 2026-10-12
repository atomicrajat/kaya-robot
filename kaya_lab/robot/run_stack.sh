#!/bin/bash
# Ball-demo stack: camera + cuVSLAM odometry + drive + robot description (no nvblox, no Nav2).
# Launched into its own session so stop kills the whole process group (see
# kaya_bringup/tools/kaya_run.sh for why that matters on this board).
#
#     kaya_lab/robot/run_stack.sh start      # then start ball_policy_node.py (see kaya_lab/README.md)
#     kaya_lab/robot/run_stack.sh stop
#
# Log: $KAYA_BALL_LOG (default ~/kaya_ball_stack.log).
LOG=${KAYA_BALL_LOG:-$HOME/kaya_ball_stack.log}; PGF=$HOME/.kaya_ball_stack.pgid
stop() { [ -f $PGF ] && { kill -INT -$(cat $PGF) 2>/dev/null; sleep 6; kill -9 -$(cat $PGF) 2>/dev/null; rm -f $PGF; }
  self="^($$|$PPID)$"; pids=$(pgrep -f "/component_container|bin/ros2 launch kaya_bringup|kaya_drive/drive_node|camera_tuner" | grep -Ev "$self" | tr "\n" " ")
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null; sleep 2; echo "stopped; $(free -m | awk "/^Mem:/{print \$7\" MB available\"}")"; }
case $1 in
 start) stop; setsid nohup bash -c "source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash && exec ros2 launch kaya_bringup slam_cuvslam.launch.py color_fps:=15 depth_for_nvblox:=true" > $LOG 2>&1 < /dev/null & echo $! > $PGF; echo "started pgid $(cat $PGF), log $LOG";;
 stop) stop;;
 *) echo "usage: run_stack.sh start | stop"; exit 1;;
esac
