#!/bin/bash
# Start/stop the stack so that STOPPING ACTUALLY STOPS IT.
#
# `ros2 launch` spawns composable-node containers as separate processes. Killing
# only the launch process orphans them, and they are not small: ten orphaned
# nvblox_node processes at ~800 MB each filled this 7.5 GB board and drove load
# average to 49, which then looked like cuVSLAM hanging. Launch into its own
# session so the whole process group can be signalled as one.
LOG=${LOG:-$HOME/kaya_run.log}
PGF=$HOME/.kaya_run.pgid

start() {
  stop
  rm -f "$LOG"
  setsid nohup bash -c "source /opt/ros/jazzy/setup.bash && \
    source $HOME/ros2_ws/install/setup.bash && \
    exec ros2 launch kaya_bringup nav.launch.py $*" > "$LOG" 2>&1 < /dev/null &
  echo $! > "$PGF"
  echo "started pgid $(cat $PGF), log $LOG"
}

stop() {
  if [ -f "$PGF" ]; then
    kill -TERM -"$(cat $PGF)" 2>/dev/null
    sleep 5
    kill -9 -"$(cat $PGF)" 2>/dev/null
    rm -f "$PGF"
  fi
  # Belt and braces: anything left from an earlier, less careful session.
  #
  # Matched on the EXECUTABLE path, never on a bare word. `pgrep -f` searches
  # whole command lines, so a pattern like "nvblox" also matches the very
  # command invoking this script -- `kaya_run.sh start nvblox_map:=...home.nvblx`
  # contains it -- and the sweep then kills its own ssh session before it can
  # start anything. That happened, and the launch silently never ran.
  #
  # Excluding this shell and its parent as well, so a future pattern that
  # over-matches cannot repeat it.
  self="^($$|$PPID)$"
  pids=$(pgrep -f "nvblox_ros/nvblox_node|/component_container|kaya_bringup.origin_anchor|bin/ros2 launch kaya_bringup" \
         | grep -Ev "$self" | tr "\n" " ")
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  sleep 2
  echo "stopped; $(free -m | awk "/^Mem:/ {print \$7\" MB available\"}")"
}

case "$1" in
  start) shift; start "$@" ;;
  stop) stop ;;
  *) echo "usage: kaya_run.sh start [launch args...] | stop"; exit 1 ;;
esac
