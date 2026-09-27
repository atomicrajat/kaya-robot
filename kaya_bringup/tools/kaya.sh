#!/bin/bash
# Bring up Kaya: robot stack, then the dashboard, from the workstation.
#
#     ./kaya.sh            start everything
#     ./kaya.sh stop       stop everything
#     ./kaya.sh status     what is running
#
# PUT THE ROBOT ON THE MARK FIRST. Relocalisation against the saved map still
# fails (furniture has moved since it was built), so cuVSLAM anchors a fresh
# frame wherever the robot is standing at launch. The keyframe store was
# recorded from the mark, so anchoring anywhere else offsets every remembered
# place by that distance -- and it fails SILENTLY: the robot drives confidently
# to the wrong room with a photo of the right one on the screen.
set -u
# The robot to drive. Defaults to this project's robot; set KAYA_ROBOT for yours,
# e.g. KAYA_ROBOT=me@192.168.1.50 ./kaya.sh
ROBOT=${KAYA_ROBOT:-orin-nano@192.168.1.28}
HERE=$(cd "$(dirname "$0")/../.." && pwd)
DASH="$HERE/dashboard/kaya_dashboard.py"
SSH="ssh -o BatchMode=yes -o ConnectTimeout=20"

dash_pids() { pgrep -f "python3 .*kaya_dashboard.py" 2>/dev/null | grep -Ev "^($$|$PPID)$"; }

stop_all() {
  echo "stopping the dashboard"
  p=$(dash_pids); [ -n "$p" ] && echo "$p" | xargs -r kill -9 2>/dev/null
  echo "stopping the robot stack"
  timeout 90 $SSH "$ROBOT" '~/kaya_run.sh stop' 2>&1 | tail -1
}

case "${1:-start}" in
stop) stop_all; exit 0 ;;
status)
  timeout 30 $SSH "$ROBOT" 'pgrep -c -f "[c]omponent_container" >/dev/null && echo "robot: stack running" || echo "robot: stopped"; free -m|sed -n 2p|awk "{print \"  \"\$7\" MB free\"}"; sudo nvpmodel -q 2>/dev/null|tail -1|sed "s/^/  power mode: /"' 2>&1 | tail -4
  [ -n "$(dash_pids)" ] && echo "dashboard: running  http://$(hostname -I|awk '{print $1}'):8080" || echo "dashboard: stopped"
  exit 0 ;;
esac

echo "=== 1/3  robot stack ==="
timeout 60 $SSH "$ROBOT" 'echo ok' >/dev/null 2>&1 || { echo "cannot reach the robot at $ROBOT -- is it powered and on wifi?"; exit 1; }
timeout 150 $SSH "$ROBOT" '~/kaya_drive.sh' 2>&1 | tail -2 || exit 1

echo "=== 2/3  waiting for the map frame to anchor ==="
for i in $(seq 1 40); do
  sleep 8
  if timeout 20 $SSH "$ROBOT" 'grep -q "map frame is now anchored" ~/kaya_run.log' 2>/dev/null; then
    echo "  anchored after ~$((i*8))s"; break
  fi
  [ "$i" = 40 ] && { echo "  NEVER ANCHORED -- check: ssh $ROBOT 'tail -40 ~/kaya_run.log'"; exit 1; }
done

echo "=== 3/3  dashboard ==="
p=$(dash_pids); [ -n "$p" ] && echo "$p" | xargs -r kill -9 2>/dev/null
sleep 2
setsid nohup bash -c "source /opt/ros/jazzy/setup.bash && cd '$HERE' && exec python3 dashboard/kaya_dashboard.py" > /tmp/kaya_dashboard.log 2>&1 < /dev/null &
for i in $(seq 1 20); do sleep 2; ss -ltn 2>/dev/null | grep -q ':8080' && break; done

echo
if ss -ltn 2>/dev/null | grep -q ':8080'; then
  # The pose is the check that matters: it must read ~(0,0,0), because that is
  # the frame the keyframe store was recorded in.
  sleep 6
  curl -s -m 20 localhost:8080/api/state 2>/dev/null | python3 -c "
import json,sys,math
try: d=json.load(sys.stdin)
except Exception: sys.exit()
p=d.get('pose')
if p:
    off=math.hypot(p['x'],p['y'])
    print('  pose (%+.3f, %+.3f) yaw %+.1f' % (p['x'],p['y'],math.degrees(p['yaw'])))
    print('  %s' % ('ON THE MARK -- keyframe poses are valid' if off < 0.05 else
          'WARNING: %.2f m from the origin. If the robot was not on the mark, '
          'every remembered place is offset by that much.' % off))
print('  camera: %s' % d.get('camera_source'))
" 2>/dev/null
  echo
  echo "  dashboard:  http://$(hostname -I|awk '{print $1}'):8080"
else
  echo "  DASHBOARD FAILED -- see /tmp/kaya_dashboard.log"; tail -5 /tmp/kaya_dashboard.log
fi
