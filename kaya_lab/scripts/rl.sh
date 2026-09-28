#!/bin/bash
# Train or play a Kaya task with Isaac Lab + RSL-RL.
#
#   kaya_lab/scripts/rl.sh train --task Kaya-PointGoal-v0 --num_envs 256 --viz kit   # GUI: watch it learn
#   kaya_lab/scripts/rl.sh train --task Kaya-PointGoal-v0 --num_envs 4096            # headless (Isaac Lab 3.0 default)
#   kaya_lab/scripts/rl.sh play  --task Kaya-PointGoal-v0 --num_envs 32 --viz kit    # replay the latest checkpoint
#
# Isaac Lab 3.0 is headless unless you pass --viz kit (the old --headless flag is deprecated).
#
# Logs and checkpoints: ~/IsaacLab/logs/rsl_rl/<experiment_name>/<date>/
# Close any other Isaac Sim first: a second instance on the GPU halves throughput.
set -e
MODE=$1; shift
HERE=$(cd "$(dirname "$0")/.." && pwd)                 # kaya_lab/
ISAACLAB=${ISAACLAB:-$HOME/IsaacLab}
export PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}"
cd "$ISAACLAB"
exec ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/${MODE}.py \
    --external_callback kaya_lab.tasks.register "$@"
