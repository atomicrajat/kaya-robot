#!/bin/bash
# Stop any running Isaac Lab train/play started by rl.sh, and wait until the GPU is free.
# The match pattern lives in this file, not on the command line: `pkill -f "<pattern>"`
# typed into a shell matches that shell's own command line and kills it first.
pat='kit/python/bin/python3 scripts/reinforcement_learning/rsl_rl/(train|play)\.py'
pids=$(pgrep -f "$pat" | grep -Ev "^($$|$PPID)$")
[ -z "$pids" ] && { echo "nothing running"; exit 0; }
kill -INT $pids 2>/dev/null
for i in $(seq 30); do pgrep -f "$pat" >/dev/null || break; sleep 1; done
pgrep -f "$pat" >/dev/null && kill -9 $(pgrep -f "$pat") 2>/dev/null
sleep 2
echo "stopped; GPU memory in use: $(nvidia-smi --query-gpu=memory.used --format=csv,noheader)"
