"""Kaya RL tasks. Isaac Lab's train/play commands load them with

    --external_callback kaya_lab.tasks.register

(with kaya_lab/ on PYTHONPATH - see kaya_lab/scripts/rl.sh).
"""


def register():
    from . import ball_goal, point_goal

    point_goal.register()
    ball_goal.register()
    return None     # no extra CLI arguments of our own
