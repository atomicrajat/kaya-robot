# Data

| folder | what |
|---|---|
| `aruco/` | the three printed goal markers (150 mm, DICT_4X4_50) and the 3-page PDF |
| `b5_runs_20260928/` | the log of every real run (1-38) of the ball demo; each run's story is in `../LEARNING_LOG.md` |
| `ball_frames_20260927/` | the two scratch scripts of the first sphere-silhouette fit |

**What is deliberately not here:** the robot's camera frames, the rejected-detection
snapshots and the video contact sheets of the real runs. They are images of
a private home. They live outside the repo (`KAYA_PRIVATE_MEDIA`, by default
`../private_media/` beside the checkout, same paths) and
`scripts/test_ball_detector_real_frames.py` reads them from there. The 2D map
in `docs/images/` is the only view of the space that is published.
