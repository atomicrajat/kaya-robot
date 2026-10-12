# Chapter 6: the ball, from a camera frame to a goal on the real floor

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: `kaya_lab/LEARNING_LOG.md` from "Can the camera see the ball" to the end;
`SIM_RL_PLAN.md` sections 2 and 2b; commits `b9f9718` to `3c63211`; the run logs and
contact sheets in `kaya_lab/data/`.*

## 6.1 Design the task around the camera

The demo: a yellow ball and a goal somewhere on the floor; Kaya must find the
ball, get behind it, push it, and put it in. The camera faces forward only
and sits 17.5 cm up, pitched 4.89 degrees nose-up, so the floor is visible
only from about 0.39 m ahead. The first question was therefore whether the
ball is visible **while being pushed**, because a policy blind at the moment
it needs to steer learns nothing.

Real frames were taken with the ball at 0.5 m, at 10 cm, and touching the
front in three positions. Touching, the ball is a cut-off yellow cap in the
bottom 20% of the image, 15-17% of its disc, top edge at rows 381-387. Never
hidden. And the two front wheels bound it: pushed sideways until it meets a
wheel, the ball moves only about 4 cm from centre, so the gap between the
wheels is a shallow natural pocket about 12 cm wide. The simulator must keep
ball-against-wheel contact or it trains on a different robot.

**The ball's size came from geometry.** A 15.3 cm ball keeps its top below
the camera and blocks only the bottom quarter of the view; an 18 cm ball
would sit level with the lens and block 45%; a tennis ball disappears when
close. The box goal (46 cm wide, 30 tall, 30 deep, open-fronted, on its
side) replaced a wide frame because the whole ball fits inside, so success
is unambiguous and the ball cannot bounce back out.

## 6.2 Measuring the ball in the image: check the method, not just the number

The detector finds yellow with an HSV threshold (hue is colour independent
of brightness, so "yellow" is one hue band in any light) and closes the gaps
the ball's black stripes cut into the blob. Then it must turn a blob into a
position, and three methods were tried:

1. Distance from blob width, the pinhole model. Right for a whole ball;
   wrong the moment the image edge cuts it (it said 0.18 m for a touching
   ball).
2. A circle fitted to the top arc. Better, but a sphere far off the optical
   axis projects as a stretched shape, not a circle.
3. **The exact silhouette of a sphere of known radius resting on the floor**,
   fitted by least squares to the blob's edge pixels with image-border pixels
   excluded (they are not a real edge). This fit first had a sign error
   (camera tilted down instead of up), caught because it fitted the *easy*
   whole-ball frame badly; then it fell into wrong local minima until a
   coarse grid search chose the starting point.

Two lessons that recur for the rest of the chapter: **sanity-check a new tool
on the case where you know the answer**, and **depth cannot help up close**
(the Gemini's depth starts at 0.17 m; a touching ball's surface is 5 cm from
the lens), so close range comes from the colour silhouette alone.

The sim was then checked against those frames: seventeen simulated Kayas
creeping into balls at different lateral offsets contact them across exactly
the pocket the real frames measured (-8.4 to +3.6 cm from the colour lens),
with depth agreeing to about a centimetre.

## 6.3 The policy never sees an image

Rendering a camera for thousands of environments is far too slow. Training
instead computes **what the detector would output** from the true ball
position: projected through the colour camera's real intrinsics at the
measured lens pose; counted as visible only if at least 12 pixels land on
the image; range noise of 1 cm plus 3% (plus 2 cm when cut off), bearing
noise 0.7 degrees, 5% random dropouts, and one step (100 ms) of latency.
Checked against the real frames: a touching ball's top edge lands at rows
383 and 387 in sim against 383 and 387 real; at 0.5 m the ball is 122 pixels
across against 124.

On the robot the real detector produces the same numbers from real frames.
Because the policy only ever sees "ball at (x, y), seen or not", the
detector can be swapped without retraining, which is exactly what happened
later when colour alone proved too easy to fool.

**Teacher, then student.** B1 trained a teacher told the true ball position
at all times: with a curriculum, 99.9% from the hardest starts. B2 trained
the student that sees only the detector output. Two ideas made the hard part
(partial observability) cheap:

- **Memory without a recurrent network.** When the ball is detected its
  position is stored in the odometry frame; every step it is re-expressed in
  the robot's current frame together with "seconds since last seen". Turn
  away and the remembered ball stays put; let the memory go stale and the
  policy learns to go and look. The real robot has odometry, so this carries
  over directly.
- **An asymmetric critic.** The actor (deployed) sees 15 numbers; the critic
  (training only) sees those plus the true ball position and velocity. Better
  value estimates make learning faster, and the critic is discarded at
  deployment, so the "cheat" never reaches the robot.

The student matched the teacher: 99.9-100%, 14.3 s per goal against 13.6,
and the fraction of time it kept the ball in view rose from 57% to 83% with
nobody telling it to. In the replay a disc shows the policy's *belief* about
the ball (green seen, red remembered), one of the best debugging tools
there is.

**Curriculum learning.** From a random start, a random policy almost never
scores and never learns what scoring is worth. So starts have a level from
0 (ball at the mouth, robot right behind it) to 1 (ball up to 2.3 m out,
robot anywhere); recent success above 75% raises it, below 30% lowers it.
Success hovers near 75% by design while the level climbs: **read the level,
not just the success rate.** And the level lives in the environment, not the
policy, so a plain replay restarts at level 0.

**A checker that could not see success.** The first scripted sanity check
reported 0% scored, with every ball stopping at +7.3 cm against a +7.65 cm
threshold. Too close to be coincidence. The environment resets a scored
episode inside `step()` and clears the flag before the script looks.
Reading the +100 reward instead gave 95%. **When a result sits suspiciously
close to a threshold, suspect the measurement.**

## 6.4 Full fidelity before the real robot

B4 ran the unchanged B2 policy on **rendered** camera frames through the real
image detector, with full roller physics and the real box: 90.6% against
97% with the stand-in. Four bugs were caught there instead of on the floor:
the camera's pose went stale after a reset (the detector thought the camera
was 1.9 m away); every camera saw the neighbouring arenas' balls on one open
floor; **brown cardboard passed the colour threshold** (rendered hue 17-19
against a threshold starting at 18); and debug markers rendered into the
robot's own camera. Numbers said something was wrong; looking at the
rendered frames and masks said what.

## 6.5 The real robot: it worked, and everything around it lied

A policy trained only in simulation scored with the real ball in the real
box on runs 1, 2 and 7 (29, 35 and 21 seconds). The learned behaviour carried
over intact: get behind the ball, push, strafe to line up. Every failure was
in perception or localisation, not in the policy, which is the typical
sim-to-real story.

**Measure the physical thing first.** No real frame would fit the sphere
model, and three theories followed (lens distortion, stripes and glare,
camera height). The ball was 14.0 cm, not 15.3. A tape measure settled it,
the fits dropped to 0.24-0.86 degrees, and the policy was fine-tuned for 50
iterations on the right size.

**Perception failed in order, each with a fix:**

1. Colour alone was fooled by warm light on wood and cardboard.
2. A roundness gate was set too strict from guesses; the real ball close up
   and moving fits at 1.8-3.1 degrees, phantoms at 12-17. Thresholds come
   from measured data.
3. "Biggest blob" let a lit cardboard box hide the real ball; look at several
   candidates.
4. **YOLO11s**, pretrained on COCO's "sports ball", on the Jetson's GPU
   through TensorRT with plain ctypes calls into the CUDA runtime (6.4 ms;
   the robot has no PyTorch). It finds the ball and ignores cardboard, but
   misses a ball cut off by the image edge, so colour is kept only for the
   touching zone, and the sphere fit runs inside YOLO's box.

**Localisation.** cuVSLAM jumped 0.3-2 m with the camera facing a plain
wall. For a short game on a small floor, wheel odometry drifts slowly and
never jumps: pick the sensor for the job. Later, turning came from the
camera's gyro (the spin test: gyro +170.7, wheels +171.7, cuVSLAM +178.7
degrees against about 172 commanded) and distance from the wheels.

**Speed.** At 0.15 m/s a push sent the ball 60 cm in 4 s and it ran off.
Retrain at the new top speed rather than scaling actions on the robot; a
scaled policy's beliefs about how its own commands move it are wrong.

Tools that paid for themselves: a dry-run mode; saving a snapshot of every
rejected detection; logging the detection source and fit residual every
step; odometry-jump and play-area stops.

## 6.6 Play like a footballer: v2

On the real floor a pushed ball drifts sideways, and once it is against a
wall or in a corner the v1 policy could not get it back. Not bad luck: v1
arenas had no walls (a ball leaving the area ended the episode), and the
policy only ever pushed. **A policy is only good at situations it trained
on.** v2 trains in rooms: four walls redrawn every episode, up to four
obstacles whose number grows with the curriculum, and balls started against
walls and in corners. Avoiding walls needs to *see* walls, so the policy gets
a 16-ray depth scan taken at camera height (it passes over the ball) that is
one row of the real depth image. The scan is computed analytically and was
verified against brute-force ray marching to 0.2 cm. One engineering trap:
Isaac Lab imports a task's config module before the simulator starts, so a
config in a file that imports USD code crashes start-up. Configs go in
config-only files.

v2 on the robot: 9.2 and 9.8 s straight, and off the right wall in 13.7 s,
exactly what v1 failed. The corner beside the box went 0 for 3, for three
different reasons: YOLO scored the ball at exactly 0.30 against a 0.30
threshold for 33 s (a probe that saves the frame and prints every candidate
showed it in one shot); two false goals, first from a loose scored check,
then from 20-30 cm of wheel drift after hard strafing (**when the robot's
estimate and your eyes disagree, your eyes win**, and the lasting fix was to
see the box through printed ArUco markers); and the robot jamming the ball
against the side panel.

**A wrong hypothesis caught before training on it.** The jam looked like a
physics gap: surely the sim ball was too rigid and slippery to jam. Made
squashier and grippier, a scripted push test showed the *old* ball already
jammed 90% of the time and the new one 72%. The physics was never the gap;
the missing pieces were the situation (balls rarely started beside a panel)
and a reason to stop pushing (a jam cost only time). v2.1 added both and
kept v2's ball. **Test that a fix does what you think before you train on
it.**

**Stall detection from the IMU.** Pinned against a wall, the wheel encoders
report 0.08 m/s of motion and the camera sees plain cardboard: both senses
lie. The IMU does not: a wall hit is a -1.27 m/s² spike, a free ball push has
none. Averaging over three samples read floor vibration as impacts (552 of
589 steps stalled); 0.1 s of averaging and a 3 s hold while the command
keeps pushing the same way fixed it. And while stalled the policy's speed
input is set to zero, because in sim a blocked robot's speed *is* zero. Feed
the policy what the sim would.

**v2.2 and v2.3, reward design.** A ball pushed square-on into a surface is
driven into it; struck off-centre by a front wheel it has a side to escape
to. v2.2 penalises pushing a centred ball that is right next to a surface.
v2.3 encodes the user's recipe, "step back, then strike it with a wheel,
then come back": the jam penalty escalates with every consecutive jammed
step, and a one-off kick bonus pays only if the robot first backed off and
the ball then moved off while sitting in front of a front wheel. **RL
optimises what you pay for, not what you meant**; "ball moved after a jam"
would also pay a robot that shoves harder and gets lucky.

Reading the errors even when training works: every run since B1 printed
"Patch buffer overflow" once at the first reset. PhysX was dropping
contacts on that step (the v2 rooms asked for 1.08 million contact patches
and got 262k). Harmless there, but a dropped contact can let a ball fall
through a wall. The buffer was raised.

## 6.7 The video beats the log

Run 20's log said the robot dribbled the ball 1.5 m along the wall. The
user's video showed it pinned beside the box for 25 s, not moving: the
wheels spun while strafing and wheel odometry "drove" it away. A change to
the stall logic had just been made on the strength of a log; the video
proved it wrong and it was reverted the same hour. From then on every run
was filmed, and 1 frame/s contact sheets sit beside each log in the repo.

The ball was also measured properly: 1.7 kg on a board squashed it from 14.0
to 12.5 cm, which is 2,200 N/m per contact, and it never slides on cardboard,
so its grip is about 1.0. For an air ball that puts it at only about 0.7 psi
above the room: a soft ball. Why a ball locks in a corner: pressed against
the wall, the panel, the floor and the robot at once, no rolling direction
works at every contact and a grippy ball cannot slide instead; pushing
harder tightens the lock, backing off releases it, so "relax, then hit" is
physically right.

A corner evaluation in sim (512 episodes each, every ball in the corner
between a panel and the goal wall) showed v2.3 kicks three times as often as
v2.2 without scoring more, that the measured ball makes the corner harder
(80% against 85%, twice as slow) but not impossible, and that **the sim was
at 80% while the real robot was 0 of about 8**. The rest of the gap was what
the robot *believed*: in run 20 the box position went 23 s stale after every
marker was rejected as a jump following the fake drift. The evaluation
itself nearly lied: stopping at the first 512 finished episodes drops the
slow ones still running, which are mostly failures, and flatters every
policy. It now counts exactly the first two episodes of every robot.

## 6.8 Sixteen filmed runs

The pattern of the night of 2026-09-28/29: run, watch the video, find the one
thing that went wrong, fix it, run again. Almost every fix was in the robot's
own code, not the policy, which kept doing sensible things with wrong inputs.

- **Everything the policy sees is a claim, so check it.** In sim the inputs
  are true by construction. On the robot each one comes from a sensor plus
  code, and each lied: the depth scan read "3 m clear" while pressed against
  a panel (plain cardboard closer than 0.3 m returns nothing with the
  projector off; the box's part of the scan is now computed from its marker
  pose); the detector found "balls" on cardboard; the heading lost 65 degrees
  in a spin; a filter threw the real ball away as "outside the arena".
- **Guards against one failure cause the next one.** "The ball must be in
  front" (against phantoms) rejected a real ball behind the robot. "Colour
  needs YOLO" (against cardboard) rejected a real ball too far for YOLO.
  "The robot must be outside the box" (against a false goal) rejected a real
  goal where the robot's centre was 9 cm past the mouth. Each guard's comment
  names the run it came from so the next failure can be traced to it.
- **A queue of one is a bug waiting for load.** The IMU subscription kept
  only the newest sample. Fine while idle; once detection took 50-170 ms it
  dropped most of the 200 Hz gyro stream, and in a fast search spin the
  heading lost 65 degrees, the box re-anchored 1.76 m away, and the robot
  pushed the ball the wrong way.
- **A search behaviour came for free.** With the ball behind it and nothing
  in view, v2.3 spun to look, found it 1.3 m behind, dribbled it 2.5 m and
  scored. Balls are often out of view in sim, so it had learned to look.

v2.3 scored 3 real goals in 11 runs. The one failure left matched the user's
words exactly: "it always tries to goal from the side, hence it gets stuck".
The ball clips the panel's front corner and ends up pressed against its
outside. v2.4 trains that start (balls beside the box up to 35 cm in front
of the mouth), with the measured ball, the real depth blindness under 0.3 m,
and a penalty for the ball against a panel's outside.

**v2.4 on the real robot: 3 goals in 4.** Beside the box on the right
(pinned at the corner, backed off, came round the front, 49.7 s); from behind
on the left (brought the ball across the mouth before going in, 42.0 s); and
out of the corner between the panel and the back wall (73.7 s). The fourth
lost time with the wheels snagging on the goal's side panels while turning,
which in sim had been free because the bump penalty never counted the panels
and rollers slide off rigid cardboard. v2.5 fixes that on paper and is
untrained.
