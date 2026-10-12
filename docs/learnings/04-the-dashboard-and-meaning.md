# Chapter 4: the dashboard, and giving places meaning

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
Sources: commits `d8488a9`, `e6752ef`, `412f907`, `ea112a5`, `cdb5309`, `29d0b3c`, `b36836e`,
`38ded7a`, `e6b4a3e`, `826e77f`, `1a91641`, `5465836`, `8d932d0`, `b63ba23`, `efe41d9`, `2c52566`;
`dashboard/README.md`, `ROADMAP.md`.*

## 4.1 Why a custom dashboard

The cheap plan was Foxglove: its bridge was already installed, and measured
with a client subscribed to the worst case (mesh, point cloud, colour, map
slice, 15.4 MB/s), it cost cuVSLAM 0.35 ms per frame and 224 MB. Fine. But
Foxglove Studio could not render this robot's map: every nvblox topic and the
SLAM status carry custom message types it rejects with "Unsupported message
encoding", and no panel setting fixes that. The roadmap had always expected a
custom web page eventually, for the question box; it came forward.

It runs on the **workstation**. Both machines run ROS 2 Jazzy on the same
subnet, so the workstation sees all 151 of the robot's topics over DDS, and
the dashboard costs the Jetson nothing beyond traffic it already publishes.
Two map layers, both plain occupancy grids because the custom nvblox types
are not installed off-robot: the saved 202 m² grid (static, the whole
building) and Nav2's live global costmap (the 20 m window it is planning on
now). Showing only one would mislead in opposite directions.

## 4.2 A display that reports health over a dead source

The robot's battery died two hours into a session, and the dashboard went on
showing a confident position and "map frame anchored" for a machine that was
off the network: 100% packet loss, topics down from 151 to 7. Nothing had to
fail for the display to become fiction, because a TF buffer hands back the
last transform it saw forever.

This was the third time that session the same shape of bug had been undone
(the anchor node claiming success on a failed localisation, cuVSLAM claiming
a pose 2.4 m from truth), and this one was self-inflicted hours after the
other two were fixed. The rules that came out of it:

- every pose carries its age, and the link reads live, stale or offline;
- past 5 s the pose is `null` and "anchored" is `unknown`, never the last
  thing the robot said;
- the map greys out under an OFFLINE banner so it cannot be read as
  telemetry;
- a goal posted while nothing subscribes to `/goal_pose` returns 503, because
  publishing into an empty topic succeeds and is indistinguishable from
  working, which is exactly what a dead robot looks like from here;
- the VIO health figure reads "n/a here" rather than substituting some other
  number and calling it VIO; the Isaac types that carry it exist only on the
  robot.

## 4.3 Four small things that cost real time

- Nav2's trajectory visualiser does not publish one line per trajectory. It
  publishes one **sphere marker per sampled state** with the points array
  empty: 7656 markers in one message. The obvious filter for "at least two
  points" drops all of them silently.
- That message must be subscribed **reliable**. It fragments across many
  UDP datagrams and a best-effort subscriber discards the whole sample when
  one fragment is lost; best-effort delivered nothing across an entire
  drive.
- GPU load on Orin is **per mille** in sysfs, not percent; reported raw it
  shows "537% GPU" and is believed.
- The CAD model is genuine PBR with metalness 1.0 on the aluminium and brass,
  and **a metal with nothing to reflect renders black** in three.js. Both
  renderers set an environment map; without it the model is a silhouette
  that reads as a failed load. The converter also had to handle a stage in
  centimetres, Z-up to Y-up, and 20 real materials a first pass had thrown
  away.

## 4.4 Teleop with a deadman

The dashboard's control pad publishes into `/cmd_vel_smoothed`, not
`/cmd_vel`, so the collision monitor still sits between the operator and the
wheels. Driving by hand from a camera view is exactly when a last-resort
safety layer earns its place. And it is a deadman: the browser re-sends the
held command every 150 ms and the backend expires it after 500 ms, so a
closed tab, a frozen page, a dropped link or a lost focus all stop the robot.
Verified: three held commands gave 0.4 rad/s on `/cmd_vel`, and when the
sends stopped the topic went to zero on its own. Manual mode first cancels
whatever Nav2 is doing, so the planner and the operator never drive at once.

## 4.5 Remember scenes; let a model choose

The map is geometry. It knows a wall is at (3.2, -1.4), not that it is a
kitchen. The first plan for "show me where I can cook" was object detection:
YOLO's COCO classes include oven, sink and fridge, so a room could be inferred
from labels. The plan that replaced it is simpler on the robot and more
capable:

    while driving:   save a colour keyframe plus its map pose, every ~1 m or 45 degrees
    after a drive:   a vision model captions each keyframe ONCE
    on a question:   question + captions -> the model picks a place -> its pose goes to Nav2

Detection would have needed a network at 1-2 Hz competing with cuVSLAM for
the GPU, plus depth projection, TF and clustering, and would still have been
limited to 80 classes that cannot express "a cosy corner to read in". Saving
JPEGs costs almost nothing and the vocabulary becomes anything describable.

**The trigger needed a floor.** The first drive produced 428 keyframes over
5.6 x 7.3 m in 15 minutes: 427 fired on "the view changed" and zero on
distance or rotation, because the appearance threshold had no minimum
distance and fired continuously. Those 428 thin to 62 distinct places at the
intended spacing, and a shortlist built from near-duplicates spends a
model's attention on the same wall eight times. A changed view now counts
only after 0.25 m of travel.

**Caption once, query many.** The obvious build sends every keyframe image
to the model with the question attached. At about 2 s of vision inference
per frame that is over a minute per question, every question, worsening with
every drive. Captioning all 62 frames once took 1m58s; a question is then
text against text, about a second, flat in the size of the store. It also
makes the memory inspectable: `captions.json` is readable English, so when
the robot claims the kitchen is at (-2.06, 4.74) you can read what it thought
it saw there.

**It is allowed to say no.** Asked for a swimming pool it answered that no
such place was in the list, rather than picking the nearest-looking room. A
wrong answer here is not a wrong sentence; it is a robot driving into the
wrong room. Answering and driving are separate for the same reason: the
dashboard shows the remembered photo, the pose and the model's reason, with a
separate button to go.

**Where the model runs.** On the workstation, against the instinct to put
models on the robot. Not by preference: the split is by **latency
sensitivity**. Captioning and answering are one-shot with a human waiting,
where a second is not felt. Tracking and following are a control loop, where
latency is error, and those stay on the robot. Two environment variables move
the model to the Orin with no code change.

## 4.6 A keyframe pose is where the camera was, not where the thing is

Asked where to eat and where to cook, the store returned the same pose for
both, and it was in neither room. The frame at (-0.61, 4.73) sits at the
mouth of a corridor and can see a fridge 2.7 m down it; its caption read "a
stainless steel refrigerator against the wall, indicating a kitchen area".
The frame actually standing in the kitchen hedged, "possibly a kitchen or a
hallway leading to a kitchen", because the prompt had told the model to say
"unclear" when unclear. A text match picks the confident sentence, so the
robot parked at the doorway looking in.

The fix is structural, not domain-specific: captions now separate `here`
(what the robot is standing among) from `beyond` (what it can merely see
from there). A fridge in `beyond` is a signpost pointing at the kitchen; a
sink in `here` is the kitchen. The same 62 images re-read with that question
sort themselves out, and all six acceptance questions resolve to different,
correct places.

Two things were tried alongside it and removed. A domain rule ("somewhere to
cook is a sink, stove or counter") made things worse in an A/B against the
same store: it taught the model that a sink means kitchen, and "take me to
the washbasin" then answered with a bedroom. And a note about that wrong
answer, written inside the prompt string as a warning, was read by the model
and reproduced as the answer. Nothing in the prompt names a room, a pose or
an object; the question it asks about distance is true in any building.

**Proven on the robot.** Robot on the mark, fresh frame, anchor at (-0.004,
-0.002) against the store's origin of (-0.008, 0.002): 5 mm, so the recorded
poses were valid. "Where can I eat" picked the dining area, the goal went to
Nav2, and the robot drove 4.8 m to "Reached the goal!". The store's poses
live in the frame of the session that recorded them, which is only reusable
because that drive started on the mark; launch anywhere else and every
remembered place is offset by that distance with no error anywhere.

## 4.7 Open-vocabulary detection, and the follower that stopped the robot by watching it

For "go to the fridge" rather than "go to the kitchen", and for "follow that",
an open-vocabulary detector (OWLv2) runs on the workstation against the
robot's stream and projects boxes through depth and TF into map coordinates.
Measured live: a bed at (1.86, -0.10) and a chair at (2.47, 2.17), repeatable
to about a centimetre across frames, 35-38 ms per pass. A probe script with
no publisher at all proved the chain before anything was allowed to move.

Two findings from the first live test, either of which would have had the
follower drive at nothing:

- **Prompts match appearance, not identity.** Against a dark blue book held
  flat to the camera: "a blue book", "a book" and "a blue notebook" found
  nothing; "a blue board" scored 0.483 with the box exactly on it. The model
  is right; the obvious prompt is wrong.
- **The threshold was too permissive.** The real book scored 0.483; with the
  book removed, curtains and wall edges scored 0.10-0.12 against the same
  prompt, and the threshold was 0.12. "A lost target stops" cannot save a
  target that never existed. Now 0.25.

Then the follower broke the robot by observing it. Subscribed from the
workstation to the full-rate depth image, it pulled about 20 MB/s across the
wifi link, backed up the robot's own DDS and starved the collision monitor
of its only source, 2554 times in one run against zero in every run without
it. The monitor's documented response to losing a source is to stop the
robot, and it did: 205 non-zero smoothed commands, nothing on `/cmd_vel`. The
follower tracked its target perfectly, published correct goals a metre
short of it, and the wheels never turned. DDS gives a subscriber no way to
ask for less, so the throttle lives at the source: a node on the robot
republishes colour, depth and IR at 2 Hz, about 360 KB/s, which loses nothing
because a room does not change fast. The dashboard's live view uses the same
feed.
