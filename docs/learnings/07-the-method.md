# Chapter 7: the method

*Part of the Kaya learning narrative. Index: [`../../LEARNINGS.md`](../../LEARNINGS.md).
The chapters before this one are the evidence; this one is what they have in common.*

Both projects were built the same way: make one change, measure the thing
the change was supposed to improve, write down the number, and keep the
number next to the setting so nobody re-tries it blind. Sixteen habits came
out of that, each learned by getting it wrong at least once.

**1. Verify against the thing, not the counter.** The `time_domain` change
was validated with `vo_state` and frame times, which measure performance,
not accuracy; it took drift from 1.2% to 32% while every health counter read
perfect. The same instrumentation hid the regression for a day. A health
report is not a measurement of the outcome.

**2. Measure the distribution, not the mean.** Frame sync on and off both
averaged 30 Hz. Only the gaps between frames (5.7 ms median with 190 ms
bursts, against a steady 33.6 ms) showed the problem, and only the age of the
pose estimate showed its cost.

**3. Compare both eyes.** The left IR ran 15% slower than the right while the
pairing tool reported 0% unmatched, correctly, because every surviving left
frame had a partner. A number can be right and still not be the one that
matters.

**4. A stationary test cannot see a motion-proportional error.** Corrupt
IMU timestamps held 1.4 mm over 94 s at rest and ruined every driven loop.
If a failure only happens while moving, only a driven loop can find it.

**5. Diff the configuration against the last commit that worked.** Four
environmental hypotheses were measured and disproved before one `git diff`
found the single changed line. Do the diff first.

**6. A result that sits suspiciously close to a threshold is the
measurement's fault.** Every ball stopping at +7.3 cm against a +7.65 cm
success line was the checker clearing its own flag, not the physics.

**7. Sanity-check a new tool on the case where you know the answer.** The
sphere fit's sign error showed up on the easy whole-ball frame. A tool that
has never been seen to pass an easy case cannot be trusted on a hard one.

**8. Two independent measurements agreeing is how trust is earned.** The
real camera frames and the CAD-built simulator gave the same 12 cm pocket
between the front wheels; the simulated IMU at rest matched the real one to
the second decimal, and when it did not, the real robot's description was
wrong.

**9. Read the errors even when it works.** "Patch buffer overflow" printed
once per training run for days before anyone asked what it meant. It was
PhysX dropping contacts.

**10. Guards against one failure cause the next one.** Three detection
filters, each right for the run that motivated it, each later rejected a
real ball or a real goal. Write down *why* each guard exists, naming the run,
so the next failure can be traced back to it instead of being fixed by
another guard.

**11. Film the robot.** The log said 1.5 m of dribbling; the video showed 25
seconds pinned against a panel. A log records what the robot believed. When
the belief is the bug, only the video can say so.

**12. Measure the physical thing before debugging the model of it.** The
ball was 14 cm, not 15.3; three theories about lens distortion, glare and
camera height died with a tape measure.

**13. Do not stack guesses.** When a fix made the servo-model mismatch
worse, the right move was to look at the data (the original's floating-point
tick), not to add a second fix on top of the first.

**14. Retract in writing.** The lighting explanation for bad odometry was
recorded, disproved and retracted in the same commit history, and the
retraction says why the first version was wrong. A repo that only records
what turned out to be true teaches the next reader to trust claims that have
not yet been tested.

**15. A display that reports health over a dead source is the recurring
bug.** The anchor node announced success on failed localisations; cuVSLAM
said "Successfully localized" 2.4 m from truth on an exhausted board; the
dashboard showed a confident pose for a robot whose battery had died. Every
report now carries its age, says `unknown` when it does not know, and a goal
to nobody is an error.

**16. Hygiene is not optional on a small board.** Orphaned processes faked
two estimator bugs; a self-matching `pkill` killed three ssh sessions; a
battery death stranded verified, uncommitted work once. Launch into a process
group, match on paths, and commit as you verify.

---

What this adds up to is simple to state and hard to keep doing: the robot
only ever tells you what it believes, so every number you read is a claim
until something independent of the thing that produced it agrees. The
measurement tools in `kaya_bringup/tools/`, the thirteen simulator checks,
the real-frame tests for the detector, the contact sheets beside every run
log, and the twenty-two rows of the settings table are all the same idea
applied in different places.
