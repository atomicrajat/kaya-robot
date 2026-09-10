"""Phase 10: ask the robot where something is, in words.

    python3 perception/scene_memory.py caption          # once, after a drive
    python3 perception/scene_memory.py ask "where can I eat"

Turns the Phase 9 keyframe store into something answerable. A question comes
in as English, a place comes out as (x, y, yaw) in the map frame -- and Nav2
already drives to a pose like that within 6 cm.

CAPTION ONCE, QUERY MANY
The obvious implementation sends every keyframe image to a vision model with
the question attached. It is also the wrong one. 62 keyframes at roughly two
seconds of vision inference each is over a minute per question, every question,
and it gets worse with every drive: the store grows, the questions do not get
faster. So this runs the vision model ONCE per keyframe, at capture time,
and writes down what it saw. A question after that is text against text --
about a second, and flat in the size of the store.

It also makes the memory inspectable. `captions.json` is readable English, so
when the robot answers "the kitchen is at (2.1, -0.4)" you can see exactly what
it thought it saw there, rather than re-running a model to find out.

WHY IT IS ALLOWED TO SAY NO
The model is asked to pick a keyframe or return nothing, and "nothing" is a
first-class answer. If no keyframe shows a kitchen, the honest output is that
there is no kitchen in the store -- not the closest-looking frame. This matters
more here than in a chat: a wrong answer is not a wrong sentence, it is a robot
driving somewhere. The store from a first drive covers 5.6 x 7.3 m of one flat;
plenty of reasonable questions have no answer in it.

TWO MODELS, ONE DOWNLOAD
Captioning needs vision. Answering needs only language, and the vision model
answers language fine, so both use qwen2.5vl and there is one thing to install.

THE POSES ARE ONLY AS GOOD AS THE FRAME THEY WERE RECORDED IN
This store was recorded with no map_path, because relocalisation failed three
times from the mark -- furniture has moved since the map was built. cuVSLAM
therefore built a fresh frame anchored wherever the robot stood at launch, and
the store is reusable only because that was the MARK: the first keyframe reads
(-0.008, 0.002) yaw 0.0, which is the origin anchor firing on the marked spot.

So before driving to any pose from this store: put the robot on the mark,
facing the same way, and launch with no map_path. Launch from anywhere else and
every stored pose is offset by that distance -- and the failure is silent, the
robot driving confidently to the wrong room with a photo of the right one on
the screen. See keyframes/FRAME.md.

WHERE THIS RUNS, and why that is not a contradiction
On the workstation, not the robot -- against the roadmap's instinct to put
models on-board. Because this query is ONE-SHOT: the human waits a second for
an answer to a question they just typed, and a second is not felt. Tracking and
following are the opposite -- a control loop where latency is error -- and
those belong on the robot (Phase 11, NanoOWL). The split is by latency
sensitivity, not by preference.
"""

import argparse
import base64
import json
import math
import os
import urllib.error
import urllib.request

OLLAMA = os.environ.get('OLLAMA_HOST', 'http://127.0.0.1:11434')
MODEL = os.environ.get('KAYA_VLM', 'qwen2.5vl:7b')
STORE = os.path.expanduser('~/work_projects/kaya_robot/keyframes')

# A KEYFRAME POSE IS WHERE THE CAMERA WAS, NOT WHERE THE THING IS.
# The first version of this prompt asked for prose, and it conflated the two.
# One frame taken at the mouth of a corridor described "a stainless steel
# refrigerator against the wall, indicating a kitchen area" -- from 2.7 m away,
# down the corridor -- while the frame actually standing in the kitchen, with
# the sink and the cabinets in reach, hedged with "possibly a kitchen or a
# hallway leading to a kitchen". Asked where to cook, a text match picks the
# confident sentence, and the robot parks at the doorway looking in.
#
# So the caption is now structured, and the split that matters is HERE versus
# BEYOND: what the robot is standing among, against what it can merely see from
# there. A fridge in `beyond` is a signpost pointing at the kitchen. A sink in
# `here` IS the kitchen.
CAPTION_PROMPT = """You are the memory of a small indoor robot, about 30 cm
tall. This photo is from its camera, looking forward, as it drove through a
home.

Reply with strict JSON only:

{
  "room": "<kitchen | dining area | living room | bedroom | bathroom |
            hallway | doorway | utility | workspace | unclear>",
  "standing_in_it": <true if the robot is INSIDE that room, false if it is
                     looking into it from a hallway, a doorway, or down a
                     corridor>,
  "here": ["<objects within about two metres, in the space the robot is
             actually in>"],
  "beyond": ["<objects clearly further away -- through a doorway, down a
               corridor, in the next room>"],
  "summary": "<one sentence describing this view>"
}

The `here` / `beyond` split is the important part, so take care with it. A
fridge glimpsed at the far end of a corridor belongs in `beyond`, and the room
is then the corridor, not the kitchen. Only call the room a kitchen when the
robot is among the kitchen things.

Judge distance by how large objects appear and by whether a doorway or wall
edge sits between the camera and them. Describe only what is visible. A view of
a blank wall or a stretch of floor is a useful thing to record as exactly that:
"unclear" is a correct answer, and an honest one."""

# There is deliberately NO rule below spelling out what eating or cooking needs.
# One was tried -- "somewhere to eat is a dining table, somewhere to cook is a
# sink, stove or counter" -- and it made the answers WORSE: it taught the model
# that a sink means kitchen, and "take me to the washbasin" then chose a bedroom
# over the bathroom. Removing it, eat, cook and washbasin all resolve correctly.
# The model already knows what rooms are for. The only thing it was missing was
# whether the robot was STANDING in one or looking into it from a corridor, and
# that is what `here` and `beyond` supply.
#
# Hand-written domain rules are also exactly what would not survive a building
# this store has never seen, which is the point of the whole exercise.
#
# Keep explanations like this one OUT of the prompt string. This note was first
# written inside it, and since it names a wrong answer in passing, the model
# read it and reproduced that wrong answer.
ANSWER_PROMPT = """A small indoor robot has driven through a home and
remembers these places. For each, `here` is what the robot was standing among
and `beyond` is what it could only see from there.

{places}

The person asked: "{question}"

Pick the ONE place that best answers them, and reply with strict JSON only:

  {{"place": <number>, "confidence": <0.0 to 1.0>, "why": "<one short sentence>"}}

How to choose, in order:

1. Prefer a place whose `here` holds what the question is about, and where
   `standing_in_it` is true. That is the robot actually being in the room.
2. A thing in `beyond` is a SIGNPOST toward a place, not the place itself.
   Never answer with a place that only sees the target from a distance while
   another place in the list stands among it -- sending the robot to the
   doorway when it could be in the room is a wrong answer, not a cautious one.
3. Places hedge. A frame that says "possibly a kitchen" while listing a sink
   and cabinets in `here` is a better kitchen than one that says "indicating a
   kitchen area" about a fridge in `beyond`. Judge by the objects, not by how
   confident the wording sounds.
If NO place in the list answers the question, reply exactly:

  {{"place": null, "confidence": 0.0, "why": "<what is missing>"}}

Answering with a place that does not actually match sends a robot driving to
the wrong room, so null is the right answer whenever nothing fits. Do not
invent a place that is not in the list."""


def _post(path, payload, timeout=300):
    req = urllib.request.Request(
        OLLAMA + path, data=json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def available():
    """Is the model actually loaded and ready? Checked before promising anything."""
    try:
        tags = json.loads(urllib.request.urlopen(OLLAMA + '/api/tags',
                                                 timeout=5).read())
    except (urllib.error.URLError, OSError) as e:
        return False, 'ollama is not reachable at %s (%s)' % (OLLAMA, e)
    names = [m['name'] for m in tags.get('models', [])]
    if not any(n == MODEL or n.startswith(MODEL.split(':')[0]) for n in names):
        return False, ('%s is not installed. Run:  ollama pull %s'
                       % (MODEL, MODEL))
    return True, MODEL


def load_places(store=STORE, shortlist=True):
    """The keyframes to reason over: the thinned shortlist if one exists.

    The raw store oversamples badly -- a first drive put 428 frames into 62
    distinct places -- and near-duplicates spend a model's attention on the
    same wall eight times.
    """
    path = os.path.join(store, 'shortlist.json' if shortlist else 'index.json')
    if not os.path.isfile(path):
        path = os.path.join(store, 'index.json')
    with open(path) as f:
        return json.load(f), path


def caption_one(store, kf, timeout=300):
    img = os.path.join(store, 'images', kf['file'])
    with open(img, 'rb') as f:
        b64 = base64.b64encode(f.read()).decode()
    out = _post('/api/generate', {
        'model': MODEL, 'prompt': CAPTION_PROMPT, 'images': [b64],
        'stream': False, 'format': 'json', 'options': {'temperature': 0.1},
    }, timeout=timeout)
    raw = out['response'].strip()
    try:
        c = json.loads(raw)
    except ValueError:
        # Keep the prose rather than losing the frame; describe() copes.
        return {'room': 'unclear', 'standing_in_it': False, 'here': [],
                'beyond': [], 'summary': raw[:400]}
    c.setdefault('room', 'unclear')
    c.setdefault('standing_in_it', False)
    for k in ('here', 'beyond'):
        v = c.get(k) or []
        c[k] = [str(x) for x in v] if isinstance(v, list) else [str(v)]
    c.setdefault('summary', '')
    return c


def describe(p):
    """One line per place, as the answering model sees it.

    `here` and `beyond` are kept visibly separate, because collapsing them is
    the exact mistake that parked the robot at a doorway looking at a fridge.
    """
    c = p.get('caption')
    if isinstance(c, str):                       # a store from the prose era
        return c.replace('\n', ' ')
    c = c or {}
    where = 'in it' if c.get('standing_in_it') else 'looking in from outside'
    bits = ['%s (%s)' % (c.get('room', 'unclear'), where)]
    if c.get('here'):
        bits.append('here: ' + ', '.join(c['here']))
    if c.get('beyond'):
        bits.append('beyond (only visible from here): ' + ', '.join(c['beyond']))
    if c.get('summary'):
        bits.append(c['summary'].replace('\n', ' '))
    return ' | '.join(bits)


def caption_all(store=STORE, force=False, limit=None, log=print):
    """Caption every keyframe that has no caption yet, saving as it goes."""
    places, src = load_places(store)
    out_path = os.path.join(store, 'captions.json')
    have = {}
    if os.path.isfile(out_path) and not force:
        with open(out_path) as f:
            have = {c['file']: c for c in json.load(f)}
        log('resuming: %d of %d already captioned' % (len(have), len(places)))

    todo = [k for k in places if k['file'] not in have]
    if limit:
        todo = todo[:limit]
    log('captioning %d keyframes from %s with %s'
        % (len(todo), os.path.basename(src), MODEL))

    for i, kf in enumerate(todo, 1):
        try:
            text = caption_one(store, kf)
        except Exception as e:                            # noqa: BLE001
            log('  [%d/%d] %s FAILED: %s' % (i, len(todo), kf['file'], e))
            continue
        rec = dict(kf)
        rec['caption'] = text
        have[kf['file']] = rec
        # Written every frame, not at the end: captioning 62 images takes
        # minutes, and losing all of it to one bad frame would be silly.
        ordered = [have[k['file']] for k in places if k['file'] in have]
        with open(out_path, 'w') as f:
            json.dump(ordered, f, indent=1)
        log('  [%d/%d] (%+.2f, %+.2f) %s'
            % (i, len(todo), kf['x'], kf['y'], describe(rec)[:100]))
    log('captions.json now holds %d places' % len(have))
    return out_path


def load_captions(store=STORE):
    path = os.path.join(store, 'captions.json')
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f)


def ask(question, store=STORE, timeout=120):
    """Answer a question with a place, or with an honest nothing.

    Returns a dict the dashboard can render directly: the chosen keyframe, its
    pose, why it was chosen, and -- when nothing fits -- what was missing.
    """
    ok, why = available()
    if not ok:
        return {'ok': False, 'error': why}
    places = load_captions(store)
    if not places:
        return {'ok': False, 'error':
                'nothing has been captioned yet. Drive the robot with '
                'keyframe_store.py running, then: '
                'python3 perception/scene_memory.py caption'}

    listing = '\n'.join('%d. %s' % (i, describe(p))
                        for i, p in enumerate(places, 1))
    out = _post('/api/generate', {
        'model': MODEL,
        'prompt': ANSWER_PROMPT.format(places=listing, question=question),
        'stream': False, 'format': 'json',
        'options': {'temperature': 0.0},
    }, timeout=timeout)

    try:
        pick = json.loads(out['response'])
    except ValueError:
        return {'ok': False, 'error': 'the model did not return JSON',
                'raw': out.get('response', '')[:300]}

    n = pick.get('place')
    if n is None:
        return {'ok': True, 'found': False, 'question': question,
                'why': pick.get('why', 'no place in the store matches')}
    try:
        p = places[int(n) - 1]
    except (ValueError, TypeError, IndexError):
        return {'ok': False, 'error': 'the model picked place %r, which is not '
                                      'in the list of %d' % (n, len(places))}
    return {
        'ok': True, 'found': True, 'question': question,
        'x': p['x'], 'y': p['y'], 'yaw_deg': p['yaw_deg'],
        'yaw': math.radians(p['yaw_deg']),
        'file': p['file'], 'caption': describe(p),
        'confidence': pick.get('confidence'), 'why': pick.get('why', ''),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('command', choices=['caption', 'ask', 'check'])
    ap.add_argument('question', nargs='*')
    ap.add_argument('--store', default=STORE)
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--limit', type=int)
    args = ap.parse_args()

    if args.command == 'check':
        ok, why = available()
        print('model: %s' % why)
        places, src = load_places(args.store)
        print('places: %d from %s' % (len(places), src))
        caps = load_captions(args.store)
        print('captions: %s' % (len(caps) if caps else 'none yet'))
        return

    if args.command == 'caption':
        ok, why = available()
        if not ok:
            raise SystemExit(why)
        caption_all(args.store, force=args.force, limit=args.limit)
        return

    r = ask(' '.join(args.question), args.store)
    if not r.get('ok'):
        raise SystemExit(r['error'])
    if not r['found']:
        print('no place in the store answers that.  %s' % r['why'])
        return
    print('%s\n  -> (%+.2f, %+.2f) yaw %+.0f deg   confidence %s'
          % (r['why'], r['x'], r['y'], r['yaw_deg'], r['confidence']))
    print('  %s' % r['file'])
    print('  %s' % r['caption'])


if __name__ == '__main__':
    main()
