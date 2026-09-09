"""Open-vocabulary detection: name a thing in words, get boxes.

RUNS ON THE WORKSTATION. The Jetson has ~600 MB free with the saved map loaded
(measured: 6934 of 7546 MB used), so this cannot live there, and it does not
need to -- the workstation already sees every robot topic over DDS.

WHY OWLv2 AND NOT NanoOWL
NanoOWL is OWL-ViT with a TensorRT engine built for Jetson. On an RTX 5090 the
TensorRT packaging buys nothing and costs an engine build, so the same model
family is used directly through transformers. If detection ever has to move
on-board, NanoOWL is the port of this file, not a different design: the
interface below -- text prompts in, boxes out -- is what the follower depends
on, and it is the same either way.
"""

import time

import numpy as np


class OwlDetector:
    """Detect whatever is named in words. Boxes are (x0, y0, x1, y1) pixels."""

    def __init__(self, model='google/owlv2-base-patch16-ensemble', device=None,
                 threshold=0.12):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor

        self.torch = torch
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.threshold = threshold
        self.processor = Owlv2Processor.from_pretrained(model)
        self.model = Owlv2ForObjectDetection.from_pretrained(model)
        # fp16 halves the weights and roughly doubles throughput; this is a
        # detector feeding a 2 Hz goal, not a measurement instrument.
        if self.device == 'cuda':
            self.model = self.model.half()
        self.model = self.model.to(self.device).eval()
        self.last_ms = None

    def detect(self, rgb, prompts):
        """rgb: HxWx3 uint8. prompts: list of phrases. -> list of dicts."""
        import torch
        t0 = time.time()
        inputs = self.processor(text=[prompts], images=rgb, return_tensors='pt')
        inputs = {k: (v.half() if v.dtype == torch.float32 and self.device == 'cuda'
                      else v).to(self.device) for k, v in inputs.items()}
        with torch.inference_mode():
            out = self.model(**inputs)
        # OWLv2 letterboxes internally; post_process wants the ORIGINAL size or
        # every box lands in the wrong place, which looks like a tracking bug
        # rather than a preprocessing one.
        res = self.processor.post_process_grounded_object_detection(
            outputs=out,
            target_sizes=torch.tensor([[rgb.shape[0], rgb.shape[1]]]).to(self.device),
            threshold=self.threshold)[0]
        self.last_ms = (time.time() - t0) * 1000.0

        dets = []
        for box, score, label in zip(res['boxes'], res['scores'], res['labels']):
            b = [float(v) for v in box.tolist()]
            dets.append({'box': b, 'score': float(score),
                         'label': prompts[int(label)]})
        dets.sort(key=lambda d: -d['score'])
        return dets


def box_depth(depth_m, box, frac=0.5):
    """Median of the VALID depth inside the middle of a box, or None.

    Only ~37% of depth pixels carry a return in this room -- the floor is
    glossy and IR stereo gets nothing off it -- so a centre-pixel lookup
    returns zero often enough to be useless. The median over valid pixels in
    the middle of the box is stable, and None is returned rather than a guess
    when the box has no depth at all: a follower that invents a distance drives
    at something that is not there.
    """
    h, w = depth_m.shape
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hw, hh = (x1 - x0) * frac / 2.0, (y1 - y0) * frac / 2.0
    a = max(0, int(cx - hw)), max(0, int(cy - hh))
    b = min(w, int(cx + hw) + 1), min(h, int(cy + hh) + 1)
    if b[0] <= a[0] or b[1] <= a[1]:
        return None
    patch = depth_m[a[1]:b[1], a[0]:b[0]]
    valid = patch[(patch > 0.05) & (patch < 10.0)]
    if valid.size < 20:
        return None
    return float(np.median(valid))
