"""YOLO11 on the Jetson's GPU through TensorRT, with no PyTorch, ONNX runtime or
pycuda: TensorRT's own Python bindings run the engine, and the four CUDA calls
it needs (malloc, copy in, copy out, free) go straight to libcudart via ctypes.

Used as step 1 of the ball detector: WHERE in the image is the ball? (COCO
class 32, "sports ball"; the pretrained model found the real striped ball at
74-79% and ignored the cardboard that fooled the colour threshold.)

Build the engine once, on the robot:
    /usr/src/tensorrt/bin/trtexec --onnx=yolo11s.onnx --saveEngine=yolo11s_fp16.engine --fp16
"""

import ctypes

import cv2
import numpy as np
import tensorrt as trt

SPORTS_BALL = 32
_H2D, _D2H = 1, 2


class _Cuda:
    def __init__(self):
        self.lib = ctypes.CDLL("libcudart.so")
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]

    def check(self, err, what):
        if err != 0:
            raise RuntimeError(f"CUDA {what} failed with error {err}")

    def malloc(self, n):
        p = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(p), n), "malloc")
        return p

    def to_device(self, dst, arr):
        self.check(self.lib.cudaMemcpy(dst, arr.ctypes.data, arr.nbytes, _H2D), "memcpy H2D")

    def to_host(self, arr, src):
        self.check(self.lib.cudaMemcpy(arr.ctypes.data, src, arr.nbytes, _D2H), "memcpy D2H")

    def sync(self):
        self.check(self.lib.cudaDeviceSynchronize(), "sync")


class YoloTRT:
    def __init__(self, engine_path: str, cls: int = SPORTS_BALL):
        self.cls = cls
        self.cuda = _Cuda()
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        self.bufs = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            host = np.zeros(shape, dtype=dtype)
            dev = self.cuda.malloc(host.nbytes)
            self.ctx.set_tensor_address(name, dev.value)
            is_in = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            self.bufs[name] = (host, dev, is_in)
        self.in_name = next(n for n, b in self.bufs.items() if b[2])
        self.out_name = next(n for n, b in self.bufs.items() if not b[2])
        self.size = self.bufs[self.in_name][0].shape[-1]            # 640

    def detect(self, rgb: np.ndarray, conf: float = 0.25):
        """RGB image -> list of (confidence, x1, y1, x2, y2) in image pixels, best first."""
        h, w = rgb.shape[:2]
        s = self.size / max(h, w)
        nh, nw = round(h * s), round(w * s)
        px, py = (self.size - nw) // 2, (self.size - nh) // 2
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)        # letterbox, as in training
        canvas[py:py + nh, px:px + nw] = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        host_in, dev_in, _ = self.bufs[self.in_name]
        self.cuda.to_device(dev_in, x.astype(host_in.dtype))
        self.ctx.execute_async_v3(0)
        self.cuda.sync()
        host_out, dev_out, _ = self.bufs[self.out_name]
        self.cuda.to_host(host_out, dev_out)
        out = host_out[0].astype(np.float32)                        # (84, 8400): cx, cy, w, h, 80 class scores
        score = out[4 + self.cls]
        keep = np.nonzero(score > conf)[0]
        if len(keep) == 0:
            return []
        cx, cy, bw, bh = out[0, keep], out[1, keep], out[2, keep], out[3, keep]
        boxes = np.stack([(cx - bw / 2 - px) / s, (cy - bh / 2 - py) / s,
                          (cx + bw / 2 - px) / s, (cy + bh / 2 - py) / s], 1)
        idx = cv2.dnn.NMSBoxes([[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in boxes],
                               score[keep].tolist(), conf, 0.5)
        idx = np.array(idx).flatten()
        dets = [(float(score[keep][i]), *[float(v) for v in boxes[i]]) for i in idx]
        return sorted(dets, reverse=True)
