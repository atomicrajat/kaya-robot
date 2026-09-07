"""Brightness and texture of the left IR image, standalone."""
import numpy as np, rclpy, sys, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class S(Node):
    def __init__(self, label, dur):
        super().__init__('ir_stats')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.cb, qos)
        self.b=[]; self.t=[]; self.label=label; self.t0=time.time(); self.dur=dur
        self.create_timer(0.5, self.tick)
    def cb(self, m):
        a=np.frombuffer(m.data,dtype=np.uint8)
        if a.size < m.height*m.width: return
        s=a[:m.height*m.width].reshape(m.height,m.width)[::4,::4].astype(np.int16)
        self.b.append(float(s.mean()))
        gx=np.abs(np.diff(s,axis=1)).mean(); gy=np.abs(np.diff(s,axis=0)).mean()
        self.t.append(float((gx+gy)/2))
    def tick(self):
        if time.time()-self.t0 < self.dur: return
        if not self.b:
            print('%s: NO IMAGES' % self.label); rclpy.shutdown(); return
        b=sorted(self.b); t=sorted(self.t)
        print('%-32s brightness %5.1f   texture %5.2f   (n=%d)'
              % (self.label, b[len(b)//2], t[len(t)//2], len(b)))
        rclpy.shutdown()

rclpy.init()
try: rclpy.spin(S(sys.argv[1], float(sys.argv[2]) if len(sys.argv)>2 else 12.0))
except Exception: pass
