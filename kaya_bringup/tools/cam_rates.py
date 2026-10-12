"""Measure every camera stream cuVSLAM and the costmaps depend on."""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
TOPICS = [
    ("/camera/left_ir/image_raw", Image),
    ("/camera/right_ir/image_raw", Image),
    ("/camera/depth/image_raw", Image),
    ("/camera/depth/points", PointCloud2),
    ("/camera/color/image_raw", Image),
]
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
               reliability=ReliabilityPolicy.BEST_EFFORT)
rclpy.init()
n = Node("cam_rates")
stamps = {t: [] for t, _ in TOPICS}
for t, ty in TOPICS:
    n.create_subscription(ty, t, (lambda k: (lambda m: stamps[k].append(time.time())))(t), q)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
print("over %.0f s:" % DUR)
for t, _ in TOPICS:
    s = stamps[t]
    if len(s) < 3:
        print("  %-32s %5d msgs   --" % (t, len(s)))
        continue
    d = [(s[i+1]-s[i])*1000 for i in range(len(s)-1)]
    d.sort()
    hz = len(s) / (s[-1]-s[0]) if s[-1] > s[0] else 0
    print("  %-32s %5.1f Hz   gap median %5.1f ms  p90 %6.1f ms  max %7.1f ms"
          % (t, hz, d[len(d)//2], d[int(len(d)*0.9)], d[-1]))
