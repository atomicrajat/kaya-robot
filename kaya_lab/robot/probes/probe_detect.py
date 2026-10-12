"""Detection-only probe: one colour frame -> every YOLO sports-ball candidate down to
5%, the colour blobs and their sphere fits. Publishes NOTHING (the robot never moves)."""
import os
import sys

# One-off probes live one level below the node. Make the node's modules (ball_policy_node,
# depth_scan, goal_marker, yolo_trt) and kaya_lab/ball_detector importable without copying.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_HERE, os.pardir),
                os.path.join(_HERE, os.pardir, os.pardir, "kaya_lab")]
import cv2, numpy as np, rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from rclpy.time import Time
from yolo_trt import YoloTRT
from ball_detector import BallDetector
from ball_policy_node import tf_matrix
rclpy.init(); n = Node("probe_detect")
got = {}
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, "/camera/color/image_raw", lambda m: got.setdefault("img", m), q)
n.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: got.setdefault("info", m), 10)
tf = tf2_ros.Buffer(); tf2_ros.TransformListener(tf, n)
t0 = time.time()
while time.time() - t0 < 10 and not ("img" in got and "info" in got and tf.can_transform("base_link", "camera_color_optical_frame", Time())):
    rclpy.spin_once(n, timeout_sec=0.2)
m = got["img"]; img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., :3].copy()
cv2.imwrite("probe_corner.png", img[..., ::-1])
K = np.array(got["info"].k).reshape(3, 3); D = np.array(got["info"].d)
T = tf_matrix(tf.lookup_transform("base_link", "camera_color_optical_frame", Time()))
y = YoloTRT("yolo11s_fp16.engine")
print("YOLO sports-ball candidates >= 5%:", [(round(c, 2), [int(v) for v in b]) for c, *b in y.detect(img, conf=0.05)] or "NONE")
det = BallDetector(K, D, T, 0.07, 0.07 - 0.037)            # colour only, all blobs
mask = det.mask(img); nb, lab, st, _ = cv2.connectedComponentsWithStats(mask)
order = 1 + np.argsort(-st[1:, cv2.CC_STAT_AREA])[:5]
for i in order:
    a = int(st[i, cv2.CC_STAT_AREA])
    if a < 50: continue
    d = det._fit_blob((lab == i).astype(np.uint8), a, img.shape[0], img.shape[1])
    x, yy, w, h = st[i, :4]
    print(f"colour blob area {a:6d} px at x {x}-{x+w} y {yy}-{yy+h}: fit ok={d.ok} base ({d.x:.2f},{d.y:+.2f}) rms {d.rms_deg:.2f} deg")
hyb = BallDetector(K, D, T, 0.07, 0.07 - 0.037, yolo=y).detect(img)
print(f"hybrid detector result: ok={hyb.ok} source={hyb.source} base ({hyb.x:.2f},{hyb.y:+.2f}) rms {hyb.rms_deg:.2f}")
