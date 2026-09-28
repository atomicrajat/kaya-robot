"""Marker probe: one frame -> the goal box pose from each visible marker. Never moves."""
import time, cv2, numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from ball_policy_node import tf_matrix
from goal_marker import GoalMarkers
rclpy.init(); n = Node("probe_marker"); got = {}
q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(Image, "/camera/color/image_raw", lambda m: got.__setitem__("img", m), q)
n.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: got.setdefault("info", m), 10)
tf = tf2_ros.Buffer(); tf2_ros.TransformListener(tf, n)
t0 = time.time()
while time.time() - t0 < 10 and not ("img" in got and "info" in got and tf.can_transform("base_link", "camera_color_optical_frame", Time())):
    rclpy.spin_once(n, timeout_sec=0.2)
gm = GoalMarkers(np.array(got["info"].k).reshape(3, 3), np.array(got["info"].d),
                 tf_matrix(tf.lookup_transform("base_link", "camera_color_optical_frame", Time())))
for k in range(3):
    got.pop("img", None)
    while "img" not in got: rclpy.spin_once(n, timeout_sec=0.2)
    m = got["img"]; img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[..., :3].copy()
    res = gm.detect(img)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    corners, ids, rej = cv2.aruco.detectMarkers(gray, gm.dict, parameters=gm.params)
    raw = [(int(i), round(float(np.linalg.norm(c[0][0] - c[0][1])), 0)) for i, c in zip(ids.flatten(), corners)] if ids is not None else []
    print(f"frame {k}: raw ids+side_px {raw}, rejected candidates {len(rej)} | " + ("; ".join(f"ID {i}: mouth centre at ({p[0]:+.3f}, {p[1]:+.3f}) m from robot centre, box facing {np.degrees(y):+.1f} deg, marker {d:.2f} m away" for i, p, y, d in res) or "no marker"))
import sys
tag = sys.argv[1] if len(sys.argv) > 1 else "x"
cv2.imwrite(f"probe_marker_{tag}.png", img[..., ::-1])
