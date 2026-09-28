"""Where is the goal box? Read it off the ArUco markers taped to it.

Odometry lost the box in the runs near it (cuVSLAM loses tracking with the
camera up against plain cardboard; wheels drift under hard strafing: three
false goals, runs 12b-14). A marker gives the box's pose relative to the
camera directly, with no drift.

Markers: DICT_4X4_50, printed 146 mm (the printer scaled the 150 mm design;
measured by the user), centred 15 cm up (data/aruco):
    ID 0  inside the back panel     - seen through the mouth
    ID 1  outside of the left panel - left as you face the mouth
    ID 2  outside of the right panel

Box frame: origin at the middle of the mouth on the floor, x INTO the box,
y to the left (facing into the box), z up - the node's "goal frame".
"""

import cv2
import numpy as np

MARKER_SIZE = 0.146
GOAL_DEPTH = 0.30
PANEL_Y = 0.23 + 0.005          # half mouth width + half a cardboard panel
MARKER_Z = 0.15

# Each marker's own axes, written in the box frame. ArUco's marker frame: x to the right
# as seen by someone facing the marker, y up (the page's TOP arrow), z out of the paper
# towards that viewer. The box frame: x into the box, y left, z up.
_AXES = {  # id: (x_marker, y_marker, z_marker) in box coordinates, marker centre
    0: ((0, -1, 0), (0, 0, 1), (-1, 0, 0), (GOAL_DEPTH, 0.0, MARKER_Z)),          # back, faces the mouth
    1: ((-1, 0, 0), (0, 0, 1), (0, 1, 0), (GOAL_DEPTH / 2, PANEL_Y, MARKER_Z)),   # left panel, faces +y
    2: ((1, 0, 0), (0, 0, 1), (0, -1, 0), (GOAL_DEPTH / 2, -PANEL_Y, MARKER_Z)),  # right panel, faces -y
}
BOX_FROM_MARKER = {mid: np.block([[np.column_stack([x, y, z]).astype(float), np.array(c, float)[:, None]],
                                  [np.zeros((1, 3)), np.ones((1, 1))]])
                   for mid, (x, y, z, c) in _AXES.items()}


class GoalMarkers:
    def __init__(self, K, D, T_base_optical, size=MARKER_SIZE):
        self.K, self.D = np.asarray(K, float), np.zeros(5) if D is None else np.asarray(D, float)
        self.T = np.asarray(T_base_optical, float)
        self.size = size
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") \
            else cv2.aruco.DetectorParameters()

    def detect(self, rgb):
        """-> list of (id, box origin xy in base_link, box +x yaw in base_link, distance m,
        marker centre xy in base_link)."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dict, parameters=self.params)
        if ids is None:
            return []
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.size, self.K, self.D)
        out = []
        for mid, rv, tv in zip(ids.flatten(), rvecs, tvecs):
            if int(mid) not in BOX_FROM_MARKER:
                continue
            R_cm, _ = cv2.Rodrigues(rv.reshape(3))
            T_cm = np.eye(4)
            T_cm[:3, :3], T_cm[:3, 3] = R_cm, tv.reshape(3)
            T_bm = self.T @ T_cm                             # marker in base_link
            T_bx = T_bm @ np.linalg.inv(BOX_FROM_MARKER[int(mid)])   # box in base_link
            yaw = float(np.arctan2(T_bx[1, 0], T_bx[0, 0]))
            # the marker centre's position is reliable at any angle; its ORIENTATION is not
            # (planar-pose ambiguity: seen side-on, a square can flip to its mirror pose)
            out.append((int(mid), T_bx[:2, 3].copy(), yaw, float(np.linalg.norm(tv)), T_bm[:2, 3].copy()))
        return out
