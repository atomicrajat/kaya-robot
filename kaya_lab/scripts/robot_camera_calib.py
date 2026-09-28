"""The robot's colour camera as the robot itself reports it (2026-09-28):
/camera/color/camera_info (K, D) and TF base_link -> camera_color_optical_frame.
Used by the image detector on the robot and by the real-frame tests."""
import numpy as np

K = np.array([[404.85430908, 0.0, 423.50286865], [0.0, 405.0791626, 237.02305603], [0.0, 0.0, 1.0]])
D = np.array([-0.03364242985844612, 0.03777329996228218, 0.0005429951124824584,
              0.00039129576180130243, -0.012874708510935307])
# tf2_echo base_link camera_color_optical_frame
T_XYZ = (0.024, 0.024, 0.138)
Q_XYZW = (-0.473, 0.485, -0.516, 0.525)


def T_base_optical():
    x, y, z, w = Q_XYZW
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, T_XYZ
    return T
