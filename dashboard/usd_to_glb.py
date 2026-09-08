"""Convert the Kaya USDZ to a GLB the dashboard can load with three.js.

There is no USDZ loader in the browser, and no usdcat/blender/assimp on this
machine, so this reads the stage with the USD Python that ships inside Isaac
Sim and writes the glTF binary directly.

Two conversions matter and are easy to get silently wrong:
  * the stage is in CENTIMETRES (metersPerUnit 0.01), so points are scaled to
    metres, which is what every other number in this project is in;
  * the stage is Z-up and three.js is Y-up, so (x, y, z) -> (x, z, -y).
"""
import json, struct, sys
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, Gf

SRC = "/home/rajatmr/work_projects/kaya_robot/kaya_robot_ttb.usdz"
DST = "/home/rajatmr/work_projects/kaya_robot/kaya/dashboard/static/kaya_robot.glb"

stage = Usd.Stage.Open(SRC)
mpu = UsdGeom.GetStageMetersPerUnit(stage)

def shader_input(sh, name, default):
    inp = sh.GetInput(name)
    v = inp.Get() if inp else None
    return default if v is None else v


def material_of(prim):
    """(name, rgb, metallic, roughness) from the bound UsdPreviewSurface.

    The stage carries 20 real materials -- AluminumSatin, RubberSoft and so on
    -- so falling back to one flat grey for everything, as a first pass of this
    script did, throws away the only thing that makes the model readable.
    """
    fallback = ('default', (0.62, 0.66, 0.70), 0.2, 0.6)
    try:
        b = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not b:
            return fallback
        surf = b.GetSurfaceOutput()
        src = surf.GetConnectedSource() if surf else None
        if not src:
            return fallback
        sh = UsdShade.Shader(src[0].GetPrim())
        dc = shader_input(sh, 'diffuseColor', None)
        if dc is None:
            return fallback
        return (b.GetPrim().GetName(),
                tuple(float(x) for x in dc),
                float(shader_input(sh, 'metallic', 0.2)),
                float(shader_input(sh, 'roughness', 0.6)))
    except Exception:
        return fallback


parts = []
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    m = UsdGeom.Mesh(prim)
    pts = m.GetPointsAttr().Get()
    counts = m.GetFaceVertexCountsAttr().Get()
    idx = m.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or not idx:
        continue

    xf = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()), dtype=np.float64)          # row-vector convention
    P = np.array(pts, dtype=np.float64)
    P = (np.hstack([P, np.ones((len(P), 1))]) @ xf)[:, :3] * mpu
    # Z-up (USD) -> Y-up (three.js)
    P = np.column_stack([P[:, 0], P[:, 2], -P[:, 1]]).astype(np.float32)

    idx = np.asarray(idx, dtype=np.int64)
    tri, o = [], 0
    for c in counts:
        for k in range(1, c - 1):
            tri.append((idx[o], idx[o + k], idx[o + k + 1]))
        o += c
    if not tri:
        continue
    T = np.asarray(tri, dtype=np.uint32)

    # Smooth normals: accumulate face normals onto their vertices. Keeps the
    # mesh indexed, which is what keeps this file ~10 MB instead of ~21 MB.
    N = np.zeros_like(P)
    v0, v1, v2 = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    for i in range(3):
        np.add.at(N, T[:, i], fn)
    ln = np.linalg.norm(N, axis=1, keepdims=True)
    N = np.divide(N, np.where(ln == 0, 1, ln)).astype(np.float32)

    parts.append((P, N, T, material_of(prim)))

print("parts:", len(parts), "tris:", sum(len(p[2]) for p in parts))
print("materials:", sorted({p[3][0] for p in parts}))

buf = bytearray()
views, accs, prims, mats = [], [], [], []
mat_of = {}


def view(data, target):
    while len(buf) % 4:
        buf.append(0)
    off = len(buf)
    buf.extend(data)
    views.append({"buffer": 0, "byteOffset": off,
                  "byteLength": len(data), "target": target})
    return len(views) - 1


for P, N, T, mat in parts:
    vp = view(P.tobytes(), 34962)
    vn = view(N.tobytes(), 34962)
    vi = view(T.tobytes(), 34963)
    accs.append({"bufferView": vp, "componentType": 5126, "count": len(P),
                 "type": "VEC3",
                 "min": P.min(axis=0).tolist(), "max": P.max(axis=0).tolist()})
    accs.append({"bufferView": vn, "componentType": 5126, "count": len(N),
                 "type": "VEC3"})
    accs.append({"bufferView": vi, "componentType": 5125, "count": T.size,
                 "type": "SCALAR"})
    name, col, metal, rough = mat
    key = name
    if key not in mat_of:
        mat_of[key] = len(mats)
        mats.append({"name": name, "pbrMetallicRoughness": {
            "baseColorFactor": [col[0], col[1], col[2], 1.0],
            "metallicFactor": metal, "roughnessFactor": rough}})
    prims.append({"attributes": {"POSITION": len(accs) - 3,
                                 "NORMAL": len(accs) - 2},
                  "indices": len(accs) - 1, "material": mat_of[key]})

gltf = {
    "asset": {"version": "2.0", "generator": "kaya usd_to_glb"},
    "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
    "meshes": [{"primitives": prims}], "materials": mats,
    "accessors": accs, "bufferViews": views,
    "buffers": [{"byteLength": len(buf)}],
}
js = json.dumps(gltf, separators=(",", ":")).encode()
js += b" " * ((4 - len(js) % 4) % 4)
while len(buf) % 4:
    buf.append(0)
glb = (struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(buf))
       + struct.pack("<II", len(js), 0x4E4F534A) + js
       + struct.pack("<II", len(buf), 0x004E4942) + bytes(buf))
import os
os.makedirs(os.path.dirname(DST), exist_ok=True)
with open(DST, "wb") as f:
    f.write(glb)

allP = np.vstack([p[0] for p in parts])
print("wrote %s  %.1f MB" % (DST, len(glb) / 1e6))
print("bbox min %s max %s (metres)" % (allP.min(axis=0).round(3), allP.max(axis=0).round(3)))
