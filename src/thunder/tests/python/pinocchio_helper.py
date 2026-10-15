#!/usr/bin/env python3
"""Pinocchio reference for the thunder dynamics and kinematics tests.

usage: pinocchio_helper.py <urdf> q_1..q_n dq_1..dq_n
       pinocchio_helper.py <urdf> --frames <link>,<link>,... q_1..q_n
prints one line:
  dynamics: M (column-major), C (column-major), Cdq, G
  frames:   for each link, its pose in the world (4x4, column-major) and its Jacobian
            (6 x n, column-major, [linear velocity of the origin; angular velocity] in world axes)
exit 1: pinocchio not available (the test is skipped), other non-zero: error
"""
import os
import sys


def _inject_cmeel_prefix_path() -> None:
    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    base = os.path.expanduser(f"~/.local/lib/python{version}/site-packages")
    cmeel_path = os.path.join(base, "cmeel.prefix", "lib", f"python{version}", "site-packages")
    if os.path.isdir(cmeel_path) and cmeel_path not in sys.path:
        sys.path.insert(0, cmeel_path)


_inject_cmeel_prefix_path()

try:
    import numpy as np
    import pinocchio as pin
except ImportError as exc:
    sys.stderr.write(f"[helper] import failed: {exc}\n")
    sys.exit(1)

if len(sys.argv) < 2:
    sys.stderr.write(f"[helper] expected urdf path and state vectors, got: {sys.argv}\n")
    sys.exit(2)

urdf_path = sys.argv[1]
args = sys.argv[2:]
frames = None
if args[:1] == ["--frames"]:
    if len(args) < 2:
        sys.stderr.write("[helper] --frames expects a comma-separated list of links\n")
        sys.exit(2)
    frames = args[1].split(",")
    args = args[2:]
try:
    raw_values = list(map(float, args))
except ValueError:
    sys.stderr.write("[helper] failed to parse numeric values\n")
    sys.exit(3)

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# no padding: thunder and pinocchio must describe the same model
n_vectors = 1 if frames is not None else 2
if len(raw_values) != n_vectors * model.nq:
    sys.stderr.write(f"[helper] expected {n_vectors} vectors of size {model.nq}, got {len(raw_values)} values\n")
    sys.exit(4)
q = np.array(raw_values[: model.nq])

if frames is not None:
    pin.computeJointJacobians(model, data, q)  # also the forward kinematics
    pin.updateFramePlacements(model, data)
    values = []
    for name in frames:
        if not model.existFrame(name):
            sys.stderr.write(f"[helper] no frame '{name}' in the URDF\n")
            sys.exit(5)
        frame_id = model.getFrameId(name)
        J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        values += [data.oMf[frame_id].homogeneous.ravel(order="F"), J.ravel(order="F")]
    print(" ".join(repr(float(v)) for v in np.concatenate(values)))
    sys.exit(0)

dq = np.array(raw_values[model.nq :])

M = pin.crba(model, data, q)
M = np.triu(M) + np.triu(M, 1).T  # crba fills only the upper triangle
C = pin.computeCoriolisMatrix(model, data, q, dq)
G = pin.computeGeneralizedGravity(model, data, q)
Cdq = pin.nonLinearEffects(model, data, q, dq) - G

values = np.concatenate([M.ravel(order="F"), C.ravel(order="F"), Cdq, G])
print(" ".join(repr(float(v)) for v in values))
