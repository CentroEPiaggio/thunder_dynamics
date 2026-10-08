#!/usr/bin/env python3
"""Pinocchio reference for the thunder dynamics test.

usage: pinocchio_helper.py <urdf> q_1..q_n dq_1..dq_n
prints one line: M (column-major), C (column-major), Cdq, G
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
try:
    raw_values = list(map(float, sys.argv[2:]))
except ValueError:
    sys.stderr.write("[helper] failed to parse numeric values\n")
    sys.exit(3)

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# no padding: thunder and pinocchio must describe the same model
if len(raw_values) != 2 * model.nq:
    sys.stderr.write(f"[helper] expected q and dq of size {model.nq}, got {len(raw_values)} values\n")
    sys.exit(4)
q = np.array(raw_values[: model.nq])
dq = np.array(raw_values[model.nq :])

M = pin.crba(model, data, q)
M = np.triu(M) + np.triu(M, 1).T  # crba fills only the upper triangle
C = pin.computeCoriolisMatrix(model, data, q, dq)
G = pin.computeGeneralizedGravity(model, data, q)
Cdq = pin.nonLinearEffects(model, data, q, dq) - G

values = np.concatenate([M.ravel(order="F"), C.ravel(order="F"), Cdq, G])
print(" ".join(repr(float(v)) for v in values))
