#!/usr/bin/env python3
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

if len(raw_values) % 3 != 0:
    sys.stderr.write("[helper] values must be q + dq + ddq with equal lengths\n")
    sys.exit(4)

n = len(raw_values) // 3
q = np.array(raw_values[:n])
dq = np.array(raw_values[n : 2 * n])
ddq = np.array(raw_values[2 * n : 3 * n])

model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

if q.size != model.nq:
    padded_q = np.zeros(model.nq)
    padded_dq = np.zeros(model.nq)
    padded_ddq = np.zeros(model.nq)
    padded_q[: min(model.nq, q.size)] = q[: min(model.nq, q.size)]
    padded_dq[: min(model.nq, dq.size)] = dq[: min(model.nq, dq.size)]
    padded_ddq[: min(model.nq, ddq.size)] = ddq[: min(model.nq, ddq.size)]
    q, dq, ddq = padded_q, padded_dq, padded_ddq

try:
    tau = pin.rnea(model, data, q, dq, ddq)
except Exception as exc:
    sys.stderr.write(f"[helper] rnea failed: {exc}\n")
    sys.exit(5)

print(" ".join(str(value) for value in tau.tolist()))
