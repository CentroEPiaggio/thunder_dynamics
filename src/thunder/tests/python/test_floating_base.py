"""Floating base (floating_loader) against Pinocchio's free-flyer.

thunder: generalized velocity u = [twist_base; dq], twist_base = [v; w] of the base origin in world axes
(floating_loader twist: world) or in base axes (twist: body), pose_base = [p; quat xyzw], outputs stacked [base; joints].
Pinocchio: q = [p; quat xyzw; q_j], v = [v_local; w_local; dq] (body twist).
With v_pin = T u (P: thunder joint order -> Pinocchio joint order)
    world: T = blkdiag(R^T, R^T, P), dT = blkdiag(-[w_b]x R^T, -[w_b]x R^T, 0);   body: T = blkdiag(I, I, P), dT = 0
    M = T^T M_pin T,  G = T^T G_pin,  tau = T^T rnea_pin(q, T u, T du + dT u),  C = T^T (C_pin T + M_pin dT),
    J = J_pin T,  J_dot = J_pin_dot T + J_pin dT.
Every check runs with both conventions.
Fixtures: tests/fixtures/floating/biped_tree.urdf (tree: two branches on the base) and franka_finger (chain).
Skipped if Pinocchio is not installed (pip install --user pin).
"""
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import casadi
import numpy as np

from thunder_core.pipeline import Config


def _import_pinocchio():
    # pip --user installs pin under a cmeel prefix (same trick as pinocchio_helper.py)
    version = f"{sys.version_info.major}.{sys.version_info.minor}"
    cmeel = os.path.expanduser(f"~/.local/lib/python{version}/site-packages/cmeel.prefix/lib/python{version}/site-packages")
    if os.path.isdir(cmeel) and cmeel not in sys.path:
        sys.path.insert(0, cmeel)
    try:
        import pinocchio
        return pinocchio
    except ImportError:
        return None


pin = _import_pinocchio()
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"
TOL = 1e-8


def hat(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def quat2rot(qt):
    v, w = qt[:3], qt[3]
    return (w * w - v @ v) * np.eye(3) + 2 * np.outer(v, v) + 2 * w * hat(v)


def floating_config(urdf, base_link, ee_link=None, builders=None, kin=None, dyn=None, reg=None, floating=None):
    cfg = {
        "pipeline": {"loaders": ["urdf_loader", "floating_loader"],
                     "builders": builders or ["kin_builder", "dyn_builder", "reg_builder"], "generators": []},
        "urdf_loader": {"urdf_path": str(urdf), "base_link": base_link, "gravity": {"value": [0, 0, -9.81]}},
        "floating_loader": floating or {},
        "kin_builder": kin or {}, "dyn_builder": dyn or {}, "reg_builder": reg or {},
    }
    if ee_link:
        cfg["urdf_loader"]["ee_link"] = ee_link
    return cfg


class FloatingModel:
    """A thunder floating-base robot and the matching Pinocchio free-flyer model."""

    def __init__(self, urdf, base_link, ee_link=None, twist="world", **options):
        self.urdf = str(urdf)
        self.body = (twist == "body")
        self.robot = Config(floating_config(urdf, base_link, ee_link, floating={"twist": twist}, **options)).execute(robot_name="fb", no_generation=True)
        self.names = self.robot.get_vector_string("jointsName")
        self.dims = self.robot.get_vector_int("jointsDimension")
        self.n = self.robot.get_int("ndof")
        self.model = pin.buildModelFromUrdf(self.urdf, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        # URDF joint and frame link of every thunder node: node <link> holds the joint leaving <link> (the k-th child
        # joint for a branch node <link>_<k>), its frame is that joint's child link, a terminal node is its own link
        children = {}
        for j in ET.parse(self.urdf).getroot().iter("joint"):
            children.setdefault(j.find("parent").get("link"), []).append((j.get("name"), j.find("child").get("link")))
        self.node_joint, self.node_frame = [], []
        for name in self.names:
            link, k = name, 0
            if name not in children and "_" in name and name.rsplit("_", 1)[1].isdigit():
                link, k = name.rsplit("_", 1)[0], int(name.rsplit("_", 1)[1])
            joint, child = children[link][k] if link in children else (None, name)
            self.node_joint.append(joint)
            self.node_frame.append(child)
        # P: thunder joint velocities -> Pinocchio joint velocities (v indices after the 6 free-flyer entries).
        # Configurations: a continuous joint has 2 entries in Pinocchio (cos, sin), see q_joints_pin.
        self.P = np.zeros((self.n, self.n))
        self.pin_joints = []
        col = 0
        for joint, dim in zip(self.node_joint, self.dims):
            if dim == 0:
                continue
            jm = self.model.joints[self.model.getJointId(joint)]
            self.P[jm.idx_v - 6, col] = 1.0
            self.pin_joints.append((jm.idx_q, jm.nq))
            col += 1

    def q_joints_pin(self, pose, q):
        q_pin = np.zeros(self.model.nq)
        q_pin[:7] = pose
        for angle, (idx, nq) in zip(q, self.pin_joints):
            q_pin[idx:idx + nq] = [angle] if nq == 1 else [np.cos(angle), np.sin(angle)]
        return q_pin

    def set_state(self, pose, q, u, du=None, ur=None, dur=None):
        r = _Setter(self.robot)
        r.set("pose_base", pose)
        r.set("q", q)
        r.set("dq", u[6:])
        r.set("twist_base", u[:6])
        if du is not None:
            r.set("ddq", du[6:])
            r.set("acc_base", du[:6])
        if ur is not None:
            r.set("dqr", ur[6:])
            r.set("twist_base_r", ur[:6])
            r.set("ddqr", dur[6:])
            r.set("acc_base_r", dur[:6])
        # Pinocchio state: v_pin = T u
        R = quat2rot(pose[3:])
        nv = 6 + self.n
        self.T = np.zeros((nv, nv))
        self.T[6:, 6:] = self.P
        self.dT = np.zeros((nv, nv))
        if self.body:
            self.T[:6, :6] = np.eye(6)
        else:
            w_b = R.T @ u[3:6]
            self.T[:3, :3] = R.T
            self.T[3:6, 3:6] = R.T
            self.dT[:3, :3] = -hat(w_b) @ R.T
            self.dT[3:6, 3:6] = -hat(w_b) @ R.T
        self.q_pin = self.q_joints_pin(pose, q)
        self.v_pin = self.T @ u

    def get(self, name):
        return np.array(self.robot.get_value(name))


class _Setter:
    """robot.set with numpy vectors"""

    def __init__(self, robot):
        self.robot = robot

    def set(self, name, value):
        if not self.robot.set(name, casadi.DM(np.asarray(value, dtype=float))):
            raise RuntimeError(f"cannot set {name}")


def random_state(rng, n):
    pose = np.r_[rng.uniform(-1, 1, 3), rng.normal(size=4)]
    pose[3:] /= np.linalg.norm(pose[3:])
    return pose, rng.uniform(-1, 1, n), rng.normal(size=6 + n), rng.normal(size=6 + n)


@unittest.skipIf(pin is None, "Pinocchio not available (pip install --user pin)")
class TestFloatingBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.models = {}
        for twist in ("world", "body"):
            cls.models[f"biped_tree/{twist}"] = FloatingModel(
                FIXTURES / "floating" / "biped_tree.urdf", "base_link", twist=twist,
                kin={"compute_derivatives": True}, dyn={"compute_derivatives": True, "compute_J_cm": True})
            cls.models[f"franka_finger/{twist}"] = FloatingModel(
                FIXTURES / "franka" / "franka_finger.urdf", "panda_link0", "panda_leftfinger", twist=twist)

    def assertClose(self, a, b, what):
        err = np.abs(np.asarray(a) - np.asarray(b)).max()
        self.assertLess(err, TOL * max(1.0, np.abs(b).max()), f"{what}: max error {err:.3e}")

    def test_dynamics(self):
        rng = np.random.default_rng(0)
        for key, m in self.models.items():
            for trial in range(5):
                with self.subTest(robot=key, trial=trial):
                    pose, q, u, du = random_state(rng, m.n)
                    ur, dur = rng.normal(size=6 + m.n), rng.normal(size=6 + m.n)
                    m.set_state(pose, q, u, du, ur, dur)
                    T, dT, qp, vp = m.T, m.dT, m.q_pin, m.v_pin
                    M_pin = pin.crba(m.model, m.data, qp)
                    M_pin = np.triu(M_pin) + np.triu(M_pin, 1).T
                    G_pin = pin.computeGeneralizedGravity(m.model, m.data, qp)
                    C_pin = pin.computeCoriolisMatrix(m.model, m.data, qp, vp)
                    M = T.T @ M_pin @ T
                    G = T.T @ G_pin
                    C = T.T @ (C_pin @ T + M_pin @ dT)
                    tau = T.T @ pin.rnea(m.model, m.data, qp, vp, T @ du + dT @ u)
                    Cdq = T.T @ pin.rnea(m.model, m.data, qp, vp, dT @ u) - G
                    self.assertClose(m.get("M"), M, "M")
                    self.assertClose(m.get("G").ravel(), G, "G")
                    self.assertClose(m.get("Cdq").ravel(), Cdq, "Cdq")
                    self.assertClose(m.get("C"), C, "C (matrix, vs computeCoriolisMatrix)")
                    self.assertClose(m.get("tau").ravel(), tau, "tau")
                    self.assertClose(m.get("tau_r").ravel(), M @ dur + C @ ur + G, "tau_r")
                    par_REG = m.get("par_REG").ravel()
                    self.assertClose(m.get("Y") @ par_REG, tau, "Y par_REG")
                    self.assertClose(m.get("Yr") @ par_REG, M @ dur + C @ ur + G, "Yr par_REG")

    def test_kinematics(self):
        rng = np.random.default_rng(1)
        for key, m in self.models.items():
            for trial in range(3):
                with self.subTest(robot=key, trial=trial):
                    pose, q, u, _ = random_state(rng, m.n)
                    m.set_state(pose, q, u)
                    pin.computeJointJacobians(m.model, m.data, m.q_pin)
                    pin.computeJointJacobiansTimeVariation(m.model, m.data, m.q_pin, m.v_pin)
                    pin.updateFramePlacements(m.model, m.data)
                    lwa = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                    functions = {f.name for f in m.robot.get_functions()}
                    for i, frame in enumerate(m.node_frame):
                        fid = m.model.getFrameId(frame)
                        J_pin = pin.getFrameJacobian(m.model, m.data, fid, lwa)
                        self.assertClose(m.get(f"T_w_{i}"), m.data.oMf[fid].homogeneous, f"T_w_{i} ({frame})")
                        self.assertClose(m.get(f"J_{i}"), J_pin @ m.T, f"J_{i} ({frame})")
                        name = m.names[i]
                        if f"J_{name}_dot" in functions:
                            dJ_pin = pin.getFrameJacobianTimeVariation(m.model, m.data, fid, lwa)
                            self.assertClose(m.get(f"J_{name}_dot"), dJ_pin @ m.T + J_pin @ m.dT, f"J_{name}_dot")

    def test_com_jacobians(self):
        # J_cm_<i> (centre of mass of link i, the base body included) against Pinocchio's Jacobian of the centre of
        # mass of the whole robot: sum_i m_i J_cm_i (linear rows) / m = J_com_pin T. biped_tree loads every link;
        # node <link> holds the body of <link>, a branch node <link>_<k> has no mass (par_DYN is numeric, so read the URDF).
        rng = np.random.default_rng(3)
        for twist in ("world", "body"):
            m = self.models[f"biped_tree/{twist}"]
            link_mass = {l.get("name"): float(l.find("inertial/mass").get("value"))
                         for l in ET.parse(m.urdf).getroot().iter("link") if l.find("inertial") is not None}
            masses = np.array([link_mass.get(name, 0.0) for name in m.names])
            for trial in range(3):
                with self.subTest(twist=twist, trial=trial):
                    pose, q, u, _ = random_state(rng, m.n)
                    m.set_state(pose, q, u)
                    J_com_pin = pin.jacobianCenterOfMass(m.model, m.data, m.q_pin)
                    self.assertClose(masses.sum(), pin.computeTotalMass(m.model), "total mass")
                    J_com = sum(mi * m.get(f"J_cm_{i}")[:3] for i, mi in enumerate(masses)) / masses.sum()
                    self.assertClose(J_com, J_com_pin @ m.T, "sum m_i J_cm_i / m")

    def test_properties_and_derivatives(self):
        # skew dM/dt - 2C, C u = Cdq, and M_dot, G_dot against central finite differences along the exact motion
        rng = np.random.default_rng(2)
        h = 1e-6

        def qmul(a, b):
            return np.r_[a[3] * b[:3] + b[3] * a[:3] + np.cross(a[:3], b[:3]), a[3] * b[3] - a[:3] @ b[:3]]

        def moved(m, pose, q, u, t):
            # constant twist for a time t: world twist Exp(w t) (x) quat (left), body twist quat (x) Exp(w t) (right)
            phi = u[3:6] * t
            a = np.linalg.norm(phi)
            e = np.r_[np.sin(a / 2) * phi / a, np.cos(a / 2)] if a > 0 else np.array([0, 0, 0, 1.0])
            if m.body:
                return np.r_[pose[:3] + quat2rot(pose[3:]) @ u[:3] * t, qmul(pose[3:], e)], q + u[6:] * t
            return np.r_[pose[:3] + u[:3] * t, qmul(e, pose[3:])], q + u[6:] * t

        for twist in ("world", "body"):
          m = self.models[f"biped_tree/{twist}"]
          for trial in range(3):
            with self.subTest(twist=twist, trial=trial):
                pose, q, u, du = random_state(rng, m.n)
                m.set_state(pose, q, u, du)
                M, C, Cdq = m.get("M"), m.get("C"), m.get("Cdq").ravel()
                M_dot, G_dot = m.get("M_dot"), m.get("G_dot").ravel()
                self.assertClose(C @ u, Cdq, "C u = Cdq")
                S = M_dot - 2 * C
                self.assertClose(S + S.T, np.zeros_like(S), "dM/dt - 2C skew")
                values = []
                for t in (h, -h):
                    pose_t, q_t = moved(m, pose, q, u, t)
                    m.set_state(pose_t, q_t, u)
                    values.append((m.get("M"), m.get("G").ravel()))
                self.assertLess(np.abs((values[0][0] - values[1][0]) / (2 * h) - M_dot).max(), 1e-5, "M_dot vs finite differences")
                self.assertLess(np.abs((values[0][1] - values[1][1]) / (2 * h) - G_dot).max(), 1e-5, "G_dot vs finite differences")

    def test_refused_methods(self):
        urdf = FIXTURES / "floating" / "biped_tree.urdf"
        cases = {
            "christoffel C": dict(dyn={"C_method": "christoffel"}),
            "lagrange dynamics": dict(dyn={"dynamics_method": "lagrange"}),
            "C_std": dict(dyn={"compute_C_std": True}),
            "lagrange regressors": dict(reg={"regressor_method": "lagrange"}),
            "base not the root": dict(floating={"base": "abad_L"}),
            "unknown twist convention": dict(floating={"twist": "spatial"}),
        }
        for what, options in cases.items():
            with self.subTest(case=what):
                with self.assertRaises(Exception):
                    Config(floating_config(urdf, "base_link", **options)).execute(robot_name="fb", no_generation=True)
        # a URDF with world + floating joint (e.g. written for MuJoCo): refused from its root, fine from base_link
        text = urdf.read_text().replace('<robot name="biped_tree">', '<robot name="biped_tree">\n  <link name="world"/>\n'
               '  <joint name="world_joint" type="floating">\n    <parent link="world"/>\n    <child link="base_link"/>\n  </joint>')
        with tempfile.TemporaryDirectory() as tmp:
            with_world = Path(tmp) / "biped_world.urdf"
            with_world.write_text(text)
            with self.subTest(case="URDF floating joint loaded from the URDF root"):
                cfg = floating_config(with_world, "base_link")
                del cfg["urdf_loader"]["base_link"]
                with self.assertRaises(Exception):
                    Config(cfg).execute(robot_name="fb", no_generation=True)
            with self.subTest(case="URDF floating joint skipped with base_link = its child"):
                reference = Config(floating_config(urdf, "base_link")).execute(robot_name="fb", no_generation=True)
                robot = Config(floating_config(with_world, "base_link")).execute(robot_name="fb", no_generation=True)
                self.assertClose(np.array(robot.get_value("M")), np.array(reference.get_value("M")), "M with world in the URDF")
        with self.subTest(case="floating_loader before the structure"):
            cfg = floating_config(urdf, "base_link")
            cfg["pipeline"]["loaders"] = ["floating_loader", "urdf_loader"]
            with self.assertRaises(Exception):
                Config(cfg).execute(robot_name="fb", no_generation=True)


if __name__ == "__main__":
    unittest.main()
