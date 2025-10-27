"""
A batched Python/PyTorch version of the thunder_robot C++ template.

"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import torch
import yaml


STD_PAR_LINK: int = 10  # [mass, 3x CoM or m*CoM, 6x inertia]


TensorLike = Union[torch.Tensor, Sequence[float], Sequence[int]]

def _as_tensor(x: TensorLike, device: torch.device, dtype: torch.dtype) -> torch.Tensor:
    if isinstance(x, torch.Tensor):
        return x.to(device=device, dtype=dtype)
    return torch.as_tensor(x, device=device, dtype=dtype)


def _expand_to_batch(x: TensorLike, batch_size: int, dim: int, device: torch.device, dtype: torch.dtype) -> torch.Tensor:
    """
    Accepts shapes:
    - (dim,) -> expand to (B, dim)
    - (B, dim) -> returned as-is (moved to device/dtype)
    """
    t = _as_tensor(x, device, dtype)
    if t.ndim == 1:
        if t.shape[0] != dim:
            raise ValueError(f"Expected vector of shape ({dim},), got {tuple(t.shape)}")
        return t.unsqueeze(0).expand(batch_size, dim).clone()
    if t.ndim == 2:
        if t.shape != (batch_size, dim):
            raise ValueError(f"Expected tensor of shape ({batch_size}, {dim}), got {tuple(t.shape)}")
        return t
    raise ValueError(f"Unsupported tensor rank {t.ndim}, expected 1 or 2")


def hat(v: torch.Tensor) -> torch.Tensor:
    """
    Skew-symmetric matrix (hat operator) for vectors v of shape (..., 3).
    Returns a tensor of shape (..., 3, 3).
    """
    if v.shape[-1] != 3:
        raise ValueError(f"hat expects last dimension=3, got {v.shape}")
    zeros = torch.zeros_like(v[..., 0])
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    out = torch.stack([
        torch.stack([zeros, -vz, vy], dim=-1),
        torch.stack([vz, zeros, -vx], dim=-1),
        torch.stack([-vy, vx, zeros], dim=-1),
    ], dim=-2)
    return out


class ThunderRobotTorch:
    """
    Batched PyTorch wrapper mirroring the thunder_robot C++ template (core state + inertial conversions).

    Constructor parameters
    - n_joints: number of joints
    - numElasticJoints: number of elastic joints (can be 0)
    - K_order, D_order, Dl_order, Dm_order: polynomial orders for elastic/friction params
    - batch_size: batch dimension size
    - device, dtype: torch device and dtype
    - isElasticJoint: optional boolean/int mask of length n_joints
    - robotName: optional robot identifier string
    """

    def __init__(
        self,
        n_joints: int,
        numElasticJoints: int,
        K_order: int,
        D_order: int,
        Dl_order: int,
        Dm_order: int,
        batch_size: int = 1,
        device: Union[str, torch.device] = "cpu",
        dtype: torch.dtype = torch.double,
        isElasticJoint: Optional[Sequence[int]] = None,
        robotName: str = "",
    ) -> None:
        self.robotName = robotName
        self.batch_size = int(batch_size)
        self.device = torch.device(device)
        self.dtype = dtype

        # Sizes
        self.n_joints = int(n_joints)
        self.numElasticJoints = int(numElasticJoints)
        self.K_order = int(K_order)
        self.D_order = int(D_order)
        self.Dl_order = int(Dl_order)
        self.Dm_order = int(Dm_order)
        self.numParDYN = STD_PAR_LINK * self.n_joints
        self.numParREG = STD_PAR_LINK * self.n_joints

        # Masks/flags (non-batched)
        if isElasticJoint is None:
            self.isElasticJoint: List[int] = [0] * self.n_joints
        else:
            if len(isElasticJoint) != self.n_joints:
                raise ValueError("isElasticJoint length must equal n_joints")
            self.isElasticJoint = [int(v) for v in isElasticJoint]

        # Symbolic masks (structure-only, not batched)
        self.DHtable_symb: List[int] = [0] * (self.n_joints * 4)
        self.gravity_symb: List[int] = [0] * 3
        self.world2L0_symb: List[int] = [0] * 6
        self.Ln2EE_symb: List[int] = [0] * 6

        # Allocate tensors
        self._resize_variables()

    # ----- allocation -----
    def _resize_variables(self) -> None:
        B = self.batch_size
        nj = self.n_joints
        ne = self.numElasticJoints
        dev, dt = self.device, self.dtype

        # State
        self.q = torch.zeros((B, nj), device=dev, dtype=dt)
        self.dq = torch.zeros((B, nj), device=dev, dtype=dt)
        self.ddq = torch.zeros((B, nj), device=dev, dtype=dt)
        self.d3q = torch.zeros((B, nj), device=dev, dtype=dt)
        self.d4q = torch.zeros((B, nj), device=dev, dtype=dt)
        self.dqr = torch.zeros((B, nj), device=dev, dtype=dt)
        self.ddqr = torch.zeros((B, nj), device=dev, dtype=dt)

        # Elastic joint states
        self.x = torch.zeros((B, ne), device=dev, dtype=dt)
        self.dx = torch.zeros((B, ne), device=dev, dtype=dt)
        self.ddx = torch.zeros((B, ne), device=dev, dtype=dt)
        self.ddxr = torch.zeros((B, ne), device=dev, dtype=dt)

        # External wrench (world or EE frame depending on usage)
        self.w = torch.zeros((B, 6), device=dev, dtype=dt)

        # Parameters (batched)
        self.par_REG = torch.zeros((B, self.numParREG), device=dev, dtype=dt)
        self.par_DYN = torch.zeros((B, self.numParDYN), device=dev, dtype=dt)
        self.par_Dl = torch.zeros((B, self.Dl_order * nj), device=dev, dtype=dt)
        self.par_K = torch.zeros((B, self.K_order * ne), device=dev, dtype=dt)
        self.par_D = torch.zeros((B, self.D_order * ne), device=dev, dtype=dt)
        self.par_Dm = torch.zeros((B, self.Dm_order * ne), device=dev, dtype=dt)
        self.par_Mm = torch.zeros((B, ne), device=dev, dtype=dt)

        # Kinematics / environment
        self.par_DHtable = torch.zeros((B, nj * 4), device=dev, dtype=dt)
        self.par_world2L0 = torch.zeros((B, 6), device=dev, dtype=dt)
        self.par_Ln2EE = torch.zeros((B, 6), device=dev, dtype=dt)
        self.par_gravity = torch.zeros((B, 3), device=dev, dtype=dt)

    # ----- getters (selected) -----
    def get_numJoints(self) -> int:
        return self.n_joints

    def get_numParDYN(self) -> int:
        return self.numParDYN

    def get_numParREG(self) -> int:
        return self.numParREG

    # ----- setters for state -----
    def setArguments(self, q: TensorLike, dq: TensorLike, dqr: TensorLike, ddqr: TensorLike) -> None:
        self.q = _expand_to_batch(q, self.batch_size, self.n_joints, self.device, self.dtype)
        self.dq = _expand_to_batch(dq, self.batch_size, self.n_joints, self.device, self.dtype)
        self.dqr = _expand_to_batch(dqr, self.batch_size, self.n_joints, self.device, self.dtype)
        self.ddqr = _expand_to_batch(ddqr, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_q(self, q: TensorLike) -> None:
        self.q = _expand_to_batch(q, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_dq(self, dq: TensorLike) -> None:
        self.dq = _expand_to_batch(dq, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_ddq(self, ddq: TensorLike) -> None:
        self.ddq = _expand_to_batch(ddq, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_d3q(self, d3q: TensorLike) -> None:
        self.d3q = _expand_to_batch(d3q, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_d4q(self, d4q: TensorLike) -> None:
        self.d4q = _expand_to_batch(d4q, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_dqr(self, dqr: TensorLike) -> None:
        self.dqr = _expand_to_batch(dqr, self.batch_size, self.n_joints, self.device, self.dtype)

    def set_ddqr(self, ddqr: TensorLike) -> None:
        self.ddqr = _expand_to_batch(ddqr, self.batch_size, self.n_joints, self.device, self.dtype)

    # Elastic states
    def set_x(self, x: TensorLike) -> None:
        self.x = _expand_to_batch(x, self.batch_size, self.numElasticJoints, self.device, self.dtype)

    def set_dx(self, dx: TensorLike) -> None:
        self.dx = _expand_to_batch(dx, self.batch_size, self.numElasticJoints, self.device, self.dtype)

    def set_ddx(self, ddx: TensorLike) -> None:
        self.ddx = _expand_to_batch(ddx, self.batch_size, self.numElasticJoints, self.device, self.dtype)

    def set_ddxr(self, ddxr: TensorLike) -> None:
        self.ddxr = _expand_to_batch(ddxr, self.batch_size, self.numElasticJoints, self.device, self.dtype)

    def set_w(self, w: TensorLike) -> None:
        self.w = _expand_to_batch(w, self.batch_size, 6, self.device, self.dtype)

    # ----- setters for parameters -----
    def set_par_DYN(self, par: TensorLike, update_REG: bool = False) -> None:
        t = _as_tensor(par, self.device, self.dtype)
        if t.shape != (self.batch_size, self.numParDYN):
            raise ValueError(f"Invalid shape for par_DYN. Expected {(self.batch_size, self.numParDYN)}, got {tuple(t.shape)}")
        self.par_DYN = t
        if update_REG:
            self.update_inertial_REG()

    def set_par_REG(self, par: TensorLike, update_DYN: bool = False) -> None:
        t = _as_tensor(par, self.device, self.dtype)
        if t.shape != (self.batch_size, self.numParREG):
            raise ValueError(f"Invalid shape for par_REG. Expected {(self.batch_size, self.numParREG)}, got {tuple(t.shape)}")
        self.par_REG = t
        if update_DYN:
            self.update_inertial_DYN()

    def set_par_K(self, par: TensorLike) -> None:
        self.par_K = _expand_to_batch(par, self.batch_size, self.K_order * self.numElasticJoints, self.device, self.dtype)

    def set_par_D(self, par: TensorLike) -> None:
        self.par_D = _expand_to_batch(par, self.batch_size, self.D_order * self.numElasticJoints, self.device, self.dtype)

    def set_par_Dm(self, par: TensorLike) -> None:
        self.par_Dm = _expand_to_batch(par, self.batch_size, self.Dm_order * self.numElasticJoints, self.device, self.dtype)

    def set_par_Mm(self, par: TensorLike) -> None:
        self.par_Mm = _expand_to_batch(par, self.batch_size, self.numElasticJoints, self.device, self.dtype)

    def set_par_Dl(self, par: TensorLike) -> None:
        self.par_Dl = _expand_to_batch(par, self.batch_size, self.Dl_order * self.n_joints, self.device, self.dtype)

    def set_par_DHtable(self, par: TensorLike) -> None:
        self.par_DHtable = _expand_to_batch(par, self.batch_size, self.n_joints * 4, self.device, self.dtype)

    def set_par_gravity(self, par: TensorLike) -> None:
        self.par_gravity = _expand_to_batch(par, self.batch_size, 3, self.device, self.dtype)

    def set_par_world2L0(self, par: TensorLike) -> None:
        self.par_world2L0 = _expand_to_batch(par, self.batch_size, 6, self.device, self.dtype)

    def set_par_Ln2EE(self, par: TensorLike) -> None:
        self.par_Ln2EE = _expand_to_batch(par, self.batch_size, 6, self.device, self.dtype)

    # ----- getters for parameters (return views) -----
    def get_par_DYN(self) -> torch.Tensor:
        return self.par_DYN

    def get_par_REG(self) -> torch.Tensor:
        return self.par_REG

    def get_par_K(self) -> torch.Tensor:
        return self.par_K

    def get_par_D(self) -> torch.Tensor:
        return self.par_D

    def get_par_Dm(self) -> torch.Tensor:
        return self.par_Dm

    def get_par_Mm(self) -> torch.Tensor:
        return self.par_Mm

    def get_par_Dl(self) -> torch.Tensor:
        return self.par_Dl

    def get_par_DHtable(self) -> torch.Tensor:
        return self.par_DHtable

    def get_par_gravity(self) -> torch.Tensor:
        return self.par_gravity

    def get_par_world2L0(self) -> torch.Tensor:
        return self.par_world2L0

    def get_par_Ln2EE(self) -> torch.Tensor:
        return self.par_Ln2EE

    # ----- inertial conversions -----
    def update_inertial_DYN(self) -> None:
        """
        Convert REG -> DYN using the parallel axis theorem in batched/vectorized form.
        REG packs [m, m*CoM(3), I(6)], DYN packs [m, CoM(3), I_cm(6)].
        """
        B, nj = self.batch_size, self.n_joints
        p_reg = self.par_REG.view(B, nj, STD_PAR_LINK)
        mass = p_reg[..., 0]  # (B, nj)
        m_CoM = p_reg[..., 1:4]  # (B, nj, 3)
        # Avoid division by zero
        CoM = torch.where(mass.unsqueeze(-1) != 0, m_CoM / mass.unsqueeze(-1), torch.zeros_like(m_CoM))
        H = hat(CoM.reshape(-1, 3)).view(B, nj, 3, 3)
        I_tmp = mass.view(B, nj, 1, 1) * (H @ H.transpose(-1, -2))
        I_tmp_v = torch.stack(
            [
                I_tmp[..., 0, 0],
                I_tmp[..., 0, 1],
                I_tmp[..., 0, 2],
                I_tmp[..., 1, 1],
                I_tmp[..., 1, 2],
                I_tmp[..., 2, 2],
            ],
            dim=-1,
        )  # (B, nj, 6)
        I_reg = p_reg[..., 4:10]
        I_dyn = I_reg - I_tmp_v
        p_dyn = torch.cat([mass.unsqueeze(-1), CoM, I_dyn], dim=-1)
        self.par_DYN = p_dyn.view(B, nj * STD_PAR_LINK)

    def update_inertial_REG(self) -> None:
        """
        Convert DYN -> REG using the parallel axis theorem in batched/vectorized form.
        DYN packs [m, CoM(3), I_cm(6)], REG packs [m, m*CoM(3), I_o(6)].
        """
        B, nj = self.batch_size, self.n_joints
        p_dyn = self.par_DYN.view(B, nj, STD_PAR_LINK)
        mass = p_dyn[..., 0]  # (B, nj)
        CoM = p_dyn[..., 1:4]  # (B, nj, 3)
        m_CoM = mass.unsqueeze(-1) * CoM
        H = hat(CoM.reshape(-1, 3)).view(B, nj, 3, 3)
        I_tmp = mass.view(B, nj, 1, 1) * (H @ H.transpose(-1, -2))
        I_tmp_v = torch.stack(
            [
                I_tmp[..., 0, 0],
                I_tmp[..., 0, 1],
                I_tmp[..., 0, 2],
                I_tmp[..., 1, 1],
                I_tmp[..., 1, 2],
                I_tmp[..., 2, 2],
            ],
            dim=-1,
        )  # (B, nj, 6)
        I_dyn = p_dyn[..., 4:10]
        I_reg = I_dyn + I_tmp_v
        p_reg = torch.cat([mass.unsqueeze(-1), m_CoM, I_reg], dim=-1)
        self.par_REG = p_reg.view(B, nj * STD_PAR_LINK)


    def load_par_REG(self, file_path: str, update_DYN: bool = False, batch: Optional[int] = None) -> torch.Tensor:
        """
        Load REG inertial parameters from YAML (keys: mass, m_CoM_x/y/z, Ixx/xy/xz/yy/yz/zz) and set par_REG.
        If batch is None, applies to all batches; else applies only to the specified batch index.
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        dyn = config.get("dynamics", {})
        # Prefer ordered keys link1..linkN if available
        keys = [f"link{i+1}" for i in range(self.n_joints)]
        entries = []
        for i in range(self.n_joints):
            k = keys[i]
            if k in dyn:
                entries.append(dyn[k])
            else:
                # Fallback to iteration order
                try:
                    entries = [node for _, node in list(dyn.items())[: self.n_joints]]
                except AttributeError:
                    entries = []
                break

        if not entries and dyn:
            # Fallback again if above failed
            entries = [node for _, node in list(dyn.items())[: self.n_joints]]

        par_reg_vec = torch.zeros((self.numParREG,), device=self.device, dtype=self.dtype)
        for i, node in enumerate(entries):
            if i >= self.n_joints:
                break
            inertial = node.get("inertial", {})
            mass = float(inertial.get("mass", 0.0))
            m_cmx = float(inertial.get("m_CoM_x", 0.0))
            m_cmy = float(inertial.get("m_CoM_y", 0.0))
            m_cmz = float(inertial.get("m_CoM_z", 0.0))
            xx = float(inertial.get("Ixx", 0.0))
            xy = float(inertial.get("Ixy", 0.0))
            xz = float(inertial.get("Ixz", 0.0))
            yy = float(inertial.get("Iyy", 0.0))
            yz = float(inertial.get("Iyz", 0.0))
            zz = float(inertial.get("Izz", 0.0))
            seg = torch.tensor([mass, m_cmx, m_cmy, m_cmz, xx, xy, xz, yy, yz, zz], device=self.device, dtype=self.dtype)
            par_reg_vec[i * STD_PAR_LINK : (i + 1) * STD_PAR_LINK] = seg

        if batch is None:
            self.par_REG = par_reg_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
        else:
            self.par_REG[batch] = par_reg_vec
        if update_DYN:
            self.update_inertial_DYN()
        return self.par_REG

    def load_conf(self, file_path: str, update_REG: bool = False, batch: Optional[int] = None) -> None:
        """
        Load a complete configuration YAML (kinematics frames, dynamics in DYN, gravity, elastic).
        Mirrors thunder_robot::load_conf.
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        bidx = slice(None) if batch is None else batch

        # --- Kinematics (DH table) ---
        kin = config.get("kinematics")
        if kin:
            self.DHtable_symb = list(kin.get("symb", [0] * (self.n_joints * 4)))
            dh_vect = kin.get("DH", [])
            # Expect length n_joints*4, pad/truncate as needed
            target_len = self.n_joints * 4
            if len(dh_vect) < target_len:
                dh_vect = list(dh_vect) + [0.0] * (target_len - len(dh_vect))
            if len(dh_vect) > target_len:
                dh_vect = list(dh_vect)[:target_len]
            self.par_DHtable[bidx] = _as_tensor(dh_vect, self.device, self.dtype)

        # --- Frames: Base_to_L0 ---
        base = config.get("Base_to_L0")
        if base:
            self.world2L0_symb = list(base.get("symb", [0] * 6))
            tr = base.get("tr", [0.0, 0.0, 0.0])
            ypr = base.get("ypr", [0.0, 0.0, 0.0])
            v6 = list(tr) + list(ypr)
            if len(v6) != 6:
                v6 = (v6 + [0.0] * 6)[:6]
            self.par_world2L0[bidx] = _as_tensor(v6, self.device, self.dtype)

        # --- Frames: Ln_to_EE ---
        ee = config.get("Ln_to_EE")
        if ee:
            self.Ln2EE_symb = list(ee.get("symb", [0] * 6))
            tr = ee.get("tr", [0.0, 0.0, 0.0])
            ypr = ee.get("ypr", [0.0, 0.0, 0.0])
            v6 = list(tr) + list(ypr)
            if len(v6) != 6:
                v6 = (v6 + [0.0] * 6)[:6]
            self.par_Ln2EE[bidx] = _as_tensor(v6, self.device, self.dtype)

        # --- Dynamics (DYN form) + joint friction Dl ---
        dyn = config.get("dynamics")
        if dyn:
            keys = [f"link{i+1}" for i in range(self.n_joints)]
            entries = []
            for i in range(self.n_joints):
                k = keys[i]
                if k in dyn:
                    entries.append(dyn[k])
                else:
                    try:
                        entries = [node for _, node in list(dyn.items())[: self.n_joints]]
                    except AttributeError:
                        entries = []
                    break
            if not entries and dyn:
                entries = [node for _, node in list(dyn.items())[: self.n_joints]]

            par_dyn_vec = torch.zeros((self.numParDYN,), device=self.device, dtype=self.dtype)
            par_Dl_vec = torch.zeros((self.Dl_order * self.n_joints,), device=self.device, dtype=self.dtype)
            for i, node in enumerate(entries):
                if i >= self.n_joints:
                    break
                inertial = node.get("inertial", {})
                mass = float(inertial.get("mass", 0.0))
                cmx = float(inertial.get("CoM_x", 0.0))
                cmy = float(inertial.get("CoM_y", 0.0))
                cmz = float(inertial.get("CoM_z", 0.0))
                xx = float(inertial.get("Ixx", 0.0))
                xy = float(inertial.get("Ixy", 0.0))
                xz = float(inertial.get("Ixz", 0.0))
                yy = float(inertial.get("Iyy", 0.0))
                yz = float(inertial.get("Iyz", 0.0))
                zz = float(inertial.get("Izz", 0.0))
                seg = torch.tensor([mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz], device=self.device, dtype=self.dtype)
                par_dyn_vec[i * STD_PAR_LINK : (i + 1) * STD_PAR_LINK] = seg

                # friction Dl polynomial
                fric = node.get("friction", {})
                Dl = fric.get("Dl", [0.0] * self.Dl_order)
                if len(Dl) < self.Dl_order:
                    Dl = list(Dl) + [0.0] * (self.Dl_order - len(Dl))
                if len(Dl) > self.Dl_order:
                    Dl = list(Dl)[: self.Dl_order]
                par_Dl_vec[self.Dl_order * i : self.Dl_order * (i + 1)] = _as_tensor(Dl, self.device, self.dtype)

            if batch is None:
                self.par_DYN = par_dyn_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
                self.par_Dl = par_Dl_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
            else:
                self.par_DYN[batch] = par_dyn_vec
                self.par_Dl[batch] = par_Dl_vec
            if update_REG:
                self.update_inertial_REG()

        # --- Gravity ---
        grav = config.get("gravity")
        if grav:
            self.gravity_symb = list(grav.get("symb", [0] * 3))
            gval = grav.get("value", [0.0, 0.0, 0.0])
            if len(gval) != 3:
                gval = (list(gval) + [0.0, 0.0, 0.0])[:3]
            self.par_gravity[bidx] = _as_tensor(gval, self.device, self.dtype)

        # --- Elastic ---
        elastic = config.get("elastic")
        if elastic and "joints" in elastic:
            joints = elastic["joints"]
            # joints is expected as mapping; iterate deterministic order link1.. or fallback to items
            if isinstance(joints, dict):
                joint_nodes = [joints.get(f"joint{i+1}") for i in range(self.numElasticJoints)]
                joint_nodes = [jn for jn in joint_nodes if jn is not None] or [node for _, node in joints.items()]
            elif isinstance(joints, list):
                joint_nodes = joints
            else:
                joint_nodes = []

            par_K_vec = torch.zeros((self.K_order * self.numElasticJoints,), device=self.device, dtype=self.dtype)
            par_D_vec = torch.zeros((self.D_order * self.numElasticJoints,), device=self.device, dtype=self.dtype)
            par_Dm_vec = torch.zeros((self.Dm_order * self.numElasticJoints,), device=self.device, dtype=self.dtype)
            par_Mm_vec = torch.zeros((self.numElasticJoints,), device=self.device, dtype=self.dtype)

            idx = 0
            for node in joint_nodes:
                if idx >= self.numElasticJoints:
                    break
                K = list(node.get("K", [0.0] * self.K_order))
                D = list(node.get("D", [0.0] * self.D_order))
                Dm = list(node.get("Dm", [0.0] * self.Dm_order))
                Mm = float(node.get("Mm", 0.0))
                if len(K) < self.K_order:
                    K += [0.0] * (self.K_order - len(K))
                if len(D) < self.D_order:
                    D += [0.0] * (self.D_order - len(D))
                if len(Dm) < self.Dm_order:
                    Dm += [0.0] * (self.Dm_order - len(Dm))
                par_K_vec[self.K_order * idx : self.K_order * (idx + 1)] = _as_tensor(K, self.device, self.dtype)
                par_D_vec[self.D_order * idx : self.D_order * (idx + 1)] = _as_tensor(D, self.device, self.dtype)
                par_Dm_vec[self.Dm_order * idx : self.Dm_order * (idx + 1)] = _as_tensor(Dm, self.device, self.dtype)
                par_Mm_vec[idx] = torch.tensor(Mm, device=self.device, dtype=self.dtype)
                idx += 1

            if batch is None:
                self.par_K = par_K_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
                self.par_D = par_D_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
                self.par_Dm = par_Dm_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
                self.par_Mm = par_Mm_vec.unsqueeze(0).expand(self.batch_size, -1).clone()
            else:
                self.par_K[batch] = par_K_vec
                self.par_D[batch] = par_D_vec
                self.par_Dm[batch] = par_Dm_vec
                self.par_Mm[batch] = par_Mm_vec

    def save_par_REG(self, path_yaml_DH_REG: str, batch: int = 0) -> None:
        """
        Save REG inertial parameters and link friction Dl to a YAML file (keys: mass, m_CoM_*, I..).
        """
        data = {"dynamics": {}}
        par_REG_b = self.par_REG[batch].detach().cpu()
        par_Dl_b = self.par_Dl[batch].detach().cpu() if self.par_Dl.numel() > 0 else torch.zeros(0)
        for i in range(self.n_joints):
            seg = par_REG_b[i * STD_PAR_LINK : (i + 1) * STD_PAR_LINK].tolist()
            Dl = par_Dl_b[self.Dl_order * i : self.Dl_order * (i + 1)].tolist() if self.Dl_order > 0 else []
            data["dynamics"][f"link{i+1}"] = {
                "inertial": {
                    "mass": seg[0],
                    "m_CoM_x": seg[1],
                    "m_CoM_y": seg[2],
                    "m_CoM_z": seg[3],
                    "Ixx": seg[4],
                    "Ixy": seg[5],
                    "Ixz": seg[6],
                    "Iyy": seg[7],
                    "Iyz": seg[8],
                    "Izz": seg[9],
                },
                "friction": {"Dl": Dl},
            }
        with open(path_yaml_DH_REG, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def save_par_DYN(self, path_yaml_DH_DYN: str, batch: int = 0) -> None:
        """
        Save DYN inertial parameters and link friction Dl to a YAML file (keys: mass, CoM_*, I..).
        """
        data = {"dynamics": {}}
        par_DYN_b = self.par_DYN[batch].detach().cpu()
        par_Dl_b = self.par_Dl[batch].detach().cpu() if self.par_Dl.numel() > 0 else torch.zeros(0)
        for i in range(self.n_joints):
            seg = par_DYN_b[i * STD_PAR_LINK : (i + 1) * STD_PAR_LINK].tolist()
            Dl = par_Dl_b[self.Dl_order * i : self.Dl_order * (i + 1)].tolist() if self.Dl_order > 0 else []
            data["dynamics"][f"link{i+1}"] = {
                "inertial": {
                    "mass": seg[0],
                    "CoM_x": seg[1],
                    "CoM_y": seg[2],
                    "CoM_z": seg[3],
                    "Ixx": seg[4],
                    "Ixy": seg[5],
                    "Ixz": seg[6],
                    "Iyy": seg[7],
                    "Iyz": seg[8],
                    "Izz": seg[9],
                },
                "friction": {"Dl": Dl},
            }
        with open(path_yaml_DH_DYN, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def save_par(self, par_file: str, batch: int = 0) -> int:
        """
        Save a snapshot of key parameter arrays similar to C++ save_par.
        Returns 1 on success, 0 on failure.
        """
        try:
            b = batch
            data = {
                "DHtable": self.par_DHtable[b].detach().cpu().tolist(),
                "world2L0": self.par_world2L0[b].detach().cpu().tolist(),
                "Ln2EE": self.par_Ln2EE[b].detach().cpu().tolist(),
                "gravity": self.par_gravity[b].detach().cpu().tolist(),
                "par_DYN": self.par_DYN[b].detach().cpu().tolist(),
                "par_REG": self.par_REG[b].detach().cpu().tolist(),
                "par_K": self.par_K[b].detach().cpu().tolist(),
                "par_Dl": self.par_Dl[b].detach().cpu().tolist(),
                "par_D": self.par_D[b].detach().cpu().tolist(),
                "par_Dm": self.par_Dm[b].detach().cpu().tolist(),
                "par_Mm": self.par_Mm[b].detach().cpu().tolist(),
            }
            with open(par_file, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
        except Exception:
            return 0
        return 1


