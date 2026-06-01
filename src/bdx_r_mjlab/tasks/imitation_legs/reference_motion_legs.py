"""Reference motion library for legs-only BDX-R (no head/neck).

Loads polynomial_coefficients_legs.pkl produced by:
  uv run scripts/auto_waddle.py --duck bdx_r_legs -j8 --sweep
  uv run scripts/fit_poly.py --duck bdx_r_legs

Placo legs joint order (from placo_defaults.json):
  0 Left_Hip_Yaw   1 Left_Hip_Roll   2 Left_Hip_Pitch  3 left_knee   4 Left_Ankle
  5 Right_Hip_Yaw  6 Right_Hip_Roll  7 Right_Hip_Pitch 8 right_knee  9 Right_Ankle

MjLab actuator order (from bdxr_legs.xml):
  0 Left_Hip_Yaw   1 Left_Hip_Roll   2 Left_Hip_Pitch  3 Left_Knee   4 Left_Ankle
  5 Right_Hip_Yaw  6 Right_Hip_Roll  7 Right_Hip_Pitch 8 Right_Knee  9 Right_Ankle

Orders are identical → identity remapping.
"""

import pickle
from pathlib import Path

import numpy as np
import torch

N_JOINTS = 10
# Identity mapping — placo and MjLab legs order are the same
POLY_FROM_MJLAB = list(range(N_JOINTS))

# Layout: root_pos(3) + root_quat(4) + jpos(10) + jvel(10) + foot(2) + lin_vel(3) + ang_vel(3) = 35
ROOT_POS_SLICE  = slice(0, 3)
ROOT_QUAT_SLICE = slice(3, 7)
JPOS_SLICE      = slice(7, 17)
JVEL_SLICE      = slice(17, 27)
FOOT_SLICE      = slice(27, 29)


class ReferenceMotionLibraryLegs:
    """Loads legs-only polynomial reference motions."""

    def __init__(self, pkl_path: str | Path, device: str = "cpu"):
        with open(pkl_path, "rb") as f:
            raw = pickle.load(f)

        vels, coeffs_list, periods = [], [], []
        for key, entry in raw.items():
            vx, vy, vtheta = [float(v) for v in key.split("_")]
            vels.append([vx, vy, vtheta])
            periods.append(entry["period"])

            n_dims = len(entry["coefficients"])
            n_c = len(entry["coefficients"]["dim_0"])
            mat = np.zeros((n_dims, n_c), dtype=np.float32)
            for d in range(n_dims):
                mat[d] = entry["coefficients"][f"dim_{d}"]
            coeffs_list.append(mat)

        self.velocities: torch.Tensor = torch.tensor(vels, dtype=torch.float32)
        self.coeff_mats: torch.Tensor = torch.tensor(np.array(coeffs_list), dtype=torch.float32)
        self.periods: torch.Tensor = torch.tensor(periods, dtype=torch.float32)
        self._poly_from_mjlab = torch.tensor(POLY_FROM_MJLAB, dtype=torch.long)
        self.n_clips = len(vels)
        self.to(device)

    def to(self, device: str) -> "ReferenceMotionLibraryLegs":
        self.velocities = self.velocities.to(device)
        self.coeff_mats = self.coeff_mats.to(device)
        self.periods = self.periods.to(device)
        self._poly_from_mjlab = self._poly_from_mjlab.to(device)
        self.device = device
        if hasattr(self, "_period_disp"):
            delattr(self, "_period_disp")
        return self

    def nearest_clip(self, vel_cmd: torch.Tensor) -> torch.Tensor:
        diff = self.velocities.unsqueeze(0) - vel_cmd.unsqueeze(1)
        dists = torch.sum(diff ** 2, dim=-1)
        return torch.argmin(dists, dim=-1)

    def query(self, clip_idx: torch.Tensor, phase: torch.Tensor):
        coeffs = self.coeff_mats[clip_idx]
        n_c = coeffs.shape[-1]
        k = torch.arange(n_c, device=self.device, dtype=torch.float32)
        t_powers = phase.unsqueeze(-1) ** k
        values = torch.sum(coeffs * t_powers.unsqueeze(1), dim=-1)

        root_pos  = values[:, ROOT_POS_SLICE]
        root_quat = values[:, ROOT_QUAT_SLICE]   # (B, 4) xyzw scipy
        jpos_poly = values[:, JPOS_SLICE]
        jvel_poly = values[:, JVEL_SLICE]
        foot      = values[:, FOOT_SLICE]

        jpos = jpos_poly[:, self._poly_from_mjlab]
        jvel = jvel_poly[:, self._poly_from_mjlab]
        return root_pos, root_quat, jpos, jvel, foot

    def period(self, clip_idx: torch.Tensor) -> torch.Tensor:
        return self.periods[clip_idx]

    @property
    def period_displacement(self) -> torch.Tensor:
        if not hasattr(self, "_period_disp"):
            n = self.n_clips
            idx_all = torch.arange(n, device=self.device)
            phase0 = torch.zeros(n, device=self.device)
            phase1 = torch.ones(n, device=self.device) * 0.9999
            pos0, _, _, _, _ = self.query(idx_all, phase0)
            pos1, _, _, _, _ = self.query(idx_all, phase1)
            self._period_disp = (pos1[:, :2] - pos0[:, :2])
        return self._period_disp


_LIBRARY: ReferenceMotionLibraryLegs | None = None


def get_library_legs(pkl_path: str | Path, device: str) -> ReferenceMotionLibraryLegs:
    global _LIBRARY
    if _LIBRARY is None:
        _LIBRARY = ReferenceMotionLibraryLegs(pkl_path, device)
    elif _LIBRARY.device != device:
        _LIBRARY.to(device)
    return _LIBRARY
