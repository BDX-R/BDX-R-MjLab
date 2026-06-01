"""Reference motion library backed by polynomial_coefficients.pkl.

Loads the polynomial fit produced by fit_poly.py and provides fast
GPU-batched queries: given velocity commands + gait phase → reference
joint positions, velocities and foot contacts.
"""

import pickle
from pathlib import Path

import numpy as np
import torch

# ── joint order remapping ──────────────────────────────────────────────────────
# Polynomial order  (from placo_defaults.json):
#   0 Left_Hip_Yaw  1 Left_Hip_Roll  2 Left_Hip_Pitch  3 left_knee  4 Left_Ankle
#   5 Neck_Pitch  6 Head_Pitch  7 Head_Yaw  8 Head_Roll
#   9 Right_Hip_Yaw 10 Right_Hip_Roll 11 Right_Hip_Pitch 12 right_knee 13 Right_Ankle
#
# MjLab actuator order (from BDX-R XML):
#   0 Left_Hip_Yaw  1 Left_Hip_Roll  2 Left_Hip_Pitch  3 Left_Knee  4 Left_Ankle
#   5 Right_Hip_Yaw 6 Right_Hip_Roll 7 Right_Hip_Pitch 8 Right_Knee  9 Right_Ankle
#   10 Neck_Pitch  11 Head_Pitch  12 Head_Yaw  13 Head_Roll
#
# POLY_FROM_MJLAB[j] = index in the polynomial array that corresponds to mjlab joint j
POLY_FROM_MJLAB = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13, 5, 6, 7, 8]
N_JOINTS = 14
# Layout: root_pos(3) + root_quat(4) + jpos(14) + jvel(14) + foot(2) + lin_vel(3) + ang_vel(3) = 43
ROOT_POS_SLICE  = slice(0, 3)
ROOT_QUAT_SLICE = slice(3, 7)
JPOS_SLICE      = slice(7, 21)
JVEL_SLICE      = slice(21, 35)
FOOT_SLICE      = slice(35, 37)


class ReferenceMotionLibrary:
    """Loads polynomial reference motions and answers batched queries."""

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

    # ── device management ──────────────────────────────────────────────────────

    def to(self, device: str) -> "ReferenceMotionLibrary":
        self.velocities = self.velocities.to(device)
        self.coeff_mats = self.coeff_mats.to(device)
        self.periods = self.periods.to(device)
        self._poly_from_mjlab = self._poly_from_mjlab.to(device)
        self.device = device
        if hasattr(self, "_period_disp"):
            delattr(self, "_period_disp")
        return self

    # ── core API ───────────────────────────────────────────────────────────────

    def nearest_clip(self, vel_cmd: torch.Tensor) -> torch.Tensor:
        """Return clip index for each env.

        Args:
            vel_cmd: (B, 3) tensor of (vx, vy, vtheta) commands.
        Returns:
            (B,) long tensor of clip indices.
        """
        diff = self.velocities.unsqueeze(0) - vel_cmd.unsqueeze(1)  # (B, N, 3)
        dists = torch.sum(diff ** 2, dim=-1)                         # (B, N)
        return torch.argmin(dists, dim=-1)                           # (B,)

    def query(
        self,
        clip_idx: torch.Tensor,
        phase: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Evaluate polynomials at given phase.

        Args:
            clip_idx: (B,) clip indices.
            phase:    (B,) phase in [0, 1].
        Returns:
            joint_pos:     (B, 14) in MjLab joint order.
            joint_vel:     (B, 14) in MjLab joint order.
            foot_contacts: (B, 2)  [left, right] binary contacts.
        """
        coeffs = self.coeff_mats[clip_idx]               # (B, 36, 16)
        n_c = coeffs.shape[-1]
        k = torch.arange(n_c, device=self.device, dtype=torch.float32)
        t_powers = phase.unsqueeze(-1) ** k              # (B, 16)
        values = torch.sum(coeffs * t_powers.unsqueeze(1), dim=-1)  # (B, 36)

        root_pos  = values[:, ROOT_POS_SLICE]                            # (B, 3)
        root_quat = values[:, ROOT_QUAT_SLICE]                           # (B, 4) xyzw scipy
        jpos_poly = values[:, JPOS_SLICE]                                # (B, 14)
        jvel_poly = values[:, JVEL_SLICE]                                # (B, 14)
        foot      = values[:, FOOT_SLICE]                                # (B, 2)

        jpos = jpos_poly[:, self._poly_from_mjlab]
        jvel = jvel_poly[:, self._poly_from_mjlab]
        return root_pos, root_quat, jpos, jvel, foot

    def period(self, clip_idx: torch.Tensor) -> torch.Tensor:
        """Return gait period (s) for each env. (B,)"""
        return self.periods[clip_idx]

    @property
    def period_displacement(self) -> torch.Tensor:
        """XY displacement per gait period for each clip. Shape (N, 2).

        Computed as pos(phase≈1) - pos(phase=0) for each clip.
        This lets the ghost accumulate position across multiple periods.
        """
        if not hasattr(self, "_period_disp"):
            n = self.n_clips
            # Evaluate at phase 0 and phase 1-ε for all clips at once
            idx_all = torch.arange(n, device=self.device)
            phase0 = torch.zeros(n, device=self.device)
            phase1 = torch.ones(n, device=self.device) * 0.9999
            pos0, _, _, _, _ = self.query(idx_all, phase0)  # root_pos at phase=0 (should be ~0)
            pos1, _, _, _, _ = self.query(idx_all, phase1)  # root_pos at phase≈1
            self._period_disp = (pos1[:, :2] - pos0[:, :2])   # (N, 2)
        return self._period_disp



# ── singleton ──────────────────────────────────────────────────────────────────

_LIBRARY: ReferenceMotionLibrary | None = None


def get_library(pkl_path: str | Path, device: str) -> ReferenceMotionLibrary:
    """Return (or create) the global ReferenceMotionLibrary singleton."""
    global _LIBRARY
    if _LIBRARY is None:
        _LIBRARY = ReferenceMotionLibrary(pkl_path, device)
    elif _LIBRARY.device != device:
        _LIBRARY.to(device)
    return _LIBRARY
