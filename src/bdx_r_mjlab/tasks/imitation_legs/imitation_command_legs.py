"""ImitationCommandLegs: same as ImitationCommand but for legs-only BDX-R (10 joints).

CMD layout: [sin(phase), cos(phase), ref_jpos×10, ref_jvel×10, foot_contacts×2] = 24 dims
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
import torch

from mjlab.managers import CommandTerm, CommandTermCfg
from bdx_r_mjlab.tasks.imitation_legs.reference_motion_legs import (
    ReferenceMotionLibraryLegs,
    get_library_legs,
)

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.viewer import DebugVisualizer

CMD_PHASE_SLICE = slice(0, 2)
CMD_JPOS_SLICE  = slice(2, 12)
CMD_JVEL_SLICE  = slice(12, 22)
CMD_FOOT_SLICE  = slice(22, 24)
CMD_DIM         = 24


class ImitationCommandLegs(CommandTerm):
    cfg: "ImitationCommandLegsCfg"

    def __init__(self, cfg: "ImitationCommandLegsCfg", env: ManagerBasedRlEnv) -> None:
        super().__init__(cfg, env)
        self._lib: ReferenceMotionLibraryLegs = get_library_legs(cfg.pkl_path, env.device)

        B = env.num_envs
        self._clip_idx = torch.zeros(B, dtype=torch.long, device=env.device)

        self._ghost_abs_time = torch.zeros(B, device=env.device)

        self._spawn_pos     = torch.zeros(B, 3, device=env.device)
        self._spawn_yaw_cos = torch.ones(B, device=env.device)
        self._spawn_yaw_sin = torch.zeros(B, device=env.device)
        self._spawn_qyaw_z  = torch.zeros(B, device=env.device)
        self._spawn_qyaw_w  = torch.ones(B, device=env.device)

        self._cmd             = torch.zeros(B, CMD_DIM, device=env.device)
        self._ghost_root_pos  = torch.zeros(B, 3, device=env.device)
        self._ghost_root_quat = torch.zeros(B, 4, device=env.device)
        self._ghost_model     = None

    @property
    def command(self) -> torch.Tensor:
        return self._cmd

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        vel_cmd = self._env.command_manager.get_command("twist")[env_ids, :3]
        self._clip_idx[env_ids] = self._lib.nearest_clip(vel_cmd)

        entity = self._env.scene[self.cfg.entity_name]
        free_q = entity.indexing.free_joint_q_adr
        pose7  = entity.data.data.qpos[env_ids][:, free_q]
        pos_w     = pose7[:, :3]
        quat_wxyz = pose7[:, 3:7]

        self._spawn_pos[env_ids] = pos_w

        w, x, y, z = quat_wxyz[:, 0], quat_wxyz[:, 1], quat_wxyz[:, 2], quat_wxyz[:, 3]
        yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self._spawn_yaw_cos[env_ids] = torch.cos(yaw)
        self._spawn_yaw_sin[env_ids] = torch.sin(yaw)
        self._spawn_qyaw_z[env_ids]  = torch.sin(yaw / 2.0)
        self._spawn_qyaw_w[env_ids]  = torch.cos(yaw / 2.0)

    def _update_command(self) -> None:
        self._ghost_abs_time += self._env.step_dt

        period    = self._lib.period(self._clip_idx)
        raw_phase = self._ghost_abs_time / period
        n_periods = raw_phase.floor()
        phase     = raw_phase % 1.0

        root_pos, root_quat, jpos, jvel, foot = self._lib.query(self._clip_idx, phase)

        per_period_disp = self._lib.period_displacement[self._clip_idx]
        twist   = self._env.command_manager.get_command("twist")
        vtheta  = twist[:, 2]
        yaw_per_period = vtheta * period
        accum_yaw      = n_periods * yaw_per_period

        Δx = per_period_disp[:, 0]
        Δy = per_period_disp[:, 1]
        θ  = yaw_per_period
        n  = n_periods
        denom_sq    = 2.0 * (1.0 - torch.cos(θ))
        is_straight = denom_sq < 1e-8
        num_r = torch.cos(n * θ) - 1.0
        num_i = torch.sin(n * θ)
        den_r = torch.cos(θ) - 1.0
        den_i = torch.sin(θ)
        safe_d = torch.where(is_straight, torch.ones_like(denom_sq), denom_sq)
        G_r = torch.where(is_straight, n, (num_r*den_r + num_i*den_i) / safe_d)
        G_i = torch.where(is_straight, torch.zeros_like(n), (num_i*den_r - num_r*den_i) / safe_d)
        accum_x = G_r * Δx - G_i * Δy
        accum_y = G_i * Δx + G_r * Δy

        cos_acc = torch.cos(accum_yaw)
        sin_acc = torch.sin(accum_yaw)
        placo_x = cos_acc * root_pos[:, 0] - sin_acc * root_pos[:, 1] + accum_x
        placo_y = sin_acc * root_pos[:, 0] + cos_acc * root_pos[:, 1] + accum_y
        placo_z = root_pos[:, 2]

        cos_y = self._spawn_yaw_cos
        sin_y = self._spawn_yaw_sin
        self._ghost_root_pos[:, 0] = self._spawn_pos[:, 0] + cos_y * placo_x - sin_y * placo_y
        self._ghost_root_pos[:, 1] = self._spawn_pos[:, 1] + sin_y * placo_x + cos_y * placo_y
        self._ghost_root_pos[:, 2] = placo_z

        rx, ry, rz, rw = root_quat[:, 0], root_quat[:, 1], root_quat[:, 2], root_quat[:, 3]
        vtheta_clip = self._lib.velocities[self._clip_idx, 2]
        dc_yaw      = phase * vtheta_clip * period
        nz = -torch.sin(dc_yaw / 2.0)
        nw =  torch.cos(dc_yaw / 2.0)
        pnx = nw*rx - nz*ry
        pny = nw*ry + nz*rx
        pnz = nw*rz + nz*rw
        pnw = nw*rw - nz*rz

        az = torch.sin(accum_yaw / 2.0)
        aw = torch.cos(accum_yaw / 2.0)
        qz = self._spawn_qyaw_z
        qw = self._spawn_qyaw_w
        total_z = qz*aw + qw*az
        total_w = qw*aw - qz*az
        out_x = total_w*pnx - total_z*pny
        out_y = total_w*pny + total_z*pnx
        out_z = total_w*pnz + total_z*pnw
        out_w = total_w*pnw - total_z*pnz
        self._ghost_root_quat[:] = torch.stack([out_x, out_y, out_z, out_w], dim=-1)

        angle = phase * 2.0 * torch.pi
        self._cmd[:, CMD_PHASE_SLICE] = torch.stack([torch.sin(angle), torch.cos(angle)], dim=-1)
        self._cmd[:, CMD_JPOS_SLICE]  = jpos
        self._cmd[:, CMD_JVEL_SLICE]  = jvel
        self._cmd[:, CMD_FOOT_SLICE]  = foot

    def _update_metrics(self) -> None:
        pass

    def _debug_vis_impl(self, visualizer: DebugVisualizer) -> None:
        env_indices = visualizer.get_env_indices(self.num_envs)
        if not env_indices:
            return

        if self._ghost_model is None:
            self._ghost_model = copy.deepcopy(self._env.sim.mj_model)
            self._ghost_model.geom_rgba[:] = self.cfg.ghost_color

        entity   = self._env.scene[self.cfg.entity_name]
        indexing = entity.indexing
        free_q   = indexing.free_joint_q_adr.cpu().numpy()
        joint_q  = indexing.joint_q_adr.cpu().numpy()
        ref_jpos = self._cmd[:, CMD_JPOS_SLICE].cpu().numpy()

        ghost_pos_np    = self._ghost_root_pos.cpu().numpy()
        ghost_quat_xyzw = self._ghost_root_quat.cpu().numpy()
        ghost_quat_wxyz = ghost_quat_xyzw[:, [3, 0, 1, 2]]

        for batch in env_indices:
            qpos = np.zeros(self._env.sim.mj_model.nq)
            qpos[free_q[0:3]] = ghost_pos_np[batch]
            qpos[free_q[3:7]] = ghost_quat_wxyz[batch]
            qpos[joint_q]     = ref_jpos[batch]
            visualizer.add_ghost_mesh(
                qpos, model=self._ghost_model, label=f"ref_ghost_{batch}"
            )


@dataclass
class ImitationCommandLegsCfg(CommandTermCfg):
    pkl_path: str = ""
    entity_name: str = "robot"
    ghost_color: tuple[float, float, float, float] = (0.0, 0.9, 0.2, 0.4)
    resampling_time_range: tuple[float, float] = field(default_factory=lambda: (1e6, 1e6))
    debug_vis: bool = True

    def build(self, env: ManagerBasedRlEnv) -> ImitationCommandLegs:
        return ImitationCommandLegs(self, env)
