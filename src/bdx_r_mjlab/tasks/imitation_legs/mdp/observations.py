"""Observation terms for legs-only imitation learning."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv

# CMD layout: [sin/cos phase(2), jpos(10), jvel(10), foot(2)] = 24 dims
_PHASE_SLICE = slice(0, 2)
_JPOS_SLICE  = slice(2, 12)


def imitation_phase(env: ManagerBasedRlEnv, command_name: str = "imitation") -> torch.Tensor:
    """Gait phase as (sin, cos). Shape (B, 2)."""
    return env.command_manager.get_command(command_name)[:, _PHASE_SLICE]


def ref_joint_pos(env: ManagerBasedRlEnv, command_name: str = "imitation") -> torch.Tensor:
    """Reference joint positions in MjLab joint order. Shape (B, 10)."""
    return env.command_manager.get_command(command_name)[:, _JPOS_SLICE]
