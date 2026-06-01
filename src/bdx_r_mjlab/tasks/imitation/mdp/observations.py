"""Observation terms for imitation learning (read from ImitationCommand)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv

# Slices match ImitationCommand's CMD layout
_PHASE_SLICE = slice(0, 2)
_JPOS_SLICE  = slice(2, 16)


def imitation_phase(env: ManagerBasedRlEnv, command_name: str = "imitation") -> torch.Tensor:
    """Gait phase as (sin, cos). Shape (B, 2)."""
    return env.command_manager.get_command(command_name)[:, _PHASE_SLICE]


def ref_joint_pos(env: ManagerBasedRlEnv, command_name: str = "imitation") -> torch.Tensor:
    """Reference joint positions in MjLab joint order. Shape (B, 14)."""
    return env.command_manager.get_command(command_name)[:, _JPOS_SLICE]
