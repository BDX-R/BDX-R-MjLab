"""Imitation reward terms — read reference state from ImitationCommand."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
    from mjlab.entity import Entity
    from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")
_JPOS_SLICE = slice(2, 16)
_JVEL_SLICE = slice(16, 30)
_FOOT_SLICE = slice(30, 32)


def imitate_joint_pos(
    env: ManagerBasedRlEnv,
    std: float = 0.25,
    command_name: str = "imitation",
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """exp(-||q_ref - q_actual||² / std²) on all joints."""
    ref = env.command_manager.get_command(command_name)[:, _JPOS_SLICE]
    asset: Entity = env.scene[asset_cfg.name]
    err = torch.sum((ref - asset.data.joint_pos) ** 2, dim=-1)
    return torch.exp(-err / std ** 2)


def imitate_joint_vel(
    env: ManagerBasedRlEnv,
    std: float = 2.0,
    command_name: str = "imitation",
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """exp(-||qd_ref - qd_actual||² / std²) on all joints."""
    ref = env.command_manager.get_command(command_name)[:, _JVEL_SLICE]
    asset: Entity = env.scene[asset_cfg.name]
    err = torch.sum((ref - asset.data.joint_vel) ** 2, dim=-1)
    return torch.exp(-err / std ** 2)


def imitate_foot_contact(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    command_name: str = "imitation",
) -> torch.Tensor:
    """Fraction of feet with matching contact state vs reference (0–1 per env)."""
    from mjlab.sensor import ContactSensor

    ref_foot = env.command_manager.get_command(command_name)[:, _FOOT_SLICE]  # (B, 2)
    sensor: ContactSensor = env.scene[sensor_name]
    assert sensor.data.found is not None
    actual = (sensor.data.found > 0).float()                                   # (B, 2)
    return (ref_foot.round() == actual).float().mean(dim=-1)                   # (B,)
