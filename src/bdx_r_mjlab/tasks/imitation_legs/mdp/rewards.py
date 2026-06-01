"""Imitation reward terms for legs-only robot — read reference from ImitationLegsCommand."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
    from mjlab.entity import Entity
    from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")
# CMD layout: [sin/cos phase(2), jpos(10), jvel(10), foot(2)] = 24 dims
_JPOS_SLICE = slice(2, 12)
_JVEL_SLICE = slice(12, 22)
_FOOT_SLICE = slice(22, 24)


def imitate_joint_pos(
    env: ManagerBasedRlEnv,
    std: float = 0.25,
    command_name: str = "imitation",
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
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
    ref = env.command_manager.get_command(command_name)[:, _JVEL_SLICE]
    asset: Entity = env.scene[asset_cfg.name]
    err = torch.sum((ref - asset.data.joint_vel) ** 2, dim=-1)
    return torch.exp(-err / std ** 2)


def imitate_foot_contact(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    command_name: str = "imitation",
) -> torch.Tensor:
    from mjlab.sensor import ContactSensor
    ref_foot = env.command_manager.get_command(command_name)[:, _FOOT_SLICE]
    sensor: ContactSensor = env.scene[sensor_name]
    assert sensor.data.found is not None
    actual = (sensor.data.found > 0).float()
    return (ref_foot.round() == actual).float().mean(dim=-1)
