"""RS02 and RS03 single-motor sinusoidal trajectory tracking tasks."""

from __future__ import annotations

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.utils.os import update_assets

from . import mdp

_REPO_ROOT = Path(__file__).parents[4]   # …/BDX-R-MjLab/

# ── RS02 (O2) chirp-ID tuned values ──────────────────────────────────────────
_RS02_XML      = _REPO_ROOT / "experiments/chirp_id/rs02_motor.xml"
_RS02_MESH     = _REPO_ROOT / "src/RS02_description/meshes"
_RS02_KP       = 17.0
_RS02_KD       = 1.0
_RS02_ARMATURE = 0.0142
_RS02_FRICTION = 0.0
_RS02_DELAY    = 3
_RS02_EFFORT   = 17.0

# ── RS03 (O3) chirp-ID tuned values — fill in after running chirp ID ─────────
_RS03_XML      = _REPO_ROOT / "experiments/chirp_id/rs03_motor.xml"
_RS03_MESH     = _REPO_ROOT / "src/RS03_description/meshes"
_RS03_KP       = 78.957
_RS03_KD       = 5.027
_RS03_ARMATURE = 0.06
_RS03_FRICTION = 0.0
_RS03_DELAY    = 3
_RS03_EFFORT   = 60.0

_RS02_ASSET_CFG = SceneEntityCfg("rs02")
_RS03_ASSET_CFG = SceneEntityCfg("rs03")


def _get_rs02_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(_RS02_XML))
    assets: dict[str, bytes] = {}
    update_assets(assets, _RS02_MESH, spec.meshdir)
    spec.assets = assets
    return spec


def _get_rs03_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(_RS03_XML))
    assets: dict[str, bytes] = {}
    update_assets(assets, _RS03_MESH, spec.meshdir)
    spec.assets = assets
    return spec


def _make_entity(kp, kd, effort, armature, friction, delay, spec_fn) -> EntityCfg:
    actuator = BuiltinPositionActuatorCfg(
        target_names_expr=("motor_joint",),
        stiffness=kp,
        damping=kd,
        effort_limit=effort,
        armature=armature,
        frictionloss=friction,
        delay_min_lag=delay,
        delay_max_lag=delay,
        delay_per_env_phase=False,
    )
    return EntityCfg(
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={"motor_joint": 0.0},
            joint_vel={"motor_joint": 0.0},
        ),
        spec_fn=spec_fn,
        articulation=EntityArticulationInfoCfg(actuators=(actuator,)),
    )


def _make_rs02_entity() -> EntityCfg:
    return _make_entity(
        _RS02_KP, _RS02_KD, _RS02_EFFORT, _RS02_ARMATURE, _RS02_FRICTION,
        _RS02_DELAY, _get_rs02_spec,
    )


def _make_rs03_entity() -> EntityCfg:
    return _make_entity(
        _RS03_KP, _RS03_KD, _RS03_EFFORT, _RS03_ARMATURE, _RS03_FRICTION,
        _RS03_DELAY, _get_rs03_spec,
    )


def _motor_tracking_env_cfg(
    entity_name: str,
    entity_fn,
    asset_cfg: SceneEntityCfg,
    play: bool,
) -> ManagerBasedRlEnvCfg:
    num_envs = 1 if play else 4096

    actor_obs_terms = {
        "joint_pos": ObservationTermCfg(
            func=mdp.joint_pos,
            params={"asset_cfg": asset_cfg},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        ),
        "joint_vel": ObservationTermCfg(
            func=mdp.joint_vel,
            params={"asset_cfg": asset_cfg},
            noise=Unoise(n_min=-1.5, n_max=1.5),
        ),
        "target_pos": ObservationTermCfg(
            func=mdp.target_pos,
            params={"command_name": "sinusoid", "asset_cfg": asset_cfg},
        ),
        "last_action": ObservationTermCfg(func=envs_mdp.last_action),
    }
    critic_obs_terms = {
        **actor_obs_terms,
        "pos_error": ObservationTermCfg(
            func=mdp.pos_error,
            params={"command_name": "sinusoid", "asset_cfg": asset_cfg},
        ),
    }

    return ManagerBasedRlEnvCfg(
        decimation=4,
        episode_length_s=10.0,
        sim=SimulationCfg(
            mujoco=MujocoCfg(
                timestep=0.005,
                integrator="implicitfast",
                gravity=(0.0, 0.0, -9.81),
            )
        ),
        scene=SceneCfg(
            num_envs=num_envs,
            entities={entity_name: entity_fn()},
        ),
        actions={
            "joint_pos": JointPositionActionCfg(
                entity_name=entity_name,
                actuator_names=("motor_joint",),
                scale=1.0,
                offset=0.0,
                use_default_offset=False,
            )
        },
        observations={
            "actor": ObservationGroupCfg(actor_obs_terms),
            "critic": ObservationGroupCfg(critic_obs_terms),
        },
        commands={
            "sinusoid": mdp.SinusoidalCommandCfg(
                amp_range=(0.2, 1.0),
                freq_range=(0.5, 3.0),
                resampling_time_range=(12.0, 12.0),
            ),
        },
        rewards={
            "tracking_pos": RewardTermCfg(
                func=mdp.tracking_position,
                weight=1.0,
                params={"command_name": "sinusoid", "asset_cfg": asset_cfg},
            ),
            "action_rate": RewardTermCfg(
                func=envs_mdp.action_rate_l2,
                weight=-0.01,
            ),
        },
        terminations={
            "time_out": TerminationTermCfg(
                func=envs_mdp.time_out,
                time_out=True,
            ),
        },
        events={
            "reset_scene_to_default": EventTermCfg(
                func=envs_mdp.reset_scene_to_default,
                mode="reset",
            ),
            "randomize_pd_gains": EventTermCfg(
                func=envs_mdp.randomize_pd_gains,
                mode="reset",
                params={
                    "asset_cfg": SceneEntityCfg(entity_name, actuator_ids=slice(None)),
                    "kp_range": (0.85, 1.15),
                    "kd_range": (0.85, 1.15),
                },
            ),
        },
    )


def rs02_motor_tracking_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    return _motor_tracking_env_cfg("rs02", _make_rs02_entity, _RS02_ASSET_CFG, play)


def rs03_motor_tracking_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    return _motor_tracking_env_cfg("rs03", _make_rs03_entity, _RS03_ASSET_CFG, play)
