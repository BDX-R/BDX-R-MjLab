"""Legs-only imitation environment for BDX-R.

Extends bdxr_flat_env_legs_cfg with:
  - ImitationCommandLegs term (10-joint reference, green ghost)
  - phase + ref_joint_pos observations (10 joints)
  - imitate_joint_pos / imitate_joint_vel / imitate_foot_contact rewards

"""

from pathlib import Path

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg

from bdx_r_mjlab.tasks.velocity.config.bdxr.legs_env_cfgs import bdxr_flat_env_legs_cfg
from bdx_r_mjlab.tasks.imitation_legs import mdp
from bdx_r_mjlab.tasks.imitation_legs.imitation_command_legs import ImitationCommandLegsCfg

_DEFAULT_PKL = str(Path(__file__).parents[5] / "data" / "polynomial_coefficients_legs.pkl")


def bdxr_imitation_legs_env_cfg(
    pkl_path: str = _DEFAULT_PKL,
    play: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Create legs-only imitation env for BDX-R."""
    cfg = bdxr_flat_env_legs_cfg(play=play)

    cfg.commands["imitation"] = ImitationCommandLegsCfg(pkl_path=pkl_path)

    imitation_obs = {
        "imitation_phase": ObservationTermCfg(
            func=mdp.imitation_phase,
            params={"command_name": "imitation"},
        ),
        "ref_joint_pos": ObservationTermCfg(
            func=mdp.ref_joint_pos,
            params={"command_name": "imitation"},
        ),
    }
    cfg.observations["actor"].terms.update(imitation_obs)
    cfg.observations["critic"].terms.update(imitation_obs)

    cfg.rewards["imitate_joint_pos"] = RewardTermCfg(
        func=mdp.imitate_joint_pos,
        weight=1.5,
        params={"std": 0.25, "command_name": "imitation"},
    )
    cfg.rewards["imitate_joint_vel"] = RewardTermCfg(
        func=mdp.imitate_joint_vel,
        weight=0.1,
        params={"std": 2.0, "command_name": "imitation"},
    )
    cfg.rewards["imitate_foot_contact"] = RewardTermCfg(
        func=mdp.imitate_foot_contact,
        weight=0.5,
        params={"sensor_name": "feet_ground_contact", "command_name": "imitation"},
    )

    return cfg
