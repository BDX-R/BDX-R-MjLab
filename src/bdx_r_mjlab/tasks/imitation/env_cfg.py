"""Imitation + velocity tracking environment for BDX-R.

Extends bdxr_flat_env_cfg with:
  - ImitationCommand term (manages phase, reference clips, green ghost)
  - phase + ref_joint_pos observations
  - imitate_joint_pos / imitate_joint_vel / imitate_foot_contact rewards
"""

from pathlib import Path

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg

from bdx_r_mjlab.tasks.velocity.config.bdxr.env_cfgs import bdxr_flat_env_cfg
from bdx_r_mjlab.tasks.imitation import mdp
from bdx_r_mjlab.tasks.imitation.imitation_command import ImitationCommandCfg

_DEFAULT_PKL = str(Path(__file__).parents[4] / "data" / "polynomial_coefficients.pkl")


def bdxr_imitation_env_cfg(
    pkl_path: str = _DEFAULT_PKL,
    play: bool = False,
) -> ManagerBasedRlEnvCfg:
    """Create imitation + velocity tracking env for BDX-R."""
    cfg = bdxr_flat_env_cfg(play=play)

    # ── imitation command term (phase + reference + ghost) ────────────────────
    cfg.commands["imitation"] = ImitationCommandCfg(pkl_path=pkl_path)

    # ── observations ──────────────────────────────────────────────────────────
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

    # ── rewards ───────────────────────────────────────────────────────────────
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
