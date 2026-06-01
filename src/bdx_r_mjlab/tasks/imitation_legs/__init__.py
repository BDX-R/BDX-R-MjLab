from bdx_r_mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfg import bdxr_imitation_legs_env_cfg
from .rl_cfg import bdxr_imitation_legs_ppo_runner_cfg
from . import mdp

register_mjlab_task(
    task_id="Mjlab-Imitation-Flat-BDX-R-Legs",
    env_cfg=bdxr_imitation_legs_env_cfg(),
    play_env_cfg=bdxr_imitation_legs_env_cfg(play=True),
    rl_cfg=bdxr_imitation_legs_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)
