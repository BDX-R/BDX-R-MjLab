from bdx_r_mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner


from .env_cfgs import (
  bdxr_flat_env_cfg,
  bdxr_rough_env_cfg,
)

from .rl_cfg import bdxr_ppo_runner_cfg

register_mjlab_task(
    task_id="Mjlab-Velocity-Rough-BDX-R",
    env_cfg=bdxr_rough_env_cfg(),
    play_env_cfg=bdxr_rough_env_cfg(play=True),
    rl_cfg=bdxr_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
    task_id="Mjlab-Velocity-Rough-BDX-R-Play",
    env_cfg=bdxr_rough_env_cfg(),
    play_env_cfg=bdxr_rough_env_cfg(play=True),
    rl_cfg=bdxr_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
    task_id="Mjlab-Velocity-Flat-BDX-R",
    env_cfg=bdxr_flat_env_cfg(),
    play_env_cfg=bdxr_flat_env_cfg(play=True),
    rl_cfg=bdxr_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
    task_id="Mjlab-Velocity-Flat-BDX-R-Play",
    env_cfg=bdxr_flat_env_cfg(),
    play_env_cfg=bdxr_flat_env_cfg(play=True),
    rl_cfg=bdxr_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)
