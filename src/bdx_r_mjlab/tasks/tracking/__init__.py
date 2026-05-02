from bdx_r_mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner

from .env_cfg import bdxr_flat_tracking_env_cfg
from .full_env_cfg import bdxr_full_flat_tracking_env_cfg
from .rl_cfg import bdxr_tracking_ppo_runner_cfg

register_mjlab_task(
    task_id="Mjlab-Tracking-Flat-BDX-R",
    env_cfg=bdxr_flat_tracking_env_cfg(has_state_estimation=False),
    play_env_cfg=bdxr_flat_tracking_env_cfg(has_state_estimation=False, play=True),
    rl_cfg=bdxr_tracking_ppo_runner_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)

register_mjlab_task(
    task_id="Mjlab-Tracking-Flat-BDX-R-Full",
    env_cfg=bdxr_full_flat_tracking_env_cfg(has_state_estimation=False),
    play_env_cfg=bdxr_full_flat_tracking_env_cfg(has_state_estimation=False, play=True),
    rl_cfg=bdxr_tracking_ppo_runner_cfg(experiment_name="bdxr_full_tracking"),
    runner_cls=MotionTrackingOnPolicyRunner,
)
