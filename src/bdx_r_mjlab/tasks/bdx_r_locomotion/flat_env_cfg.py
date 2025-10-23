from dataclasses import dataclass
import math
from bdx_r_mjlab.tasks.bdx_r_locomotion.rough_env_cfg import (
    BdxRRoughEnvCfg,
)


@dataclass
class BdxRFlatEnvCfg(BdxRRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        assert self.scene.terrain is not None
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.curriculum.terrain_levels = None

        self.curriculum.command_vel = None

        assert self.events.push_robot is not None
        self.events.push_robot.params["velocity_range"] = {
            "x": (-0.5, 0.5),
            "y": (-0.5, 0.5),
        }


@dataclass
class BdxRFlatEnvCfg_PLAY(BdxRFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Effectively infinite episode length.
        self.episode_length_s = int(1e9)
        assert self.events.push_robot is not None
        self.events.push_robot = None
        assert self.events.foot_friction is not None
        self.events.foot_friction = None

        self.commands.twist.ranges.lin_vel_x = (0.35, 0.35)
        self.commands.twist.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.twist.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.twist.ranges.heading = (0.0, 0.0)

        self.events.reset_base.params["pose_range"]["x"] = (0.0, 0.0)
        self.events.reset_base.params["pose_range"]["y"] = (0.0, 0.0)
        self.events.reset_base.params["pose_range"]["yaw"] = (
            1.25 * math.pi,
            1.25 * math.pi,
        )
