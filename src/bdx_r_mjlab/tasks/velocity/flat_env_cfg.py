from dataclasses import dataclass

from bdx_r_mjlab.tasks.velocity.rough_env_cfg import (
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


@dataclass
class BdxRFlatEnvCfg_PLAY(BdxRFlatEnvCfg):
  def __post_init__(self):
    super().__post_init__()

    # Effectively infinite episode length.
    self.episode_length_s = int(1e9)