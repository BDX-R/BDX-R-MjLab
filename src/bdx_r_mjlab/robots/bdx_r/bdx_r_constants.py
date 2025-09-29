"""BDX-R constants."""

from pathlib import Path

import mujoco

from bdx_r_mjlab import BDX_R_MJLAB_SRC_PATH
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import ActuatorCfg, CollisionCfg

##
# MJCF and assets.
##

BDX_R_XML: Path = (
  BDX_R_MJLAB_SRC_PATH / "robots" / "bdx_r" / "BDX-R-CAD-main" / "BDX-R_description" / "BDX-R.xml"
)
assert BDX_R_XML.exists()


def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, BDX_R_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(BDX_R_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Motor specs.
ARMATURE_LEG = 0.02
ARMATURE_ANKLE = 0.042

EFFORT_LIMIT_LEG = 42.0
EFFORT_LIMIT_ANKLE = 11.9

# VELOCITY_LIMIT_LEG = 18.849
# VELOCITY_LIMIT_ANKLE = 37.699

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_LEG = ARMATURE_LEG * NATURAL_FREQ**2
STIFFNESS_ANKLE = ARMATURE_ANKLE * NATURAL_FREQ**2

DAMPING_LEG = 2.0 * DAMPING_RATIO * ARMATURE_LEG * NATURAL_FREQ
DAMPING_ANKLE = 2.0 * DAMPING_RATIO * ARMATURE_ANKLE * NATURAL_FREQ

# Actuators configs.
BDX_R_ACTUATOR_LEG = ActuatorCfg(
  joint_names_expr=[
    ".*_Hip_Yaw",
    ".*_Hip_Roll",
    ".*_Hip_Pitch",
    ".*_Knee",
  ],
  effort_limit=EFFORT_LIMIT_LEG,
  armature=ARMATURE_LEG,
  stiffness=STIFFNESS_LEG,
  damping=DAMPING_LEG,
)

BDX_R_ACTUATOR_ANKLE = ActuatorCfg(
  joint_names_expr=[
    ".*_Ankle",
  ],
  effort_limit=EFFORT_LIMIT_ANKLE,
  armature=ARMATURE_ANKLE,
  stiffness=STIFFNESS_ANKLE,
  damping=DAMPING_ANKLE,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.30846),
  joint_pos={".*": 0.0},
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.30846),
  joint_pos={".*": 0.0},
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3 and custom friction and solimp.

# TODO Louis

FULL_COLLISION = CollisionCfg(
  geom_names_expr=[".*_collision"],
  condim={r"^(left|right)_foot_collision$": 3},
  priority={r"^(left|right)_foot_collision$": 1},
  friction={r"^(left|right)_foot_collision$": (0.6,)},
)

# FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
#   geom_names_expr=[".*_collision"],
#   contype=0,
#   conaffinity=1,
#   condim={r"^(left|right)_foot[1-7]_collision$": 3},
#   priority={r"^(left|right)_foot[1-7]_collision$": 1},
#   friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
# )

# FEET_ONLY_COLLISION = CollisionCfg(
#   geom_names_expr=[r"^(left|right)_foot[1-7]_collision$"],
#   contype=0,
#   conaffinity=1,
#   condim=3,
#   priority=1,
#   friction=(0.6,),
# )

##
# Final config.
##

BDX_R_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    BDX_R_ACTUATOR_LEG,
    BDX_R_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,
)

BDX_R_ROBOT_CFG = EntityCfg(
  init_state=KNEES_BENT_KEYFRAME,
  collisions=(FULL_COLLISION,),
  spec_fn=get_spec,
  articulation=BDX_R_ARTICULATION,
)

BDX_R_ACTION_SCALE: dict[str, float] = {}
for a in BDX_R_ARTICULATION.actuators:
  e = a.effort_limit
  s = a.stiffness
  names = a.joint_names_expr
  if not isinstance(e, dict):
    e = {n: e for n in names}
  if not isinstance(s, dict):
    s = {n: s for n in names}
  for n in names:
    if n in e and n in s and s[n]:
      BDX_R_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(BDX_R_ROBOT_CFG)

  viewer.launch(robot.spec.compile())