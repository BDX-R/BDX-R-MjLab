"""BDXR constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import (
  ElectricActuator,
  reflected_inertia_from_two_stage_planetary,
)
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

BDXR_ROOT = Path(__file__).parent 

BDXR_XML: Path = BDXR_ROOT / "xmls" / "bdxr.xml"

assert BDXR_XML.exists(), f"XML not found at {BDXR_XML}"

def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, BDXR_XML.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(BDXR_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Motor specs (from Booster T1).
ARMATURE_ROBSTRIDE_03 = 0.02
ARMATURE_ROBSTRIDE_02 = 0.0042
ARMATURE_ROBSTRIDE_05 = 0.0007

ACTUATOR_ROBSTRIDE_03 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBSTRIDE_03,
  velocity_limit=18.849,
  effort_limit=42.0,
)
ACTUATOR_ROBSTRIDE_02 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBSTRIDE_02,
  velocity_limit=37.699,
  effort_limit=10.9,
)
ACTUATOR_ROBSTRIDE_05 = ElectricActuator(
  reflected_inertia=ARMATURE_ROBSTRIDE_02,
  velocity_limit=45,
  effort_limit=4.2,
)


NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_ROBSTRIDE_03 = ARMATURE_ROBSTRIDE_03 * NATURAL_FREQ**2
STIFFNESS_ROBSTRIDE_02 = ARMATURE_ROBSTRIDE_02 * NATURAL_FREQ**2
STIFFNESS_ROBSTRIDE_05 = ARMATURE_ROBSTRIDE_05 * NATURAL_FREQ**2

DAMPING_ROBSTRIDE_03 = 2.0 * DAMPING_RATIO * ARMATURE_ROBSTRIDE_03 * NATURAL_FREQ
DAMPING_ROBSTRIDE_02 = 2.0 * DAMPING_RATIO * ARMATURE_ROBSTRIDE_02 * NATURAL_FREQ
DAMPING_ROBSTRIDE_05 = 2.0 * DAMPING_RATIO * ARMATURE_ROBSTRIDE_05 * NATURAL_FREQ


BDXR_ACTUATOR_ROBSTRIDE_03 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_Hip_Yaw", ".*_Hip_Roll", ".*_Hip_Pitch", ".*_Knee",),
  stiffness=STIFFNESS_ROBSTRIDE_03,
  damping=DAMPING_ROBSTRIDE_03, 
  effort_limit=ACTUATOR_ROBSTRIDE_03.effort_limit,
  armature=ACTUATOR_ROBSTRIDE_03.reflected_inertia,
)
BDXR_ACTUATOR_ROBSTRIDE_02 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_Ankle",".*Neck_Pitch",),
  stiffness=STIFFNESS_ROBSTRIDE_02,
  damping=DAMPING_ROBSTRIDE_02,
  effort_limit=ACTUATOR_ROBSTRIDE_02.effort_limit,
  armature=ACTUATOR_ROBSTRIDE_02.reflected_inertia,
)
BDXR_ACTUATOR_ROBSTRIDE_05 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*Head_Yaw",".*Head_Pitch",".*Head_Roll",),
  stiffness=STIFFNESS_ROBSTRIDE_05,
  damping=DAMPING_ROBSTRIDE_05,
  effort_limit=ACTUATOR_ROBSTRIDE_05.effort_limit,
  armature=ACTUATOR_ROBSTRIDE_05.reflected_inertia,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.33),
  joint_pos={
    ".*Neck_Pitch": 0.0,
    ".*Head_Yaw": 0.0,
    ".*Head_Pitch": 0.0,
    ".*Head_Roll": 0.0,
    ".*_Hip_Yaw": 0.0,
    ".*_Hip_Pitch": 0.0,
    ".*_Hip_Roll": 0.0,
    ".*_Knee": 0.0,
    ".*_Ankle": 0.0,
  },
  joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.33),
  joint_pos={
    ".*Neck_Pitch": 0.0,
    ".*Head_Yaw": 0.0,
    ".*Head_Pitch": 0.0,
    ".*Head_Roll": 0.0,
    ".*_Hip_Yaw": 0.0,
    ".*_Hip_Yaw": 0.0,
    ".*_Hip_Pitch": 0.0,
    ".*_Hip_Roll": 0.0,
    ".*_Knee": 0.0,
    ".*_Ankle": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-7]_collision$": 1},
  friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot[1-7]_collision$": 1},
  friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

# This disables all collisions except the feet.
# Feet get condim=3, all other geoms are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(left|right)_foot[1-7]_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
)

##
# Final config.
##

BDXR_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    BDXR_ACTUATOR_ROBSTRIDE_03,
    BDXR_ACTUATOR_ROBSTRIDE_02,
    BDXR_ACTUATOR_ROBSTRIDE_05,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_bdxr_robot_cfg() -> EntityCfg:
  """Get a fresh T1 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=BDXR_ARTICULATION,
  )


BDXR_ACTION_SCALE: dict[str, float] = {}
for a in BDXR_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    BDXR_ACTION_SCALE[n] = 0.25 * e / s


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_bdxr_robot_cfg())

  viewer.launch(robot.spec.compile())
