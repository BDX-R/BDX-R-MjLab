"""BDXR constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg, DelayedActuatorCfg, IdealPdActuatorCfg
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

BDXR_XML_LEGS: Path = BDXR_ROOT / "xmls" / "bdxr_legs.xml"

assert BDXR_XML_LEGS.exists(), f"XML not found at {BDXR_XML_LEGS}"

def get_assets(meshdir: str) -> dict[str, bytes]:
  assets: dict[str, bytes] = {}
  update_assets(assets, BDXR_XML_LEGS.parent / "assets", meshdir)
  return assets


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(BDXR_XML_LEGS))
  spec.assets = get_assets(spec.meshdir)
  return spec


##
# Actuator config.
##

# Armature values from chirp system identification.
ARMATURE_ROBSTRIDE_03 = 0.06    # chirp ID tuned (was 0.02)
ARMATURE_ROBSTRIDE_02 = 0.0142  # chirp ID tuned (was 0.0042)

# KP/KD hardcoded to match real robot MIT control law.
# These are decoupled from armature — previously they were derived via
# KP = armature * (10*2π)², but that formula used the wrong armature values.
KP_ROBSTRIDE_03 = 78.957
KD_ROBSTRIDE_03 = 5.027
KP_ROBSTRIDE_02 = 16.581
KD_ROBSTRIDE_02 = 1.056

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


BDXR_ACTUATOR_ROBSTRIDE_03 = DelayedActuatorCfg(
    base_cfg=BuiltinPositionActuatorCfg(
        target_names_expr=(".*_Hip_Yaw", ".*_Hip_Roll", ".*_Hip_Pitch", ".*_Knee"),
        stiffness=KP_ROBSTRIDE_03,
        damping=KD_ROBSTRIDE_03,
        effort_limit=ACTUATOR_ROBSTRIDE_03.effort_limit,
        armature=ACTUATOR_ROBSTRIDE_03.reflected_inertia,
    ),
    delay_target="position",
    delay_min_lag=3,
    delay_max_lag=3,
)
BDXR_ACTUATOR_ROBSTRIDE_02 = DelayedActuatorCfg(
    base_cfg=BuiltinPositionActuatorCfg(
        target_names_expr=(".*_Ankle",),
        stiffness=KP_ROBSTRIDE_02,
        damping=KD_ROBSTRIDE_02,
        effort_limit=ACTUATOR_ROBSTRIDE_02.effort_limit,
        armature=ACTUATOR_ROBSTRIDE_02.reflected_inertia,
    ),
    delay_target="position",
    delay_min_lag=3,
    delay_max_lag=3,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.33),
  joint_pos={
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
  condim={r"^(left|right)_foot_collision[1-7]$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision[1-7]$": 1},
  friction={r"^(left|right)_foot_collision[1-7]$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=0,
  conaffinity=1,
  condim={r"^(left|right)_foot_collision[1-7]$": 3, ".*_collision": 1},
  priority={r"^(left|right)_foot_collision[1-7]$": 1},
  friction={r"^(left|right)_foot_collision[1-7]$": (0.6,)},
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

BDXR_LEGS_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    BDXR_ACTUATOR_ROBSTRIDE_03,
    BDXR_ACTUATOR_ROBSTRIDE_02,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_bdxr_robot_legs_cfg() -> EntityCfg:
  """Get a fresh T1 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=BDXR_LEGS_ARTICULATION,
  )


BDXR_ACTION_SCALE_LEGS: dict[str, float] = {}

for a in BDXR_LEGS_ARTICULATION.actuators:
    assert isinstance(a, DelayedActuatorCfg)

    base = a.base_cfg

    e = base.effort_limit
    s = base.stiffness
    names = base.target_names_expr

    assert e is not None

    for n in names:
        BDXR_ACTION_SCALE_LEGS[n] = 0.25 * e / s

if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_bdxr_robot_legs_cfg())

  viewer.launch(robot.spec.compile())
