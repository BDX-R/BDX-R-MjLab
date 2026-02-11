"""BDX-R velocity environment configurations."""

from bdx_r_mjlab.robots import (
  BDXR_ACTION_SCALE,
  get_bdxr_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, RayCastSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from bdx_r_mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg


def bdxr_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create BDX-R rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.contact_sensor_maxmatch = 500
  cfg.sim.nconmax = 45

  cfg.scene.entities = {"robot": get_bdxr_robot_cfg()}

  # --- Raycaster Removal (Rough) ---
  # Remove terrain_scan sensor from the scene
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  # Remove the height_scan observation terms safely
  if "height_scan" in cfg.observations["actor"].terms:
    del cfg.observations["actor"].terms["height_scan"]
  if "height_scan" in cfg.observations["critic"].terms:
    del cfg.observations["critic"].terms["height_scan"]
  # ---------------------------------

  site_names = ("left_foot", "right_foot") 
  geom_names = ("Left_Foot_Pad", "Right_Foot_Pad") 

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(
      mode="subtree",
      pattern=r"^(left_Left_Foot_Pad|Right_Foot_Pad)$",
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern="base_link", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="base_link", entity="robot"),
    fields=("found",),
    reduce="none",
    num_slots=1,
  )
  
  # Set sensors
  cfg.scene.sensors = (feet_ground_cfg, self_collision_cfg)

  if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
    cfg.scene.terrain.terrain_generator.curriculum = True

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = BDXR_ACTION_SCALE

  cfg.viewer.body_name = "base_link"

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.viz.z_offset = 1.15
  twist_cmd.ranges.lin_vel_x = (0.0, 0.0)
  twist_cmd.ranges.lin_vel_y = (-1, -1)
  twist_cmd.ranges.ang_vel_z = (0.0, 0.0)


  cfg.observations["critic"].terms["foot_height"].params[
    "asset_cfg"
  ].site_names = site_names

  cfg.events["base_com"].params["asset_cfg"].body_names = ("base_link",)

  # Rewards...
  cfg.rewards["pose"].params["std_standing"] = {".*": 0.05}
  cfg.rewards["pose"].params["std_walking"] = {
    r".*Neck_Pitch.*": 0.1, r".*Head_Pitch.*": 0.05, r".*Head_Yaw.*": 0.05, r".*Head_Roll.*": 0.05,
    r".*_Hip_Pitch.*": 0.3, r".*_Hip_Roll.*": 0.15, r".*_Hip_Yaw.*": 0.15,
    r".*_Knee.*": 0.35, r".*_Ankle.*": 0.1,
  }
  cfg.rewards["pose"].params["std_running"] = {
    r".*Neck_Pitch.*": 0.2, r".*Head_Pitch.*": 0.05, r".*Head_Yaw.*": 0.05, r".*Head_Roll.*": 0.05,
    r".*_Hip_Pitch.*": 0.5, r".*_Hip_Roll.*": 0.2, r".*_Hip_Yaw.*": 0.2,
    r".*_Knee.*": 0.6, r".*_Ankle.*": 0.2,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = ("base_link",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("base_link",)

  for reward_name in ["foot_clearance", "foot_swing_height", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = site_names

  cfg.rewards["body_ang_vel"].weight = -0.05
  cfg.rewards["angular_momentum"].weight = -0.02
  cfg.rewards["air_time"].weight = 0.0

  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-1.0,
    params={"sensor_name": self_collision_cfg.name},
  )

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

  return cfg


def bdxr_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create BDX-R flat terrain velocity configuration."""
  # This calls bdxr_rough_env_cfg, which already removes the raycaster
  cfg = bdxr_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  # Switch to flat terrain.
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # NOTE: Raycaster removal lines were removed from here because 
  # they are now handled inside bdxr_rough_env_cfg.

  # Disable terrain curriculum.
  if "terrain_levels" in cfg.curriculum:
    del cfg.curriculum["terrain_levels"]

  if play:
    # Disable command curriculum.
    if "command_vel" in cfg.curriculum:
      del cfg.curriculum["command_vel"]

    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg)
    twist_cmd.ranges.lin_vel_x = (0.4, 0.4)
    twist_cmd.ranges.lin_vel_y = (0.0, 0.0)
    twist_cmd.ranges.ang_vel_z = (0.0, 0.0)

  return cfg
