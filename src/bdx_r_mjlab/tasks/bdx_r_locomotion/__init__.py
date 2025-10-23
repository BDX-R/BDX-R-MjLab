import gymnasium as gym

gym.register(
    id="Mjlab-Velocity-Rough-BDX-R",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:BdxRRoughEnvCfg",
        "rl_cfg_entry_point": f"{__name__}.rl_cfg:BdxRPPORunnerCfg",
    },
)

gym.register(
    id="Mjlab-Velocity-Rough-BDX-R-Play",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:BdxRRoughEnvCfg_PLAY",
        "rl_cfg_entry_point": f"{__name__}.rl_cfg:BdxRPPORunnerCfg",
    },
)

gym.register(
    id="Mjlab-Velocity-Flat-BDX-R",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:BdxRFlatEnvCfg",
        "rl_cfg_entry_point": f"{__name__}.rl_cfg:BdxRPPORunnerCfg",
    },
)

gym.register(
    id="Mjlab-Velocity-Flat-BDX-R-Play",
    entry_point="mjlab.envs:ManagerBasedRlEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:BdxRFlatEnvCfg_PLAY",
        "rl_cfg_entry_point": f"{__name__}.rl_cfg:BdxRPPORunnerCfg",
    },
)
