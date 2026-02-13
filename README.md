https://github.com/user-attachments/assets/d2e4401f-b9c9-431d-b56d-e46ba38cffaa

# BDX-R in mjlab

This repository showcase the implementation of the BDX-R robot into [mjlab](https://github.com/mujocolab/mjlab). 

> [!WARNING]
> As mjlab is still in early development, this repository may be impacted by breaking changes. If an issue were to arise when running one of the scripts, feel free to open an issue or contribute to the project. Thanks you for your understanding!

## 🤖 What's mjlab?

mjlab is a project to have the [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/index.html) API using [MjWarp](https://mujoco.readthedocs.io/en/latest/mjwarp/index.html) as the backend. If you’re wondering about the motivation behind it or how it differs from Newton, you can learn more about it [here](https://github.com/mujocolab/mjlab/blob/main/docs/motivation.md).

## 🚀 Quickstart

Clone the repository.

```bash
git clone https://github.com/BDX-R/BDX-R-MjLab.git && cd BDX-R-MjLab
```

Get the robot description.

```bash
curl -L -o bdx_r_description.tar.gz https://github.com/KaydenKnapik/BDX-R-Description/archive/refs/heads/main.tar.gz
tar -xzf bdx_r_description.tar.gz -C src/bdx_r_mjlab/robots/bdx_r/
rm bdx_r_description.tar.gz
```

List available environments.

```bash
uv run bdx_r_list_envs
```

Use the dummy agents.

```bash
uv run bdx_r_play -Velocity-Flat-BDX-R --agent zero # send zero actions to the robot
uv run bdx_r_play -Velocity-Flat-BDX-R --agent random # send random actions to the robot
```

### Velocity Tracking

Train the policy.

```bash
uv run bdx_r_train -Velocity-Flat-BDX-R --env.scene.num-envs 4096
```

Evaluate the policy.

```bash
uv run bdx_r_play -Velocity-Flat-BDX-R-Play --wandb-run-path your-org//run-id
```

## 🎯 Roadmap

- [ ] Use IMU for observations
- [ ] Use simplified meshes for collisions
- [ ] Deploy the policy on real robot
- [ ] Use rough terrains
- [ ] Add a head to the robot

Check [here](https://github.com/BDX-R) for the general roadmap of the project!

## 🛠️ Contributing

We look forward for contributions. Before submitting a PR, please run the following command for format:

```bash
make format
```

## 🙏 Acknowledgements

We're grateful to the people behind mjlab, MuJoCo Warp and Isaac Lab.
