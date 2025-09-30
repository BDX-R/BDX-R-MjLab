# BDX-R in MjLab

## Quickstart

## Clone the repository

```bash
git clone https://github.com/BDX-R/BDX-R-MjLab.git
```

### Enter the repository

```bash
cd BDX-R-MjLab
```

### Download the asset

Get the robot description:

```bash
curl -L -o bdx_r_description.tar.gz https://github.com/KaydenKnapik/BDX-R-Description/archive/refs/heads/main.tar.gz
tar -xzf bdx_r_description.tar.gz -C src/BDX-R-MjLab/robots/bdx_r/
rm bdx_r_description.tar.gz
```

### List available environments

```bash
uv run scripts/list_envs.py
```

### Train

```bash
uv run scripts/train.py Mjlab-Velocity-Flat-BDX-R --env.scene.num-envs 4096
```

### Play

```bash
uv run scripts/play.py Mjlab-Velocity-Flat-BDX-R-Play --wandb-run-path your-org/mjlab/run-id
```

## Acknowledgements

- MjLab
- MuJoCo Warp
- Isaac Lab
