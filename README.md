# BDX-R in mjlab

## Quickstart

## Clone the repository

```bash
git clone https://github.com/louislelay/bdx_r_mjlab.git
```

### Enter the repository

```bash
cd bdx_r_mjlab
```

### Download the asset

Get the robot description:

```bash
curl -L -o bdx_r_description.tar.gz https://github.com/KaydenKnapik/BDX-R-Description/archive/refs/heads/main.tar.gz
tar -xzf bdx_r_description.tar.gz -C src/bdx_r_mjlab/robots/bdx_r/
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
uv run scripts/play.py Mjlab-Velocity-Flat-BDX-R --wandb-run-path your-org/mjlab/run-id
```

## Acknowledgements

- Mjlab
- MuJoCo Warp
- Isaac Lab
