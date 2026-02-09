# 01-KG-04-HumanoidBasics — Unitree G1 MuJoCo demos

This folder demonstrates the Unitree G1 robot in MuJoCo across multiple configurations (various degrees of freedom). It includes example XMLs and notebooks that show how to load each XML, display it, and run teleoperation via the asMagic app.

## Included demos (common XML names)
- Head yaw (1 DoF): `g1_head_1dof.xml`
- Single arm (5 DoF): `g1_one_arm_5dof.xml`
- Dual arms (14 DoF): `g1_two_arms_14dof.xml`
- Waist (3 DoF): `g1_waist_3dof.xml`
- Two legs (12 DoF): `g1_two_legs_12dof.xml`
- Full robot (29 DoF): `g1_29dof.xml`

## Notebooks
- `g1_mujoco.ipynb` — displays the XMLs above for different DoF setups, builds a `mujoco.MjModel`, and launches the MuJoCo viewer with `mujoco.viewer.launch(model, data)`.
- `teleop_g1_mujoco_asmagic.ipynb` — teleoperation using the asMagic app. It wires phone pose input into IK targets and drives the MuJoCo model in a live loop.

## Quick Installation
These steps match the environment used by the notebooks in this folder.

1. Create/activate the Python environment you want to use (recommended: venv or conda).
```bash
conda create -n asHumanoids python=3.11
conda activate asHumanoids
```

2. Install Python packages (MuJoCo + runtime libs used in the notebooks):

```bash
pip install mujoco mink loop-rate-limiters asmagic scipy
```

Note: the notebooks also use `IPython` display utilities (Jupyter) if you run them interactively. If you run in a Jupyter notebook, ensure the kernel matches the environment where you installed the packages.

## How to run
- From Jupyter Notebook: open `g1_mujoco.ipynb` to explore the XML configurations and viewer.
- For teleoperation: open `teleop_g1_mujoco_asmagic.ipynb`, connect the asMagic app, then run the cells to stream phone poses into the IK loop.

## Teleoperation example (asMagic)
The teleoperation notebook integrates `mink` IK, `asmagic` phone input, and mocap targets. The code example demonstrates:
- constructing a `mink.Configuration` from a loaded `mujoco.MjModel`
- defining `mink.FrameTask`, `mink.PostureTask`, and other tasks
- receiving phone poses using `asmagic.ARDataSubscriber`
- converting phone poses and applying deltas to hand targets
- running a control loop that solves IK and updates the MuJoCo viewer

See `teleop_g1_mujoco_asmagic.ipynb` for the full implementation and parameter choices.

## Files
- Notebook: `g1_mujoco.ipynb` — XML display + viewer demos.
- Notebook: `teleop_g1_mujoco_asmagic.ipynb` — teleop via asMagic app.
- XMLs: `unitree_g1/*.xml` — the robot configurations listed above.
