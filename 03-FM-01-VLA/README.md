# Foundation Models in Robotics: VLA and VLN

Foundation models are becoming a core building block for general-purpose robot intelligence.  
Instead of training a separate policy for each task, these models learn broad priors from large-scale data and can be adapted to new tasks with less task-specific engineering.

## Two Important Paradigms

1. **VLA (Vision-Language-Action)**  
VLA models map visual observations and language instructions directly to robot actions.  
Typical form:
- Input: camera images/video + text instruction
- Output: low-level or mid-level control actions

2. **VLN (Vision-Language-Navigation)**  
VLN models focus on instruction-following for navigation in 3D environments.  
Typical form:
- Input: egocentric observations + language route/task instruction
- Output: navigation decisions (move/turn/stop or waypoint-level plans)

## Why This Matters for Robotics

- Better generalization across scenes and tasks
- Natural-language task specification
- Reusable pretrained representations
- Reduced dependence on dense task-specific reward shaping

## GR00T as the Example in This Module

In this folder, **GR00T** is used as the practical example of robotics foundation-model workflows.

- Use GR00T to study how language and visual context are transformed into action-relevant representations.
- Use simulation rollouts to validate behavior before hardware deployment.
- Connect model outputs to downstream robot control stacks (sim first, then real systems).

### Installation

```bash
git clone --recurse-submodules https://github.com/NVIDIA/Isaac-GR00T
cd Isaac-GR00T
uv sync
uv pip install -e .
```

### Evaluate model on the GR00T WholeBodyControl example
```bash
sudo apt-get update
sudo apt-get install libegl1-mesa-dev libglu1-mesa
# install git lfs
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
bash gr00t/eval/sim/GR00T-WholeBodyControl/setup_GR00T_WholeBodyControl.sh

# Terminal 1 for server
# Use model from https://github.com/NVIDIA/Isaac-GR00T/issues/574#issuecomment-4082084479
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path nvidia/GR00T-N1.6-G1-PnPAppleToPlate \
    --embodiment-tag UNITREE_G1 \
    --use-sim-policy-wrapper

# Terminal 2 for client, different env from server
export MUJOCO_GL=glx
gr00t/eval/sim/GR00T-WholeBodyControl/GR00T-WholeBodyControl_uv/.venv/bin/python gr00t/eval/rollout_policy.py \
    --policy_client_host 127.0.0.1 \
    --policy_client_port 5555 \
    --n_episodes 10 \
    --max_episode_steps=1440 \
    --env_name gr00tlocomanip_g1_sim/LMPnPAppleToPlateDC_G1_gear_wbc \
    --n_action_steps 20 \
    --n_envs 5

# if you want to visualize the window during inferencing, set onscreen=True in rollout_policy.py
```