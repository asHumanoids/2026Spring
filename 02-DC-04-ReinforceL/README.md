# Reinforcement Learning Pipeline for Humanoid Training

A common pipeline for training humanoid robots with reinforcement learning (RL) progresses through three stages:

1. **Train in simulation**: Start by training the policy in a physics simulator where data is cheap, safe, and fast to generate. This phase focuses on learning stable locomotion and basic behaviors.
2. **Sim2sim**: Transfer the policy between different simulators or simulator settings to improve robustness to modeling differences. This helps the policy generalize beyond a single simulated environment.
3. **Sim2real**: Finally, deploy the policy on the real robot, using careful calibration and safety constraints. Real-world fine-tuning may be applied to bridge the remaining gap between simulation and hardware.

## Example Flow Using the Unitree G1 Folders

- **Train in simulation**: Use `unitree_mujoco/` to run MuJoCo-based humanoid simulation and `unitree_rl_lab/` to train RL policies on that simulator.
- **Sim2sim**: Validate the learned policy across simulator settings or alternate simulator configs (e.g., different physics parameters, contact models, or terrain variations) using the same training/evaluation stack in `unitree_rl_lab/` with modified configs.
- **Sim2real**: Deploy to hardware through `unitree_sdk2/` (C++ SDK) or `unitree_sdk2_python/` (Python SDK), which interface with the real G1 robot.

## Diagram

```
Sim Training            Sim2Sim Robustness             Sim2Real Deployment
--------------          ------------------             -------------------
unitree_mujoco/         unitree_rl_lab/                unitree_sdk2/
   +                   (domain randomization,          unitree_sdk2_python/
unitree_rl_lab/         parameter sweeps)
       |                         |                               |
       +----------- policy ------+--------------- policy --------+
```
