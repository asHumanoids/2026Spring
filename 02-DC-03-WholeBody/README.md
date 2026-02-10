# Whole Body Control for Humanoids

## Purpose
Whole body control (WBC) coordinates a humanoid's full set of joints and contacts to achieve multiple, sometimes competing, objectives at once. Typical objectives include balance, locomotion, manipulation, posture, and contact force regulation. Rather than planning each joint separately, WBC solves a single constrained problem that respects the robot's dynamics and physical limits.

## Core Ideas
- **Task hierarchy:** Multiple tasks (e.g., keep the torso upright, track foot placement, move the hand) are prioritized so higher-level tasks are preserved when lower-level tasks conflict.
- **Constraint handling:** Joint limits, torque limits, friction cones, and contact stability constraints are enforced during control.
- **Redundancy resolution:** Humanoids have more degrees of freedom than needed for a single task. WBC exploits this to satisfy secondary objectives without breaking primary ones.
- **Whole-body dynamics:** Control uses the full-body model so that decisions about one limb consider impacts on the rest of the body.

## Common Formulations
- **Operational-space control (OSC):** Specifies tasks in Cartesian space and maps them to joint torques using the robot's dynamics.
- **Quadratic programming (QP):** Poses control as an optimization with constraints and weighted objectives.
- **Hierarchical QP (HQP):** Uses stacked QPs to enforce strict task priorities.
- **Model predictive control (MPC):** Optimizes over a future horizon to anticipate contact changes and balance.

## Typical Tasks in a Humanoid WBC Stack
- Maintain center of mass (CoM) within the support polygon.
- Track swing foot trajectories for stepping.
- Regulate contact forces at the feet and hands.
- Stabilize torso orientation during motion.
- Coordinate arm motion for manipulation while walking.

## Practical Considerations
- **State estimation:** WBC quality depends on accurate base pose, velocity, and contact state estimates.
- **Contact transitions:** Smoothly adding/removing contacts is critical for stability.
- **Computation:** Real-time solvers must run at high rates (often 100 Hz to 1 kHz).
- **Tuning:** Task weights and priorities significantly affect behavior and robustness.

## Example Control Loop (High Level)
1. Read state estimates (pose, velocity, contacts).
2. Assemble tasks and constraints.
3. Solve the optimization (QP/HQP/MPC).
4. Send joint torques/commands to the low-level controller.
5. Repeat at the control rate.

## Further Reading
- Sentis, L. *Whole-Body Control of a Humanoid Robot* (2010)
- Khatib, O. (1987) Operational Space Formulation
- Saad, M. et al. (2019) Whole-Body Control Frameworks for Humanoids

---

This module introduces the key concepts and methods behind whole body control so you can connect perception, planning, and actuation into a single coordinated controller.
