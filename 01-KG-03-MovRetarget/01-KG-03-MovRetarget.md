# 01-KG-03-MovRetarget — Motion Retargeting in Robotics

Motion retargeting is the problem of taking a motion demonstrated on one system (a human, a different robot, or a simulation character) and producing a motion for a *different* robot that preserves the intent of the original motion while respecting the target robot's physical and kinematic constraints.

In robotics, retargeting is usually not a simple "joint-to-joint" copy, because the source and target often differ in:
- link lengths and proportions (morphology)
- joint types and degrees of freedom (DoF)
- joint limits and velocity limits
- contact capabilities (feet, hands, suction, grippers)
- balance constraints (especially for humanoids)

## Why Retargeting Matters
- **Reuse motion data**: convert existing motion capture, animation, or prior robot trajectories into motions your robot can execute.
- **Teleoperation**: map human motion (or controller motion) to robot motion in real time.
- **Imitation learning / tracking**: create reference trajectories that a controller or RL policy can track.
- **Benchmarking**: compare controllers on a shared set of tasks/motions.

## What "Preserving the Motion" Usually Means
Different applications preserve different features. Common targets include:
- End-effector trajectories: hands/feet positions, orientations, and velocities
- Relative poses: e.g., hand-to-torso, foot-to-pelvis, gaze direction
- Contact schedule: which foot is in contact and when (humanoids)
- Style cues: smoothness, timing, symmetry

## Typical Retargeting Pipeline
1. **Normalize the source motion**
Convert into a representation that is easy to consume: per-frame poses, root trajectory, contact labels, and consistent coordinate frames.

2. **Define a mapping**
Choose how source features map to the target robot.
Examples: match human hand pose to robot wrist pose; match pelvis orientation; match foot contact points.

3. **Solve for a feasible robot motion**
Compute robot joint trajectories that best satisfy the mapping while honoring constraints.

4. **Post-process**
Time scaling, smoothing, joint-limit handling, collision filtering, and optional contact refinement.

## Common Approaches

### A. Rule-Based / Heuristic Retargeting
Fast and simple: scale translations by limb length ratios, clamp joint angles, and manually tune offsets.
Useful for quick demos but tends to break for contacts and high-precision tasks.

### B. Optimization-Based Kinematic Retargeting (IK)
Formulate per-frame or trajectory optimization:
- Decision variables: joint angles `q(t)` (and sometimes base pose)
- Objectives: minimize task-space errors (hands/feet/pelvis), regularize posture and smoothness
- Constraints: joint limits, velocity limits, self-collision (optional), contact constraints (optional)

A common trajectory-style objective (conceptually) is:
- minimize: tracking errors + posture regularization + smoothness
- subject to: kinematic constraints and joint bounds

### C. Learning-Based Retargeting
Train a model to map source pose/features to target robot pose.
Often combined with kinematic checks or a final IK projection step to ensure feasibility.

## Special Considerations for Humanoids
Retargeting walking, squatting, and other contact-rich motions requires more than matching joint angles:
- **Contacts**: feet (and sometimes hands) should remain stationary during stance.
- **Balance**: the CoM / ZMP should remain consistent with the support polygon (even if planning is kinematic).
- **Root motion**: pelvis/base translation and yaw must be chosen carefully; small errors cause foot slipping.

A common teaching decomposition is:
- footstep/contact planning
- CoM (or ZMP) planning for stability
- whole-body kinematic realization (IK) with stance/swing constraints

## What Is In This Folder
- `01-KG-03-MovRetarget-GMR/` contains an implementation of General Motion Retargeting (GMR).
- Start with `01-KG-03-MovRetarget-GMR/README.md` for an overview.
- `01-KG-03-MovRetarget-GMR/DOC.md` includes notes on IK configuration (match tables, weights, offsets).

## Suggested Exercises
1. Pick one motion clip and define what you want to preserve (hands, feet, pelvis, contacts).
2. Change the objective weights (e.g., prioritize feet vs hands) and observe the artifacts.
3. Add a simple smoothness term across time and compare to frame-by-frame IK.
4. If the motion involves stepping, check whether stance feet slip; then add stance constraints.

