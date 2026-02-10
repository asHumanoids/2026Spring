# 01-KG-05-HumanoidAdvanced — ZMP-Based Motion Planning (Unitree G1)

This folder focuses on Humanoid motion planning using Unitree G1 as an example.

## Background
Humanoid motion planning is often done in a hierarchy:
1. Footstep planning (discrete contacts)
 Decide where/when each foot lands (and sometimes which foot), given terrain and reachability.
2. Whole-body planning / control (continuous body motion with constraints)
 Given a contact sequence (e.g., footsteps), generate joint trajectories (arms/torso/legs) that satisfy kinematics, collisions, contact constraints, and balance objectives.
3. Kinodynamic planning (dynamics-aware planning)
 Plan trajectories that are feasible not only geometrically/kinematically, but also dynamically (forces/torques, momentum, friction cones, actuator limits). This can unify (1) and (2) or validate/refine them.

### ZMP-based Planning 

ZMP-based Planning is one of the most common pipelines for planning these motions, which decomposes the problem into:

#### 1. planning the footstep (or more generally the contact) positions

The footstep planning problem is a matter of choosing footstep placements on a given terrain from a start state to a goal, while ensuring that the sequence of steps can be safely executed.
Typically, the planning process consists of two stages:
- Global Path Planning: This module computes a high-level path for the robot to follow, usually avoiding obstacles and determining a viable route.
- Footstep Planning: Based on the global path, this step computes the exact placement of each foot to ensure the robot follows the intended trajectory.

Heuristic footstep planning
Each step's trajectory is defined as a sequence of homogeneous transformations in 3D space, generated from the following elements:
- 2D footsteps: The 2D feet positions generated from the previous step.
- Step Height: The vertical lift of the foot, ensuring clearance over obstacles.
- Step Time: The duration of each step, including the double supporting phase and the single supporting phase.

#### 2. planning the motion of the center of mass

In `ZMP_g1.ipynb`, we compute a **nominal CoM (x, y) trajectory** from a desired ZMP reference using Drake’s `pydrake.planning.ZmpPlanner`. Conceptually, this is the classic Linear Inverted Pendulum Model (LIPM) pipeline: assume a roughly constant CoM height and plan CoM motion so the resulting ZMP follows a reference that stays within the support polygon.

**How the notebook does it:**
- Build a piecewise-linear 2D ZMP reference `zmp_traj(t)` from the footstep sequence and the DS/SS timing (`generate_desired_zmp_traj`). During single support, the ZMP is held inside the stance foot (with a small lateral margin toward the midline). During double support, it shifts between feet.
- Choose the LIPM initial state `x0 = [com_x, com_y, com_vx, com_vy]` at the initial mid-foot with zero velocity.
- Choose an effective LIPM height `height` from the initial MuJoCo CoM height above the floor; for this kinematic demo we keep the absolute CoM height fixed as `com_z_ref`.
- Run the planner and sample the nominal CoM in the plane:

```python
zmp_planner = ZmpPlanner()
zmp_planner.Plan(zmp_traj, x0, height)
xy = np.asarray(zmp_planner.get_nominal_com(t)).reshape(2)
com_target = np.array([xy[0], xy[1], com_z_ref])
```

This gives a smooth CoM target (in x/y) that is consistent with the step timing and the desired ZMP shifts.

#### 3. calculating the individual joint motions to realize the desired center of mass and footstep trajectories

Once we have desired CoM and foot trajectories, we still need a whole-body configuration trajectory `q(t)` that realizes them. In `ZMP_g1.ipynb` this is done with a **purely kinematic, velocity-level IK loop** implemented directly on top of MuJoCo Jacobians, solved via a constrained damped least-squares (DLS) system.

**Key idea:** at each small IK timestep, solve for generalized velocities `qdot` that satisfy a set of *hard equality constraints* (CoM tracking, stance foot constraints, uprightness, etc.), while keeping the solution well-behaved via *soft objectives* (joint-centering, small velocities). Then integrate `qdot` forward to update `qpos`.

**What is constrained (from the notebook implementation):**
- Active DOFs are the floating base + leg joints; the upper-body joints are frozen (`UPPER_BODY_JOINTS_TO_FREEZE`).
- Feet targets come from `desired_feet_pos(t, ...)` (linear swing in XYZ with a sinusoidal lift).
- CoM target in x/y comes from Drake (`zmp_planner.get_nominal_com(t)`), with `z` held constant.
- Jacobians are computed with MuJoCo:
  - Foot sites: `mujoco.mj_jacSite` for position `J_p` and rotation `J_r`.
  - Whole-body CoM: `mujoco.mj_jacSubtreeCom` for CoM Jacobian.

**Constraints and objectives used:**
- Uprightness: drive the pelvis/torso z-axis toward world-up (roll/pitch stabilization) using a rotational Jacobian constraint (`upright_constraint_rows`).
- CoM tracking: constrain CoM x/y velocity so the CoM moves toward the target (`com_constraint_rows`).
- Foot orientation: both feet are constrained to have zero angular velocity (keeps soles flat).
- Foot position:
  - Double support: both feet positions are locked.
  - Single support: stance foot position is locked; swing foot position is driven toward the swing target with a proportional term `-k_swing * (p_cur - p_target)`.
- Soft regularization: a small velocity penalty plus a joint-centering term for leg hinges (keeps knees/ankles away from joint limits).

**Solve + integrate:**
- Assemble a constrained DLS problem in the active DOFs and solve it with a KKT system (`solve_constrained_dls`).
- Integrate with `mujoco.mj_integratePos(model, data.qpos, qvel, dt_ik)`.
- Re-freeze the upper body and clamp hinge limits each inner iteration, then record `qpos_traj`.

The planning here is purely kinematic: it generates feasible CoM and footstep trajectories without dynamics simulation or torque control.

- **ZMP idea**: The ZMP is the point on the ground where the net moment of the contact forces is zero. If the ZMP stays inside the support polygon (the convex hull of the feet in contact), the robot is considered statically stable.
- **Why ZMP planning**: ZMP provides a simple and intuitive stability criterion for bipedal walking. It makes it possible to plan walking patterns using linear models of the Center of Mass (CoM) motion.
- **Kinematics-only planning**: In this module we do not solve for actuator torques or full dynamics. Instead we compute footstep locations, CoM trajectories, and a kinematically consistent motion that respects stability constraints.
- **Preview control**: A common approach is to use preview control to plan CoM motion so the ZMP follows a desired reference over a look-ahead horizon. This is widely used in classical humanoid walking pattern generation.

## Notebooks
- `ZMP_g1.ipynb` — ZMP-based motion planning for the Unitree G1, including trajectory setup and kinematic walking generation.
- `ZMP_preview_control.ipynb` — preview control formulation for ZMP tracking and CoM trajectory generation.

## How to run
1. Installation
```bash
conda activate asHumanoids
pip install mujoco drake

```
2. Open a notebook in Jupyter.
3. Run the cells in order to explore the ZMP model, reference generation, and resulting motion trajectories.

## Learning goals
- Understand the ZMP stability criterion and support polygon.
- See how footsteps and CoM trajectories can be planned without full dynamics.
- Learn how preview control shapes ZMP and CoM motion for walking.
