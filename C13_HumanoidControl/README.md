# Class 13 on Humanoid Control

## Contents

1. Motivation and scope
2. Mathematical preliminaries and notation
3. Control architecture for humanoid robots
4. Floating-base dynamics and control implications
5. Task-space control and operational-space ideas
6. Centroidal dynamics and reduced-order balance control
7. Whole-body control
8. Impedance, admittance, and force regulation
9. Model predictive control for humanoids
10. Control of locomotion, manipulation, and loco-manipulation
11. From theory to implementation
12. Research directions and open problems
13. Summary of key takeaways
14. Suggested implementation roadmap for students
15. References for further study

# 1. Motivation and scope

## 1.1 Why humanoid control is fundamentally different

Humanoid control is not simply “robot control with more joints.” A humanoid is a floating-base multibody system that must coordinate many degrees of freedom while maintaining balance, respecting contact constraints, and performing tasks with the upper and lower body at the same time. Unlike a fixed-base industrial manipulator, a humanoid cannot directly command the wrench of its base with respect to the world. The robot can only influence its global motion through internal joint torques and external contact forces.

This one fact makes humanoid control qualitatively different from standard manipulator control. The controller is not only trying to move links to target poses. It must decide, explicitly or implicitly, how whole-body motion, contact forces, and task priorities interact. The robot should simultaneously:
- stabilize itself under gravity;
- satisfy contact constraints at the feet and possibly hands;
- regulate the center of mass (CoM), linear momentum, and angular momentum;
- track operational tasks such as swing-foot placement, hand trajectories, gaze, or posture;
- keep joint torques, contact forces, and friction demands feasible;
- transition robustly between contact modes.

In other words, humanoid control is a structured coordination problem over motion, force, and contact.

## 1.2 The predictive-reactive view of humanoid control

A useful high-level view is that modern humanoid control often follows a **predictive-reactive hierarchy**. A predictive layer reasons over a future horizon, often with reduced-order models such as centroidal dynamics or linear inverted pendulum variants, to produce references for balance, footsteps, or net wrenches. A reactive layer then resolves the full-body motion and force distribution subject to instantaneous constraints. In much of the recent literature, this combination takes the form of a model predictive controller (MPC) coupled with a local whole-body controller (WBC).

This hierarchy is attractive because different time scales and different model complexities are handled where they fit best:
- long-horizon balance and motion intent are planned predictively;
- fast constraint enforcement and task coordination are handled reactively;
- reduced-order models provide speed;
- full-order inverse dynamics or kinematics provide physical consistency.

## 1.3 Scope of these notes

These notes develop a graduate-level introduction to humanoid control with emphasis on model-based methods. The goal is to connect classical robot control ideas to the realities of floating-base balance, multi-contact interaction, and whole-body coordination.

The notes cover:
- floating-base equations of motion and why underactuation matters;
- task-space and operational-space viewpoints;
- centroidal dynamics and reduced-order balance models;
- whole-body control formulations based on kinematics, inverse dynamics, and optimization;
- impedance and force regulation for physical interaction;
- model predictive control for humanoid locomotion and loco-manipulation;
- implementation-oriented guidance, including solver structure, estimation interfaces, contact scheduling, and debugging.

The notes do **not** attempt a full treatment of humanoid learning-based control. Reinforcement learning, imitation learning, and foundation-model-based decision making are intentionally left to the subsequent Humanoid Learning class.

---

# 2. Mathematical preliminaries and notation

## 2.1 Configuration and generalized velocity

Let the humanoid configuration be
$$
\mathbf{q} = \begin{bmatrix} \mathbf{q}_b \\ \mathbf{q}_a \end{bmatrix},
$$
where $\mathbf{q}_b$ denotes the floating-base pose and $\mathbf{q}_a \in \mathbb{R}^{n_a}$ denotes the actuated joint coordinates.

The generalized velocity is
$$
\mathbf{v} = \begin{bmatrix} \mathbf{v}_b \\ \dot{\mathbf{q}}_a \end{bmatrix} \in \mathbb{R}^{n_v},
$$
where $\mathbf{v}_b \in \mathbb{R}^6$ is the spatial velocity of the base.

For floating-base systems it is usually better to formulate dynamics in terms of $(\mathbf{q}, \mathbf{v})$ rather than $(\mathbf{q}, \dot{\mathbf{q}})$ because orientation coordinates are not always globally smooth or minimal.

## 2.2 Equations of motion

The standard floating-base rigid-body dynamics under contact are
$$
\mathbf{M}(\mathbf{q}) \dot{\mathbf{v}} + \mathbf{h}(\mathbf{q}, \mathbf{v})
=
\mathbf{S}^\top \boldsymbol{\tau}
+
\mathbf{J}_c(\mathbf{q})^\top \boldsymbol{\lambda}.
$$

Here:
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{n_v \times n_v}$ is the mass matrix;
- $\mathbf{h}(\mathbf{q}, \mathbf{v})$ collects Coriolis, centrifugal, gravity, and possibly modeled passive effects;
- $\mathbf{S}$ selects actuated coordinates;
- $\boldsymbol{\tau} \in \mathbb{R}^{n_a}$ is the joint torque vector;
- $\mathbf{J}_c(\mathbf{q})$ is the contact Jacobian;
- $\boldsymbol{\lambda}$ parameterizes the contact force or contact wrench variables.

The actuation matrix
$$
\mathbf{S} = \begin{bmatrix} \mathbf{0}_{n_a \times 6} & \mathbf{I}_{n_a} \end{bmatrix}
$$
captures underactuation: the base coordinates are not directly actuated.

## 2.3 Contact constraints

If a contact frame is assumed to stick relative to the environment, then its velocity must vanish:
$$
\mathbf{J}_c(\mathbf{q}) \mathbf{v} = \mathbf{0}.
$$

Differentiating gives the acceleration-level constraint:
$$
\mathbf{J}_c(\mathbf{q}) \dot{\mathbf{v}}
+
\dot{\mathbf{J}}_c(\mathbf{q}, \mathbf{v}) \mathbf{v}
=
\mathbf{0}.
$$

Together with the equations of motion, this creates a constrained dynamics system in the unknown acceleration and contact force:
$$
\begin{bmatrix}
\mathbf{M} & -\mathbf{J}_c^\top \\
\mathbf{J}_c & \mathbf{0}
\end{bmatrix}
\begin{bmatrix}
\dot{\mathbf{v}} \\
\boldsymbol{\lambda}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{S}^\top \boldsymbol{\tau} - \mathbf{h} \\
-\dot{\mathbf{J}}_c \mathbf{v}
\end{bmatrix}.
$$

## 2.4 Task maps and Jacobians

A task variable $\mathbf{x}(\mathbf{q})$ may describe:
- hand position or pose;
- swing-foot pose;
- CoM position;
- torso orientation;
- gaze direction;
- relative object pose.

Its differential kinematics are
$$
\dot{\mathbf{x}} = \mathbf{J}_x(\mathbf{q}) \mathbf{v},
$$
and its acceleration is
$$
\ddot{\mathbf{x}} = \mathbf{J}_x(\mathbf{q}) \dot{\mathbf{v}} + \dot{\mathbf{J}}_x(\mathbf{q}, \mathbf{v}) \mathbf{v}.
$$

These relations are the basic building blocks for task-space feedback and whole-body optimization.

## 2.5 Centroidal quantities

Let $\mathbf{c}$ be the CoM. The centroidal momentum is
$$
\mathbf{h}_G =
\begin{bmatrix}
\mathbf{l} \\
\mathbf{k}
\end{bmatrix},
$$
where $\mathbf{l}$ is linear momentum and $\mathbf{k}$ is angular momentum about the CoM.

The centroidal momentum matrix $\mathbf{A}_G(\mathbf{q})$ satisfies
$$
\mathbf{h}_G = \mathbf{A}_G(\mathbf{q}) \mathbf{v}.
$$

The centroidal dynamics are
$$
m \ddot{\mathbf{c}} = \sum_i \mathbf{f}_i + m \mathbf{g},
$$
$$
\dot{\mathbf{k}} = \sum_i (\mathbf{p}_i - \mathbf{c}) \times \mathbf{f}_i + \sum_i \boldsymbol{\mu}_i,
$$
where $(\mathbf{p}_i, \mathbf{f}_i, \boldsymbol{\mu}_i)$ denote the location, force, and moment of contact $i$.

These equations are central because they reduce full-body behavior to the net effect of contact wrenches on global motion.

---

# 3. Control architecture for humanoid robots

## 3.1 A layered view

A practical humanoid controller is rarely a single monolithic law. Most systems use layers operating at different time scales:

1. **State estimation and contact estimation**  
   Estimate base pose, velocity, IMU state, contact states, and sometimes contact forces.

2. **Mode and gait scheduling**  
   Decide whether the robot is in double support, single support, hand support, or a transition state.

3. **Predictive motion generation**  
   Compute future references for CoM, momentum, footsteps, or net contact wrench over a horizon.

4. **Whole-body control / inverse dynamics**  
   Convert those references into instantaneous accelerations, torques, and contact forces that satisfy the robot constraints.

5. **Joint-level tracking and motor control**  
   Execute commanded torques, currents, or positions with high-rate servo loops.

This layered decomposition exists because no single model is equally suitable for all time scales and all computational budgets.

## 3.2 Why hierarchy matters

The most important reason for hierarchy is not software modularity. It is **model mismatch across scales**.

- Long-horizon planning can tolerate simplified dynamics if it captures the correct balance structure.
- Short-horizon constraint satisfaction must use more accurate kinematics and contact models.
- Actuator dynamics, delays, and drive electronics require still faster loops.

Humanoid control is not “one controller,” but an architecture in which different mathematical objects talk to one another.

## 3.3 Common architectural patterns

Three common patterns appear repeatedly.

### (a) Reduced-order planner + WBC
A reduced model such as LIP, centroidal dynamics, or a wrench model plans balance-related quantities. A WBC then produces full-body commands.

### (b) Task-space inverse dynamics
A local optimization computes accelerations, torques, and contact forces directly from task errors and constraints, often without a separate long-horizon planner in simpler tasks.

### (c) MPC + inverse dynamics
A predictive controller produces future momentum or contact-force references, while inverse dynamics enforces full-body consistency at high rate.

The current literature strongly favors predictive-reactive hierarchies, especially MPC combined with WBC.

---

# 4. Floating-base dynamics and control implications

## 4.1 Underactuation and the role of contact

For a fixed-base manipulator, inverse dynamics can in principle produce any sufficiently smooth acceleration trajectory that is dynamically feasible and within torque limits. For a humanoid, this is not true because the base is unactuated. The robot cannot directly impose arbitrary global accelerations. Without contact, the generalized actuation lies in a lower-dimensional subspace.

A clean way to see this is to partition the dynamics into base and actuated rows:
$$
\begin{aligned}
\mathbf{M}_b(\mathbf{q}) \dot{\mathbf{v}} + \mathbf{h}_b(\mathbf{q}, \mathbf{v})
&= \mathbf{J}_{c,b}^\top \boldsymbol{\lambda}, \\
\mathbf{M}_a(\mathbf{q}) \dot{\mathbf{v}} + \mathbf{h}_a(\mathbf{q}, \mathbf{v})
&= \boldsymbol{\tau} + \mathbf{J}_{c,a}^\top \boldsymbol{\lambda}.
\end{aligned}
$$

The base equation contains no direct torque input. Global motion is shaped through contact.

## 4.2 Contact-consistent motion

Suppose both feet are flat on the ground. Then a desired acceleration must satisfy the contact constraints:
$$
\mathbf{J}_c \dot{\mathbf{v}} + \dot{\mathbf{J}}_c \mathbf{v} = \mathbf{0}.
$$

If a controller asks for an acceleration incompatible with this equation, then one of three things must happen:
- the desired motion is projected into the feasible subspace;
- the contact assumption is changed;
- large tracking errors or contact violations occur.

This is why whole-body controllers work with explicit equality and inequality constraints rather than just independent joint PD loops.

## 4.3 Contact forces are part of the control problem

In humanoid control, contact forces are not merely disturbances to reject. They are decision variables. The controller often computes
- which contacts should support load;
- how much normal force each contact should bear;
- how tangential forces should be distributed to avoid slip;
- whether internal forces should be regulated between multiple contacts.

Thus motion control and force distribution are coupled.

## 4.4 Hybrid control viewpoint

Because contact modes switch over time, humanoid control is inherently hybrid. Each contact mode defines a different constrained dynamics manifold. The controller must therefore handle:
- within-mode continuous dynamics;
- discrete transitions such as touchdown and liftoff;
- reset events due to impact or velocity jumps;
- controller gain and task changes during transitions.

This explains why walking and loco-manipulation controllers are often written as state machines or phase-dependent optimization problems.

---

# 5. Task-space control and operational-space ideas

## 5.1 From joint-space tracking to task-space regulation

Joint-space control is often too indirect for humanoids. A humanoid should regulate physically meaningful quantities such as:
- CoM location;
- swing-foot position and velocity;
- torso or pelvis orientation;
- hand pose relative to the world or an object;
- contact force at a support hand.

Task-space control expresses desired behavior directly in these variables.

Let a task error be
$$
\mathbf{e}_x = \mathbf{x} - \mathbf{x}_d,
$$
with a desired second-order servo law
$$
\ddot{\mathbf{x}}^\star
=
\ddot{\mathbf{x}}_d
- \mathbf{K}_d \dot{\mathbf{e}}_x
- \mathbf{K}_p \mathbf{e}_x.
$$

Using
$$
\ddot{\mathbf{x}}
=
\mathbf{J}_x \dot{\mathbf{v}} + \dot{\mathbf{J}}_x \mathbf{v},
$$
the controller can impose
$$
\mathbf{J}_x \dot{\mathbf{v}}
=
\ddot{\mathbf{x}}^\star - \dot{\mathbf{J}}_x \mathbf{v}.
$$

This linear form is especially useful in optimization-based WBC.

## 5.2 Operational-space intuition

Operational-space control asks a conceptually important question:

> What is the dynamic behavior of the robot as seen from a task coordinate rather than from its joints?

For fixed-base manipulators, the classic operational-space formulation introduces the task-space inertia
$$
\boldsymbol{\Lambda}_x = \left( \mathbf{J}_x \mathbf{M}^{-1} \mathbf{J}_x^\top \right)^{-1},
$$
under suitable rank assumptions. This quantity tells us how difficult it is to accelerate the task in a given direction.

For humanoids under contact, an analogous idea still matters conceptually, but the presence of a floating base and contact constraints means the effective task dynamics depend on the constrained system. In practice, most humanoid systems use the operational-space intuition while implementing it through inverse dynamics or optimization rather than through closed-form decoupling.

## 5.3 Null-space and redundancy

Humanoids are highly redundant. The same hand task may be achievable with many postures, and a given CoM task may be compatible with multiple contact-force distributions.

This suggests a hierarchy:
1. satisfy essential constraints and high-priority tasks;
2. use remaining redundancy for posture, energy, torque margin, or joint-limit avoidance.

In classical formulations this is expressed with null-space projectors. In modern optimization-based controllers it is often expressed with stacked priorities or weighted costs.

## 5.4 Limits of pure task-space feedback

Task-space PD control alone is not enough for humanoids because it does not automatically guarantee:
- contact feasibility;
- friction satisfaction;
- torque limits;
- dynamic consistency;
- momentum consistency across tasks.

It remains important as a language for specifying desired behavior, but it is usually embedded inside a larger constrained control formulation.

---

# 6. Centroidal dynamics and reduced-order balance control

## 6.1 Why reduced-order models are used

Full humanoid dynamics are too expensive and too detailed for every planning layer. Many control problems, especially balance and locomotion, depend more strongly on **global mass distribution and support geometry** than on finger-level or wrist-level details.

Reduced-order models abstract the robot by keeping the quantities most relevant to balance:
- CoM motion;
- net contact wrench;
- sometimes angular momentum;
- sometimes step placement.

This allows faster predictive control.

## 6.2 Linear inverted pendulum model

The linear inverted pendulum model (LIPM) assumes:
- constant CoM height $z_c$;
- negligible centroidal angular momentum;
- point-foot or equivalent flat-ground support simplification.

In one horizontal direction,
$$
\ddot{x} = \omega^2 (x - x_Z), \qquad \omega = \sqrt{\frac{g}{z_c}},
$$
where $x$ is the CoM coordinate and $x_Z$ is the ZMP coordinate.

This model is attractive because it is linear and exposes the unstable component of CoM motion. It supports elegant concepts such as the capture point:
$$
\xi = x + \frac{\dot{x}}{\omega}.
$$

Despite its simplicity, the LIPM still underlies many controllers and teaching materials because it gives clear intuition about balance and stepping.

## 6.3 Limits of the LIPM

The LIPM is too restrictive when:
- CoM height varies significantly;
- upper-body angular momentum matters;
- terrain is not flat;
- contacts are non-coplanar;
- hand support is important;
- manipulation loads affect whole-body balance.

Therefore it is best taught as a conceptual baseline rather than as a final humanoid control model.

## 6.4 Centroidal dynamics as a richer reduced model

Centroidal dynamics keep the physically important global quantities:
$$
m \ddot{\mathbf{c}} = \sum_i \mathbf{f}_i + m \mathbf{g},
$$
$$
\dot{\mathbf{k}} = \sum_i (\mathbf{p}_i - \mathbf{c}) \times \mathbf{f}_i + \sum_i \boldsymbol{\mu}_i.
$$

Compared with the LIPM, centroidal dynamics:
- permit varying CoM height;
- incorporate multiple contacts;
- capture angular momentum;
- represent manipulation interaction forces more naturally.

For this reason they are widely used in modern humanoid MPC.

## 6.5 Wrench-feasibility viewpoint

A powerful control view is that balance can be seen as a wrench-feasibility problem. The controller seeks contact forces such that:
1. the resulting net wrench drives the CoM and angular momentum as desired;
2. each contact force lies within its feasible friction or wrench cone;
3. actuator and posture constraints remain satisfiable.

This perspective connects reduced-order planning directly to whole-body realization.

## 6.6 Momentum regulation

A common centroidal control idea is to regulate momentum:
$$
\dot{\mathbf{h}}_G^\star
=
- \mathbf{K}_h (\mathbf{h}_G - \mathbf{h}_{G,d})
+ \dot{\mathbf{h}}_{G,d}.
$$

Given this desired momentum rate, one solves for contact forces or a net wrench that realizes it. Then the full-body controller maps those force objectives into consistent joint torques and accelerations.

This makes momentum a bridge between balance and full-body motion.

---

# 7. Whole-body control

## 7.1 What whole-body control tries to do

Whole-body control (WBC) coordinates the full robot under constraints. A typical WBC simultaneously handles:
- rigid contact constraints;
- CoM or momentum regulation;
- swing-foot tracking;
- torso stabilization;
- hand pose or force tasks;
- posture regularization;
- torque and friction limits.

The essential idea is not a particular solver. It is the explicit joint treatment of tasks and constraints at the scale of the full robot.

## 7.2 Kinematic vs dynamic WBC

### Kinematic WBC
Kinematic WBC solves for generalized velocities or accelerations from task Jacobians and contact constraints, without explicitly enforcing the full rigid-body dynamics. It can work well when low-level torque control is unavailable or when motion generation is the priority.

### Dynamic WBC
Dynamic WBC solves for $\dot{\mathbf{v}}$, $\boldsymbol{\tau}$, and often $\boldsymbol{\lambda}$ while enforcing the equations of motion. This is better suited to force-sensitive balance and contact-rich tasks.

## 7.3 Optimization form of WBC

A common inverse-dynamics whole-body controller is formulated as a constrained optimization problem over the generalized acceleration, actuator torque, and contact force:

$$
\min_{\dot{\mathbf{v}}, \boldsymbol{\tau}, \boldsymbol{\lambda}}
\quad
\sum_j \left\| \mathbf{W}_j \left(\mathbf{J}_j \dot{\mathbf{v}} - \mathbf{b}_j \right) \right\|^2
+
\left\| \mathbf{W}_{\tau} \boldsymbol{\tau} \right\|^2
+
\left\| \mathbf{W}_{\lambda} \boldsymbol{\lambda} \right\|^2
$$
subject to
$$
\mathbf{M} \dot{\mathbf{v}} + \mathbf{h} = \mathbf{S}^\top \boldsymbol{\tau} + \mathbf{J}_c^\top \boldsymbol{\lambda},
$$
$$
\mathbf{J}_c \dot{\mathbf{v}} + \dot{\mathbf{J}}_c \mathbf{v} = \mathbf{0},
$$
$$
\mathbf{A}_{\lambda} \boldsymbol{\lambda} \le \mathbf{b}_{\lambda},
$$
$$
\boldsymbol{\tau}_{\min} \le \boldsymbol{\tau} \le \boldsymbol{\tau}_{\max},
$$
$$
\dot{\mathbf{v}}_{\min} \le \dot{\mathbf{v}} \le \dot{\mathbf{v}}_{\max}.
$$

This optimization is one of the standard mathematical forms of dynamic whole-body control. It expresses a central idea of humanoid control: the robot should realize multiple motion objectives simultaneously, but only in ways that remain dynamically feasible, contact-consistent, and actuator-feasible.

### Decision variables and physical meaning

The unknowns are:
- $\dot{\mathbf{v}} \in \mathbb{R}^{6+n}$, the generalized acceleration of the floating-base humanoid,
- $\boldsymbol{\tau} \in \mathbb{R}^{n}$, the vector of joint torques at the actuated joints,
- $\boldsymbol{\lambda} \in \mathbb{R}^{m}$, the stacked vector of contact forces or contact wrenches.

Here $n$ is the number of actuated joints, and the extra $6$ components in $\mathbf{v}$ correspond to the unactuated floating base. It is common in humanoid control to optimize over $\dot{\mathbf{v}}$ rather than directly over $\ddot{\mathbf{q}}$, because the generalized velocity representation is often cleaner for floating-base rigid-body dynamics.

The matrix $\mathbf{M}$ is the full generalized inertia matrix, $\mathbf{h}$ collects Coriolis, centrifugal, gravity, and sometimes modeled disturbances, $\mathbf{S}$ is the actuation selection matrix, and $\mathbf{J}_c$ is the Jacobian of the active contact constraints.

---

### Task terms in the objective

The term
$$
\left\| \mathbf{W}_j \left(\mathbf{J}_j \dot{\mathbf{v}} - \mathbf{b}_j \right) \right\|^2
$$
represents task $j$.

Each task defines a desired relation at the acceleration level. The matrix $\mathbf{J}_j$ is the Jacobian associated with that task, and $\mathbf{b}_j$ is the desired task-space acceleration after compensating for known kinematic terms. The controller tries to choose $\dot{\mathbf{v}}$ so that the realized task acceleration $\mathbf{J}_j \dot{\mathbf{v}}$ is close to the desired one.

A common construction is
$$
\mathbf{J}_j \dot{\mathbf{v}} + \dot{\mathbf{J}}_j \mathbf{v} = \ddot{\mathbf{x}}_j,
$$
where $\mathbf{x}_j$ is the task variable, such as center-of-mass position, torso orientation, hand pose, or swing-foot pose. If a desired task acceleration $\ddot{\mathbf{x}}_j^\star$ is specified, then the acceleration matching condition becomes
$$
\mathbf{J}_j \dot{\mathbf{v}} \approx \ddot{\mathbf{x}}_j^\star - \dot{\mathbf{J}}_j \mathbf{v}.
$$
Thus one usually sets
$$
\mathbf{b}_j = \ddot{\mathbf{x}}_j^\star - \dot{\mathbf{J}}_j \mathbf{v}.
$$

A standard choice for $\ddot{\mathbf{x}}_j^\star$ is a PD-type feedback law in task space:
$$
\ddot{\mathbf{x}}_j^\star
=
\ddot{\mathbf{x}}_{j,\mathrm{ref}}
+
\mathbf{K}_{d,j} \big(\dot{\mathbf{x}}_{j,\mathrm{ref}} - \dot{\mathbf{x}}_j\big)
+
\mathbf{K}_{p,j} \big(\mathbf{x}_{j,\mathrm{ref}} - \mathbf{x}_j\big).
$$

This means that the optimization is not directly tracking position or velocity; rather, it is tracking a desired task acceleration generated from motion errors.

Typical tasks include:
- center-of-mass regulation,
- pelvis or torso orientation stabilization,
- swing-foot trajectory tracking,
- hand pose tracking,
- gaze direction,
- postural regularization,
- momentum regulation.

The weighting matrix $\mathbf{W}_j$ determines how strongly task $j$ influences the solution. Larger weights push the optimizer to satisfy that task more closely. In practice, these weights encode soft priorities when all tasks cannot be satisfied exactly at the same time.

---

### Torque and contact regularization

The terms
$$
\left\| \mathbf{W}_{\tau} \boldsymbol{\tau} \right\|^2
\qquad \text{and} \qquad
\left\| \mathbf{W}_{\lambda} \boldsymbol{\lambda} \right\|^2
$$
are regularization terms.

They do not usually represent primary control objectives. Their role is to bias the solution toward numerically well-behaved, physically reasonable actuation and contact force distributions.

The torque penalty discourages unnecessarily large actuator effort. This often improves robustness, reduces energy consumption, and prevents the optimizer from relying on aggressive torque cancellation.

The contact-force penalty regularizes the distribution of forces among multiple contacts. In double support or multi-contact motion, there are often many contact-force solutions that satisfy the same net-body dynamics. Penalizing $\boldsymbol{\lambda}$ helps avoid arbitrary force redistribution and can suppress excessive internal forces.

In some implementations, the regularization on $\boldsymbol{\lambda}$ is replaced or supplemented by a force-tracking term, especially when one wants a desired load-sharing pattern between contacts.

---

### Dynamic feasibility constraint

The equation
$$
\mathbf{M} \dot{\mathbf{v}} + \mathbf{h} = \mathbf{S}^\top \boldsymbol{\tau} + \mathbf{J}_c^\top \boldsymbol{\lambda}
$$
is the rigid-body dynamics of the floating-base humanoid.

This is the main physical consistency constraint in the problem. It states that the generalized inertial and bias forces must be balanced by actuator torques and contact forces.

This equation is important because task-space objectives alone do not guarantee physically realizable motion. For example, a desired swing-foot acceleration, torso motion, and center-of-mass correction might be kinematically meaningful, but they are invalid unless some torque and contact-force combination can actually produce them.

For humanoids, this constraint is essential because the base is not directly actuated. The floating base can only be accelerated indirectly through internal joint torques and environmental contacts.

---

### Contact-consistency constraint

The equality
$$
\mathbf{J}_c \dot{\mathbf{v}} + \dot{\mathbf{J}}_c \mathbf{v} = \mathbf{0}
$$
enforces rigid sticking contact.

If a foot is assumed firmly planted on the ground, then its contact point should have zero acceleration relative to the environment. This equation is simply the acceleration-level version of that assumption.

It plays two roles:
- it prevents the optimizer from generating accelerations that would break the modeled contact,
- it links body motion to the contact-force solution.

For example, in quiet standing with both feet flat on the ground, the contact Jacobian corresponds to both stance feet, and this equality says that the stance feet are not allowed to move. In single support, only the stance foot appears in $\mathbf{J}_c$, while the swing foot is removed from the contact set and becomes a tracking task instead.

When contacts change, such as during walking, the set of active constraints changes, so both $\mathbf{J}_c$ and the feasible contact-force space change as well.

---

### Contact-force inequalities

The inequality
$$
\mathbf{A}_{\lambda} \boldsymbol{\lambda} \le \mathbf{b}_{\lambda}
$$
collects contact-force feasibility constraints.

These constraints typically encode:
- unilateral contact: normal force must be nonnegative,
- friction-cone or friction-pyramid constraints,
- center-of-pressure bounds inside the foot support region,
- torsional moment bounds,
- contact wrench limits.

For a simple point contact with linearized friction, one may impose
$$
\lambda_n \ge 0, \qquad
|\lambda_t^{(x)}| \le \mu \lambda_n, \qquad
|\lambda_t^{(y)}| \le \mu \lambda_n.
$$
For flat-foot contact, the constraints are often written at the wrench level, for example bounding the center of pressure inside the foot sole and limiting yaw torque about the contact normal.

These inequalities are essential because a dynamics equation can always be balanced by mathematically convenient but physically impossible contact forces. The friction and wrench constraints prevent the optimizer from using such unrealizable support forces.

---

### Torque and acceleration bounds

The bounds
$$
\boldsymbol{\tau}_{\min} \le \boldsymbol{\tau} \le \boldsymbol{\tau}_{\max}
$$
reflect motor limits. They encode what the actuators can really produce.

The bounds
$$
\dot{\mathbf{v}}_{\min} \le \dot{\mathbf{v}} \le \dot{\mathbf{v}}_{\max}
$$
are sometimes used for numerical conditioning, safety, or conservative dynamic feasibility. They can help prevent unrealistically large accelerations that may arise from tracking conflicts or poorly tuned weights.

In some implementations, these acceleration bounds are replaced or augmented by joint-acceleration limits, jerk limits, or velocity-dependent saturation rules.

---

### Why this becomes a quadratic program

If all task costs are quadratic and all constraints are linear in the optimization variables, then the WBC problem is a quadratic program:
$$
\min_{\mathbf{z}} \quad \frac{1}{2}\mathbf{z}^\top \mathbf{H}\mathbf{z} + \mathbf{g}^\top \mathbf{z}
\quad
\text{subject to}
\quad
\mathbf{C}\mathbf{z} = \mathbf{d}, \qquad
\mathbf{E}\mathbf{z} \le \mathbf{f},
$$
with
$$
\mathbf{z} =
\begin{bmatrix}
\dot{\mathbf{v}} \\
\boldsymbol{\tau} \\
\boldsymbol{\lambda}
\end{bmatrix}.
$$

This is one of the reasons QP-based WBC became so dominant in humanoid robotics: it is expressive enough to encode many tasks and physical constraints, while remaining computationally tractable at servo rates.

---

### Interpretation of weighted tasks

The weighted formulation above is a soft-priority formulation. It tries to satisfy all tasks as well as possible, but if they conflict, the optimizer resolves the conflict according to the weights.

This differs from strict hierarchical WBC, where tasks are ordered lexicographically:
1. hard physical constraints,
2. highest-priority motion task,
3. lower-priority tasks in the null space of higher-priority ones.

The weighted version is easier to implement and often works well in practice, but it does not provide absolute task priority guarantees. If a strict hierarchy is required, one may solve a sequence of QPs or use null-space projection methods. In many humanoid systems, physical feasibility constraints are always hard, while motion objectives are soft.

---

### Examples of tasks in common humanoid behaviors

#### Standing

In quiet standing, the active contacts are usually both feet. The controller may include:
- center-of-mass regulation,
- pelvis orientation stabilization,
- upper-body posture regularization,
- equal or desired force sharing between left and right feet.

In this case, $\mathbf{J}_c$ corresponds to the stance-foot constraints, and $\boldsymbol{\lambda}$ represents the ground reaction wrenches. The optimizer chooses torques and support forces that keep the body upright without violating friction or center-of-pressure constraints.

#### Walking

During walking, the task set changes across phases.
- In single support, the stance foot is constrained, while the swing foot is a motion task.
- In double support, both feet appear in $\mathbf{J}_c$, and the optimizer distributes the load between them.

Typical tasks include:
- center-of-mass or momentum tracking,
- swing-foot pose tracking,
- torso stabilization,
- step-timing-consistent contact transitions.

The same optimization structure can be reused, but the contact set and task references are phase-dependent.

#### Handling an object while balancing

Suppose the humanoid holds a box while standing or walking slowly. Then the arm motion, the object-induced wrench, and the balance objective must be resolved together. One may add hand or object pose tasks while the dynamics term naturally captures the changed loading through $\mathbf{h}$ or through explicit external-wrench modeling. The contact-force optimization becomes especially important because the ground reaction forces must compensate not only for body motion but also for the object load.

---

### Internal forces and force distribution

In multi-contact settings, the net effect of contact forces on the robot may be the same for multiple different values of $\boldsymbol{\lambda}$. Some of these differences correspond to internal forces: force components that redistribute load among contacts without changing the net external wrench on the body.

This is why regularization or explicit force objectives are useful. Without them, the QP may choose a mathematically valid but physically undesirable force distribution, such as overloading one foot or producing excessive hand contact force. In more advanced WBC formulations, internal-force control appears explicitly as an additional task or constraint.

---

### Practical remarks

Several implementation points are important in practice.

First, the task matrices and bias terms must be updated every servo cycle from the current state estimate. This means $\mathbf{M}$, $\mathbf{h}$, $\mathbf{J}_j$, $\mathbf{J}_c$, and $\dot{\mathbf{J}}_c \mathbf{v}$ are all recomputed online.

Second, the quality of the controller depends strongly on the contact model. If the controller assumes rigid sticking contact but the real foot is slipping or rocking, then the optimization is solving the wrong problem. This is one reason contact estimation and state estimation are tightly coupled to WBC in real humanoid systems.

Third, the tuning of weights matters. If a posture task is weighted too strongly relative to a balance task, the optimizer may sacrifice dynamic robustness for configuration tracking. If the regularization on torque is too strong, the robot may become sluggish. If the force penalty is too weak, contact forces may become unnecessarily aggressive.

Finally, WBC does not replace motion generation. In many systems, a higher-level planner or MPC provides desired center-of-mass, momentum, footstep, or hand trajectories, and the WBC solves the instantaneous constrained inverse-dynamics problem needed to realize those references.

---

### Conceptual summary

The optimization form of WBC can be interpreted as a real-time reconciliation of three requirements:
1. realize multiple whole-body motion objectives,
2. remain exactly consistent with rigid-body dynamics and active contacts,
3. respect actuation and contact feasibility.

The result is not just a motion command, but a dynamically consistent whole-body behavior together with the corresponding torque and contact-force allocation. This is precisely why WBC is such a natural formulation for humanoid robots: it treats balance, contact, posture, manipulation, and actuation as parts of one coupled optimization problem.




## 7.4 Prioritized vs weighted formulations

Two styles dominate:

### Prioritized control
Tasks are solved in strict order. Higher-priority tasks are never compromised for lower-priority tasks. This mirrors the logic:
1. satisfy physics and contact;
2. keep the robot balanced;
3. place the swing foot;
4. regulate posture.

### Weighted control
All tasks enter one optimization with weights. This is computationally simple and flexible, but the practical behavior depends on careful tuning.

Strict priority is conceptually appealing; weighted QPs are often easier to implement robustly. Many systems mix both styles.

## 7.5 Internal forces in multi-contact

When multiple contacts exist, there may be many contact-force distributions that produce the same net wrench on the robot. The difference between two such distributions is an **internal force pattern**: it changes load inside the contact network without changing the net motion.

Examples:
- both hands pushing in opposite directions against a wall and object;
- two feet sharing load differently while total support remains unchanged.

Regulating internal forces matters for:
- avoiding local slip;
- preventing overloading of a weak contact;
- maintaining grasp or brace stability.

## 7.6 Contact transitions in WBC

A well-designed WBC should not merely satisfy fixed constraints. It should handle contact transitions gracefully:
- before touchdown, the swing foot is unconstrained;
- at touchdown, impact and force ramping must be handled;
- after touchdown, the foot becomes a support constraint;
- before liftoff, normal force should reduce smoothly to zero.

In practice, phase-dependent weighting and force ramps are often essential.

---

# 8. Impedance, admittance, and force regulation

## 8.1 Why compliance matters

Rigid position control is dangerous and brittle in contact-rich humanoid tasks. If the world is not modeled perfectly, aggressive position tracking can produce:
- force spikes;
- chatter at contact;
- unstable pushing;
- poor human-robot interaction safety;
- failures during touchdown.

Compliance introduces a controlled relation between motion error and interaction force.

## 8.2 Task-space impedance control

A standard impedance law in task space is
$$
\mathbf{F}_d =
\mathbf{K}_p (\mathbf{x}_d - \mathbf{x})
+
\mathbf{K}_d (\dot{\mathbf{x}}_d - \dot{\mathbf{x}})
+
\mathbf{M}_d (\ddot{\mathbf{x}}_d - \ddot{\mathbf{x}}).
$$

The robot is made to behave like a mass-spring-damper in the task coordinates. In humanoids, impedance is often used for:
- hand interaction with objects;
- swing-foot landing softness;
- upper-body compliance during disturbance rejection;
- low-level behavior inside a whole-body controller.

## 8.3 Admittance control

Admittance is dual to impedance. Instead of mapping motion error to force, it maps measured force to commanded motion. A simple form is
$$
\mathbf{M}_a \ddot{\mathbf{x}} + \mathbf{D}_a \dot{\mathbf{x}} + \mathbf{K}_a (\mathbf{x} - \mathbf{x}_0)
=
\mathbf{F}_{\mathrm{ext}}.
$$

Admittance is useful when:
- the robot has accurate force sensing but not perfect torque control;
- the controller should adapt motion to external disturbances;
- large manipulator compliance is needed on top of stiffer low-level loops.

## 8.4 Hybrid force-motion control

When a task contains constrained and unconstrained directions simultaneously, it is often useful to regulate force in one subspace and motion in the complementary subspace.

For example, if a hand pushes on a wall:
- along the wall normal, regulate contact force;
- along the wall tangent, regulate position or sliding motion.

This can be written using projectors or explicit task decompositions. In practice, modern whole-body optimization often absorbs this idea into different equality or cost terms for different axes.

## 8.5 Passivity and robustness

A key theoretical theme behind impedance-like control is passivity. While full passivity proofs for whole humanoids can be subtle, the broad insight remains valuable:

> compliant interaction is often more robust because it stores and dissipates energy rather than aggressively injecting it into uncertain contact dynamics.

This is especially important in human-facing or poorly modeled environments.

---

# 9. Model predictive control for humanoids

## 9.1 Why MPC is attractive

Model predictive control (MPC) is attractive because it:
- optimizes behavior over a finite horizon;
- handles state and input constraints explicitly;
- can include contact-force or wrench feasibility;
- naturally supports step adjustment and receding-horizon replanning.

For humanoids, MPC often serves as the predictive layer that reasons about future balance and motion.

## 9.2 Generic finite-horizon formulation

A central mathematical view of model predictive control is that, at each control update, the controller solves a finite-horizon optimal control problem over a future time window. The goal is not merely to compute an instantaneous control action, but to predict how the robot state, control inputs, and possibly contact interactions will evolve over the next horizon, and then select the evolution that best satisfies the task while respecting the robot dynamics and physical constraints.

A generic continuous-time optimal control problem can be written as

$$
\min_{\mathbf{x}(\cdot), \mathbf{u}(\cdot), \boldsymbol{\lambda}(\cdot)}
\int_{t_0}^{t_0+T}
\ell(\mathbf{x}(t), \mathbf{u}(t), \boldsymbol{\lambda}(t)) \, dt
+
\ell_f(\mathbf{x}(t_0+T))
$$
subject to
$$
\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}),
$$
$$
\mathbf{h}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) = \mathbf{0},
$$
$$
\mathbf{g}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) \le \mathbf{0}.
$$

This compact expression is intentionally broad. It can represent many forms of humanoid MPC, ranging from simple linear inverted pendulum models to centroidal dynamics MPC, single-rigid-body models, or full nonlinear floating-base optimal control. The exact meaning of the variables and constraints depends on the chosen model and the task.

---

### Decision variables and their meaning

The optimization is performed over entire trajectories on the interval $[t_0, t_0+T]$, not just over values at a single instant.

The state trajectory
$$
\mathbf{x}(\cdot)
$$
represents the predicted evolution of the robot state. Depending on the model, $\mathbf{x}$ may include:
- center-of-mass position and velocity,
- linear and angular momentum,
- floating-base pose and twist,
- joint positions and velocities,
- contact states or phase variables,
- object states if the robot is manipulating a load.

The control trajectory
$$
\mathbf{u}(\cdot)
$$
contains the commanded inputs. Depending on the control architecture, these may be:
- joint torques,
- desired contact wrenches,
- desired momentum rates,
- footstep or ZMP-related references,
- reduced-order control inputs such as center-of-pressure or virtual forces.

The contact-force trajectory
$$
\boldsymbol{\lambda}(\cdot)
$$
represents the contact reactions with the environment. In some formulations this is an explicit optimization variable, while in others it is eliminated implicitly or does not appear because the chosen model has already reduced the contact mechanics into a simpler control input.

For humanoids, including $\boldsymbol{\lambda}$ explicitly is often useful because balancing, walking, and loco-manipulation are strongly shaped by the feasibility of ground reaction forces and other contact wrenches.

---

### Running cost and terminal cost

The objective consists of two parts:

$$
\int_{t_0}^{t_0+T}
\ell(\mathbf{x}(t), \mathbf{u}(t), \boldsymbol{\lambda}(t)) \, dt
$$

and

$$
\ell_f(\mathbf{x}(t_0+T)).
$$

The first is the **running cost**, which penalizes undesirable behavior throughout the horizon. The second is the **terminal cost**, which penalizes undesirable terminal conditions at the end of the horizon.

The running cost often contains terms such as:
- tracking error relative to a reference trajectory,
- control effort,
- rate of change of control for smoothness,
- deviation from desired contact-force distribution,
- momentum regulation error,
- orientation stabilization error,
- violation penalties for soft constraints.

A typical quadratic running cost might look like
$$
\ell(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda})
=
(\mathbf{x}-\mathbf{x}_{\mathrm{ref}})^\top \mathbf{Q} (\mathbf{x}-\mathbf{x}_{\mathrm{ref}})
+
\mathbf{u}^\top \mathbf{R} \mathbf{u}
+
\boldsymbol{\lambda}^\top \mathbf{W}_{\lambda} \boldsymbol{\lambda}.
$$

The terminal cost is often chosen as
$$
\ell_f(\mathbf{x}(t_0+T))
=
(\mathbf{x}(t_0+T)-\mathbf{x}_{\mathrm{ref},f})^\top
\mathbf{Q}_f
(\mathbf{x}(t_0+T)-\mathbf{x}_{\mathrm{ref},f}),
$$
which encourages the state at the end of the horizon to point toward a desirable future continuation. This is especially important because the horizon is finite: without a terminal cost or terminal condition, the optimizer may behave myopically and choose short-term actions that look favorable now but are poor for long-term stability.

---

### Dynamics constraint

The constraint
$$
\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda})
$$
encodes the predictive model.

This is the heart of MPC. The optimizer is not free to choose arbitrary future trajectories; every predicted state must be dynamically reachable from the current condition under the chosen inputs and contact forces.

In humanoid robotics, several classes of predictive models are common.

#### Reduced-order models
Examples include:
- Linear Inverted Pendulum Model (LIPM),
- Divergent Component of Motion (DCM) models,
- centroidal or momentum-based reduced models,
- single-rigid-body models.

These models are simpler and faster to optimize, making them attractive for real-time walking or balance control.

#### Full-order models
These may include:
- full floating-base rigid-body dynamics,
- joint-space dynamics,
- explicit contact-force coupling,
- nonlinear kinematics and actuation limits.

These models are more accurate but far more computationally demanding.

The model $\mathbf{f}$ therefore reflects a design tradeoff between fidelity and real-time solvability.

---

### Equality constraints

The equality constraint
$$
\mathbf{h}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) = \mathbf{0}
$$
collects all exact algebraic relations that must hold.

Depending on the formulation, these may include:
- rigid contact constraints,
- kinematic closure constraints,
- stance-foot consistency constraints,
- centroidal momentum relations,
- load-distribution conditions,
- model-specific algebraic constraints.

For example, if a foot is assumed to remain fixed during a stance phase, the foot-contact acceleration may be constrained to zero. In a reduced-order centroidal formulation, equality constraints may relate the net contact wrench to momentum-rate variables.

Equality constraints are typically used when a condition must be satisfied exactly in the model, rather than merely encouraged through a penalty.

---

### Inequality constraints

The inequality constraint
$$
\mathbf{g}(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}) \le \mathbf{0}
$$
collects all physical, safety, and feasibility limits.

Typical examples include:
- torque bounds,
- joint position and velocity bounds,
- friction-cone or friction-pyramid constraints,
- unilateral contact conditions,
- center-of-pressure bounds inside the foot support region,
- step location limits,
- obstacle-avoidance margins,
- force or wrench limits during object handling,
- kinematic reachability constraints.

These inequalities are crucial in humanoid control because many motions that look dynamically valid in the unconstrained equations are physically unrealizable once friction, actuation, and contact geometry are taken into account.

For instance, a balance-recovery action may be dynamically plausible, but impossible if it requires a ground reaction force outside the friction cone or a center of pressure outside the foot.

---

### Why the horizon is finite

The parameter $T$ is the prediction horizon. It defines how far into the future the controller reasons.

A short horizon gives:
- lower computational cost,
- faster update rates,
- more reactive behavior,
- but weaker long-term foresight.

A longer horizon gives:
- better anticipation of future constraints,
- improved multi-step planning behavior,
- better contact sequencing and load-shifting behavior,
- but higher computational burden.

For humanoids, the choice of horizon is task-dependent. Quiet standing or disturbance rejection may use short horizons, whereas walking pattern generation, payload handling, or multi-step balance recovery may benefit from longer predictive windows.

Because MPC solves the problem repeatedly in a receding-horizon fashion, the finite horizon is not a limitation in the same way as open-loop finite-horizon planning. After solving once, only the first portion of the optimized action is applied. Then the horizon is shifted forward, the state is re-estimated, and the problem is solved again.

---

### Receding-horizon interpretation

The finite-horizon problem should not be interpreted as a one-shot optimization over the full future execution. Instead, the controller repeatedly performs the following cycle:

1. measure or estimate the current state at time $t_0$,
2. solve the finite-horizon optimal control problem on $[t_0, t_0+T]$,
3. apply only the first control action or short initial segment,
4. advance to the next update time,
5. re-solve using the new measured state.

This repeated replanning is what makes MPC robust to disturbances, modeling error, contact uncertainty, and changes in task conditions.

For humanoids, this is especially important because contact transitions, external pushes, payload variation, and sensing noise can all invalidate long open-loop plans. MPC restores feedback by repeatedly embedding the predictive optimization inside a closed-loop controller.

---

### Discretization and numerical form

To solve the problem numerically, the continuous-time trajectories are discretized over $N$ stages with time step $\Delta t$, so that
$$
T = N \Delta t.
$$

The state, control, and contact trajectories become sequences:
$$
\mathbf{x}_0, \mathbf{x}_1, \dots, \mathbf{x}_N,
$$
$$
\mathbf{u}_0, \mathbf{u}_1, \dots, \mathbf{u}_{N-1},
$$
$$
\boldsymbol{\lambda}_0, \boldsymbol{\lambda}_1, \dots, \boldsymbol{\lambda}_{N-1}.
$$

A common discrete-time approximation is
$$
\mathbf{x}_{k+1} = \mathbf{f}_d(\mathbf{x}_k, \mathbf{u}_k, \boldsymbol{\lambda}_k),
$$
where $\mathbf{f}_d$ is obtained from Euler integration, Runge--Kutta integration, collocation, or another discretization scheme.

The continuous problem then becomes a finite-dimensional optimization problem:
$$
\min_{\{\mathbf{x}_k,\mathbf{u}_k,\boldsymbol{\lambda}_k\}}
\sum_{k=0}^{N-1}
\ell_d(\mathbf{x}_k, \mathbf{u}_k, \boldsymbol{\lambda}_k)
+
\ell_f(\mathbf{x}_N)
$$
subject to
$$
\mathbf{x}_{k+1} = \mathbf{f}_d(\mathbf{x}_k, \mathbf{u}_k, \boldsymbol{\lambda}_k),
$$
$$
\mathbf{h}_d(\mathbf{x}_k, \mathbf{u}_k, \boldsymbol{\lambda}_k)=\mathbf{0},
$$
$$
\mathbf{g}_d(\mathbf{x}_k, \mathbf{u}_k, \boldsymbol{\lambda}_k) \le \mathbf{0},
$$
for $k=0,\dots,N-1$.

If the model is linear and the costs are quadratic, the result is typically a **quadratic program (QP)**. If the model or constraints are nonlinear, the result becomes a **nonlinear program (NLP)**. This is why humanoid MPC spans a broad family of solvers, from fast linear MPC QPs to nonlinear trajectory optimization.

---

### Linear MPC as a special case

If the dynamics are linearized as
$$
\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k + \mathbf{D}_k \boldsymbol{\lambda}_k + \mathbf{d}_k,
$$
and the cost is quadratic, then the optimization becomes
$$
\min
\sum_{k=0}^{N-1}
\left[
(\mathbf{x}_k-\mathbf{x}_{k,\mathrm{ref}})^\top \mathbf{Q}_k (\mathbf{x}_k-\mathbf{x}_{k,\mathrm{ref}})
+
\mathbf{u}_k^\top \mathbf{R}_k \mathbf{u}_k
+
\boldsymbol{\lambda}_k^\top \mathbf{W}_{\lambda,k} \boldsymbol{\lambda}_k
\right]
+
(\mathbf{x}_N-\mathbf{x}_{N,\mathrm{ref}})^\top \mathbf{Q}_f (\mathbf{x}_N-\mathbf{x}_{N,\mathrm{ref}}).
$$

This form is particularly attractive in humanoid control because it can often be solved at high rate. LIPM-based ZMP MPC and many centroidal MPC implementations are derived from this idea.

---

### Interpretation in humanoid applications

The finite-horizon formulation can be interpreted concretely in common humanoid behaviors.

#### Standing and push recovery
The state may include center-of-mass position, velocity, and body orientation. The control may represent desired support forces or momentum-rate commands. The cost penalizes deviation from upright balance and excessive actuation. The constraints ensure that the predicted support forces remain inside feasible contact limits.

#### Walking
The horizon may cover one or more upcoming steps. The optimization chooses future load shifts, body motion, and possibly footstep-related quantities so that the robot remains dynamically stable while progressing toward a walking objective. Contact constraints become phase-dependent and may change across single-support and double-support intervals.

#### Handling an object while balancing
The state may include additional load effects or object motion variables. The control and contact-force trajectory must account for both balance and manipulation demands. The cost may penalize deviation from a desired object path while also regulating whole-body momentum. The constraints encode both robot feasibility and contact stability with the environment.

In all of these cases, the same mathematical idea is reused: predict future motion, optimize it under a finite horizon, apply the first action, and repeat.

---

### Relation to WBC

MPC and WBC often operate at different levels of the control hierarchy.

MPC decides what the robot should do over the near future. It reasons predictively over time and produces reference quantities such as:
- desired center-of-mass motion,
- desired momentum evolution,
- desired contact wrenches,
- desired footstep timing or placement,
- desired body orientation or object trajectory.

WBC then realizes these references instantaneously while enforcing full-body dynamics and contact consistency at the current control cycle.

In that sense, MPC is the predictive planner over a short horizon, while WBC is the instantaneous constrained inverse-dynamics realization layer.

---

### Why this formulation matters

The generic finite-horizon formulation is important because it reveals what makes MPC different from ordinary feedback laws.

A conventional feedback controller maps the current state directly to a control action. By contrast, MPC solves for a future trajectory subject to dynamics and constraints, and only then extracts the current action from that future plan.

For humanoids, this distinction is critical because balance, walking, and loco-manipulation are fundamentally constrained prediction problems. The controller must think ahead about:
- where the center of mass is moving,
- how support forces will evolve,
- whether contact forces remain feasible,
- whether the robot can slow down or redirect before violating constraints,
- how current actions affect future stance transitions and manipulation stability.

The finite-horizon optimal control form is therefore not merely a mathematical abstraction; it is the structure that allows humanoid control to combine dynamics, contact feasibility, anticipation, and feedback in a single framework.

---

### Conceptual summary

The finite-horizon formulation can be summarized as follows:
- choose a future state trajectory,
- choose future control inputs,
- optionally choose future contact forces,
- minimize tracking, smoothness, effort, or stability-related costs,
- enforce dynamics, contact conditions, and feasibility constraints,
- apply only the first part of the resulting plan,
- repeat the optimization at the next control update.

This repeated finite-horizon optimization is the core principle behind model predictive control in humanoid robotics.

## 9.3 Choice of model in humanoid MPC

Different models lead to different controller classes.

### (a) LIPM-based MPC
Fast and simple; good for gait generation on relatively regular terrain.

### (b) Centroidal-dynamics MPC
Richer global physics; common in modern locomotion and loco-manipulation.

### (c) Full-body MPC
Most expressive but much more computationally demanding.

In practice, centroidal MPC offers a strong compromise between fidelity and real-time speed.

## 9.4 Typical decision variables and constraints

For centroidal MPC, common decision variables include:
- CoM trajectory $\mathbf{c}_k$;
- momentum or momentum-rate trajectory;
- contact forces $\mathbf{f}_{i,k}$;
- sometimes footstep locations or step timings.

Typical constraints include:
- centroidal dynamics;
- friction-cone approximations;
- contact activation schedules;
- support or reachability limits;
- terminal balance conditions;
- step adjustment bounds.

## 9.5 Receding horizon and closed-loop execution

In MPC, only the first control action or first portion of the plan is executed. Then the optimization is solved again at the next time step using updated state estimates. This repeated replanning provides robustness to disturbances and model mismatch.

For humanoids, this matters because:
- contact conditions may change unexpectedly;
- footsteps may need adjustment;
- manipulation loads may vary;
- state estimation is never perfect.

## 9.6 Nonlinear vs linearized MPC

### Linear MPC
Fast and often sufficient for simpler locomotion tasks.

### Nonlinear MPC
More expressive and better for payloads, aggressive motion, or varying contact geometry, but computationally heavier.

A useful teaching point is that controller design is often driven not only by theoretical elegance but by the real-time budget of the target platform.

## 9.7 MPC outputs and interfaces

An MPC may output:
- desired CoM trajectory;
- desired ZMP or center-of-pressure trajectory;
- desired contact forces;
- desired base or torso orientation;
- step locations and timings.

These outputs are rarely sent directly to motors. They are usually consumed by a WBC or inverse-dynamics layer that realizes them with the full robot.

---

# 10. Control of locomotion, manipulation, and loco-manipulation

## 10.1 Locomotion control

In locomotion, control must coordinate:
- stance-foot support constraints;
- CoM or momentum regulation;
- swing-foot placement;
- landing transitions;
- posture and arm motion.

A common structure is:
1. MPC computes CoM / momentum / footstep references.
2. WBC tracks swing-foot, torso, and force objectives while enforcing stance constraints.
3. Joint-level loops execute the resulting torques or accelerations.

The most fragile parts are almost always contact transitions and state estimation during dynamic motion.

## 10.2 Manipulation with a humanoid upper body

Humanoid manipulation differs from fixed-base manipulation because upper-body interaction forces affect whole-body balance. Even a hand task performed at a table can shift the load distribution at the feet.

Therefore manipulation control on a humanoid often requires:
- explicit balance maintenance;
- force-aware interaction;
- posture shaping for reachability and torque margin;
- possibly hand-force tasks coupled with foot support management.

This is why humanoid manipulation is better viewed as a whole-body problem rather than as arm control on top of a stationary base.

## 10.3 Loco-manipulation

Loco-manipulation couples locomotion and manipulation in the strongest possible sense. Examples include:
- opening a heavy door while stepping;
- carrying a box while walking;
- pushing a cart;
- climbing stairs with handrail support.

In these tasks the controller must coordinate not just body motion and external contacts independently, but the **interaction dynamics of the manipulated object** and the robot.

This makes centroidal or whole-body inverse-dynamics formulations especially valuable, because they naturally represent how object reaction forces enter the balance problem.

## 10.4 Multi-contact control

With multi-contact, the controller must manage:
- multiple support constraints;
- load sharing across contacts;
- contact activation and deactivation;
- internal force regulation;
- possibly non-coplanar support geometry.

A major control challenge is the tension between expressiveness and computational tractability. Richer contact models help, but they also increase problem size and solver complexity.

## 10.5 Contact scheduling and phase logic

Even when control is optimization-based, humanoid systems often still need phase logic:
- left stance / right swing;
- double support;
- transition windows;
- hand support engaged or disengaged.

Purely continuous optimization does not automatically eliminate the need for these mode decisions. For a teaching lecture, this is worth emphasizing because students often overestimate what a continuous QP can do by itself.

---

# 11. From theory to implementation

## 11.1 Control rates and real-time budgets

A practical humanoid controller usually runs at multiple rates:
- state estimation and contact estimation at IMU / encoder rates;
- WBC or inverse dynamics at around 200 Hz to 1 kHz depending on hardware;
- MPC at lower rates such as 20 Hz to 100 Hz depending on the model and solver.

The implementation question is not only “is the method correct?” but also “can it close the loop in time?”

## 11.2 Torque, acceleration, and position interfaces

A humanoid platform may expose different control interfaces:
- torque control;
- current control;
- velocity control;
- position control with internal motor loops.

The chosen outer-loop design must respect the real hardware interface. A beautifully derived inverse-dynamics controller loses meaning if the platform only accepts stiff position commands. In such cases, one may implement outer-loop admittance or reference shaping instead of direct torque realization.

## 11.3 The estimation–control interface

Control quality depends critically on estimation quality. A humanoid controller typically needs:
- base orientation and angular velocity from IMU fusion;
- base position / velocity from contact-aided state estimation;
- joint positions and velocities;
- contact states;
- sometimes force/torque estimates.

If the controller assumes a foot is rigidly fixed while the estimator believes it is slipping, or vice versa, the resulting inconsistency can destabilize the entire system.

## 11.4 Contact modeling choices in simulation

In simulation, contact regularization, friction models, and solver settings strongly affect closed-loop behavior.

Typical pitfalls include:
- using overly stiff contacts that require tiny integration steps;
- unrealistic friction coefficients;
- mismatch between simulated actuator bandwidth and real motors;
- hidden damping or stabilization terms in the simulator;
- overtrusting perfect ground truth state estimates.

A strong implementation practice is to test control logic under mild model perturbations from the beginning.

## 11.5 Debugging strategy

When a humanoid controller fails, debug in layers:

1. **Check kinematics first**  
   Are task Jacobians, frames, and sign conventions correct?

2. **Check dynamics next**  
   Does inverse dynamics reproduce expected gravity compensation and simple contact equilibria?

3. **Check contact logic**  
   Are contact activations and deactivations synchronized with the controller assumptions?

4. **Check optimization conditioning**  
   Are tasks mutually feasible? Are weights or priorities wildly ill-scaled?

5. **Check low-level actuation mismatch**  
   Are the commanded torques or motions actually realizable by the hardware interface?

This discipline prevents students from blaming “control theory” for what is often an implementation inconsistency.

## 11.6 A minimal experimental progression

A sensible progression for students is:
1. fixed-base task-space controller in simulation;
2. floating-base gravity compensation in double support;
3. CoM regulation with fixed contacts;
4. swing-foot tracking in single support;
5. WBC with friction constraints;
6. centroidal or LIPM MPC feeding the WBC;
7. disturbance rejection and step adjustment;
8. a simple loco-manipulation task such as pushing or carrying.

This progression mirrors how many research systems are actually built.

---

# 12. Research directions and open problems

## 12.1 Real-time control with richer contact models

Rigid point or planar contact abstractions are useful but incomplete. Future humanoid control needs better integration of:
- distributed contacts;
- soft contact;
- foot compliance;
- hand-object interaction geometry;
- contact uncertainty.

The challenge is to gain realism without losing real-time performance.

## 12.2 Unifying manipulation and locomotion control

Many systems still treat locomotion and manipulation as separate modules with weak coupling. A major research direction is more unified control in which object interaction, whole-body momentum, and support management are optimized together.

## 12.3 Control under uncertainty

Humanoid control still struggles with:
- uncertain terrain;
- variable payloads;
- sensor delays;
- contact estimation errors;
- actuator saturation and thermal limits.

Robust and stochastic formulations are promising, but their computational cost remains a challenge.

## 12.4 Better exploitation of tactile and force sensing

Humanoid control will benefit from more direct use of tactile, force, and distributed-contact measurements, especially for:
- foothold quality assessment;
- hand support regulation;
- collision-tolerant navigation;
- whole-body manipulation.

This should not be treated as an add-on sensing topic; it changes what the controller can regulate in real time.

## 12.5 Hardware-aware control

A persistent gap remains between elegant control laws and the messy reality of:
- gearbox friction;
- backlash;
- compliance;
- limited torque bandwidth;
- thermal and power limits.

Future control methods must be more explicitly hardware-aware if they are to scale to large humanoid platforms.

---

# 13. Summary of key takeaways

1. Humanoid control is fundamentally a **floating-base, underactuated, constrained** control problem.

2. Contacts are not merely disturbances. They are **control resources** that determine what whole-body motion is feasible.

3. Task-space reasoning is essential, but task-space feedback alone is insufficient without dynamics and contact consistency.

4. Reduced-order balance models such as the LIPM and centroidal dynamics are indispensable because they expose the global structure of balance and enable fast prediction.

5. Whole-body control provides the local mechanism to coordinate motion, force, and contact across the full robot.

6. Impedance, admittance, and force regulation remain essential for robust physical interaction.

7. MPC is powerful because it reasons over the future while handling constraints, but it almost always needs a fast local realization layer.

8. Successful humanoid control depends as much on architecture, estimation, contact logic, and implementation discipline as on the equations themselves.

---

# 14. Suggested implementation roadmap for students

## Stage 1: Single-task control foundations
- Implement gravity compensation for a fixed-base manipulator.
- Add task-space PD for an end-effector.
- Verify Jacobians and frame conventions.

## Stage 2: Floating-base fundamentals
- Build a floating-base humanoid model.
- Verify equations of motion and contact Jacobians.
- Simulate double-support equilibrium.

## Stage 3: Basic balance control
- Implement CoM control with fixed support contacts.
- Add torso orientation stabilization.
- Observe the role of contact-force feasibility.

## Stage 4: Whole-body task coordination
- Add swing-foot tracking during single support.
- Formulate a QP-based WBC with contact constraints.
- Include torque limits and friction-cone approximations.

## Stage 5: Compliant interaction
- Add task-space impedance for hand or foot interaction.
- Study touchdown and push-recovery behavior.

## Stage 6: Predictive control
- Implement LIPM or centroidal MPC.
- Feed its output into the WBC.
- Add step adjustment or disturbance rejection.

## Stage 7: Loco-manipulation
- Introduce an object interaction force into the whole-body controller.
- Study tasks such as pushing a cart, carrying a box, or opening a door.

At each stage, evaluate not only tracking performance but also:
- contact stability;
- solver time;
- torque margin;
- robustness to state-estimation error;
- behavior under mild model mismatch.

---

# 15. References for further study

The following references are useful anchors for a graduate course on humanoid control. They mix classical foundations and representative modern works.

1. O. Khatib, “A unified approach for motion and force control of robot manipulators: The operational space formulation,” *IEEE Journal on Robotics and Automation*, 1987.  
2. B. Siciliano and J.-J. E. Slotine, “A general framework for managing multiple tasks in highly redundant robotic systems,” *ICAR*, 1991.  
3. L. Sentis and O. Khatib, “Synthesis of whole-body behaviors through hierarchical control of behavioral primitives,” *International Journal of Humanoid Robotics*, 2005.  
4. S. Kajita et al., “Biped walking pattern generation by using preview control of zero-moment point,” *ICRA*, 2003.  
5. J. Pratt, J. Carff, S. Drakunov, and A. Goswami, “Capture point: A step toward humanoid push recovery,” *Humanoids*, 2006.  
6. F. Kanehiro, M. Morisawa, W. Suleiman, K. Kaneko, and E. Yoshida, “Integrating geometric constraints into reactive leg motion generation,” *IROS*, 2008.  
7. N. Mansard, O. Stasse, P. Evrard, and A. Kheddar, “A versatile generalized inverted kinematics implementation for collaborative working humanoid robots,” *ICRA*, 2009.  
8. A. Herzog, N. Rotella, S. Mason, P. M. Wensing, and S. Schaal, “Momentum control with hierarchical inverse dynamics on a torque-controlled humanoid,” *Autonomous Robots*, 2016.  
9. P.-B. Wieber, “Trajectory free linear model predictive control for stable walking in the presence of strong perturbations,” *Humanoids*, 2006.  
10. T. Koolen et al., “Design of a momentum-based control framework and application to the humanoid robot Atlas,” *International Journal of Humanoid Robotics*, 2016.  
11. F. Farshidian, M. Neunert, and J. Buchli, “A fast nonlinear model predictive control framework for unified trajectory optimization and tracking,” *ICRA*, 2017.  
12. G. Nava, G. Romualdi, D. Pucci, and colleagues, representative works on nonlinear centroidal MPC for humanoid locomotion and payload carrying.  
13. J. Li and Q. Nguyen, representative works on force-and-moment-based MPC for highly dynamic biped locomotion.  
14. Z. Gu et al., “Humanoid locomotion and manipulation: Current progress and challenges in control, planning, and learning,” arXiv:2501.02116, 2025.  
15. S. Kajita, K. Kaneko, and the HRP series literature on ZMP-based walking and humanoid stabilization.  