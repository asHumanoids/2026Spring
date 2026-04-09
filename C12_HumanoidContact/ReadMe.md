# Class 12 on Humanoid Contact

## Contents

1. Motivation and scope
2. Mathematical preliminaries and notation
3. Contact mechanics for humanoid robots
4. Humanoid dynamics under contact
5. Balance and stability under contact
6. Contact sensing and state estimation
7. Planning with contact
8. Contact-aware control
9. From theory to implementation
10. Research directions and open problems
11. Summary of key takeaways
12. Suggested implementation roadmap for students
13. References for further study


# 1. Motivation and scope

## 1.1 Why contact is central to humanoid robots

Humanoid robots are contact-rich systems by construction. Unlike a fixed-base manipulator bolted to the world, the base of a humanoid is free to move and is not directly actuated with respect to the environment. This simple fact changes almost everything. The robot can only support itself, redirect momentum, and manipulate the world by generating contact forces through its limbs. In that sense, contact is not an accessory to humanoid behavior; it is the mechanical channel through which behavior becomes physically realizable.

Consider a few representative tasks:

- standing in place requires the feet to generate support forces and moments that counter gravity and reject small disturbances;
- walking requires repeated making and breaking of foot contacts while maintaining dynamic feasibility;
- climbing stairs often requires non-coplanar contacts and, in practice, may benefit from handrail support;
- opening a heavy door while walking is a loco-manipulation problem in which the robot must coordinate foot contacts, hand contact, body momentum, and object reaction forces;
- recovery from a push may require an additional step, a hand support, or rapid redistribution of contact forces.

The common element is not merely motion. It is **structured interaction** with the environment.

## 1.2 Contact as both opportunity and constraint

Contact plays a dual role.

On one hand, it is an **opportunity**. Contact gives the robot leverage. It allows environmental reaction forces to support weight, produce acceleration of the center of mass (CoM), regulate angular momentum, and stabilize manipulation.

On the other hand, it is a **constraint**. Contacts are typically unilateral, so the robot can push on the ground but cannot pull it. Friction is bounded, so some desired tangential forces are infeasible and lead to slip. Contact geometry matters, so point contacts, line contacts, and area contacts provide different wrench capabilities. Contact also creates hybrid behavior because contact modes switch discretely while the within-mode dynamics remain continuous.

The correct mental model is therefore not “the robot tracks a trajectory.” The better model is:

> the robot evolves through a sequence of contact modes, and within each mode it must satisfy the dynamics, the kinematics of sticking or slipping contacts, friction feasibility, actuator limits, and task objectives.

## 1.3 Scope of these notes

These notes focus on **rigid-body contact modeling for humanoids**, together with the sensing, estimation, and planning ideas needed to use those models in practice. The emphasis is on principles that remain useful across platforms:

- research humanoids in MuJoCo, Isaac Lab, or similar simulators;
- floating-base multibody models of bipeds and humanoids;
- humanoid systems that rely on contact sensing and contact-state estimation;
- planning problems involving footsteps, hand supports, and multi-contact loco-manipulation.

We do **not** attempt a complete treatment of complementarity theory, nonsmooth analysis, deformable contact mechanics, or tactile manipulation at the level of a dedicated dexterous manipulation course. We also intentionally omit detailed material on humanoid control and humanoid learning, since those are reserved for subsequent classes. The goal here is a unified working understanding that connects contact mechanics and constrained dynamics to estimation, tactile reasoning, contact planning, and implementation-oriented modeling.

# 2. Mathematical preliminaries and notation

## 2.1 Floating-base coordinates

A humanoid is modeled as a multibody system with a free-floating base and actuated joints. Let

$$
q = \begin{bmatrix} q_b \\ q_a \end{bmatrix},
$$

where $q_b$ denotes the floating-base configuration and $q_a \in \mathbb{R}^{n_a}$ denotes the actuated joint coordinates. In practice, $q_b$ may be represented by base position plus orientation, often using a quaternion or rotation matrix internally for numerical robustness.

A velocity representation is often more convenient than differentiating the configuration coordinates directly. Let

$$
v = \begin{bmatrix} v_b \\ \dot q_a \end{bmatrix},
$$

where $v_b \in \mathbb{R}^6$ is the spatial base velocity, containing linear and angular components.

The distinction between $q$ and $v$ matters because floating-base orientation is not always minimally represented in a globally smooth way. Most rigid-body algorithms therefore work with generalized configuration $q$ and generalized velocity $v$ rather than naively using $\dot q$ everywhere.

## 2.2 Actuation selection matrix

Because only the internal joints are actuated, the generalized torque vector enters through a selection matrix $S$:

$$
M(q)\dot v + h(q,v) = S^T \tau + \sum_i J_i(q)^T \lambda_i.
$$

Here:

- $M(q)$ is the generalized mass matrix;
- $h(q,v)$ collects Coriolis, centrifugal, gravitational, and sometimes passive terms;
- $\tau \in \mathbb{R}^{n_a}$ are joint torques;
- $J_i(q)$ is the Jacobian of contact $i$;
- $\lambda_i$ is the contact wrench or contact force parameter for that contact.

The presence of $S^T \tau$ rather than an arbitrary generalized force vector is the formal expression of **underactuation**. The robot cannot directly command the base wrench. It can only influence the base through internal actuation and external contacts.

## 2.3 Contact Jacobians

Let a contact point or contact frame be attached to a body of the robot. Its spatial velocity can be written as

$$
v_c = J_c(q) v.
$$

If the contact is assumed to stick, then the relative velocity with respect to the environment is constrained. For a stationary environment contact, the ideal sticking condition is

$$
J_c(q) v = 0.
$$

Differentiating yields the acceleration-level contact constraint:

$$
J_c(q) \dot v + \dot J_c(q,v) v = 0.
$$

This is the standard way contact enters inverse dynamics and whole-body control formulations.

## 2.4 A note on frames and wrenches

Humanoid contact calculations are full of frame conventions. Students commonly lose track of signs, moments, and Jacobians because frames are not handled consistently. A useful discipline is:

- always state in which frame a wrench is expressed;
- always state where the wrench is applied;
- use spatial vector notation consistently when possible;
- transform contact forces and moments before summing them.

For implementation, this matters more than elegance. A theoretically correct equation with inconsistent frames is a practical bug.

# 3. Contact mechanics for humanoid robots

## 3.1 Unilateral contact

Most environment contacts in humanoid locomotion are unilateral. The robot can push into the environment, but it cannot sustain tensile contact forces with the ground unless there is adhesion, suction, a hand grasp, or some other additional mechanism.

A common way to describe unilateral contact is with a gap function $\phi(q)$ satisfying

$$
\phi(q) \ge 0.
$$

Interpretation:

- $\phi(q) > 0$: separated bodies;
- $\phi(q) = 0$: touching contact surface;
- $\phi(q) < 0$: interpenetration, which is physically invalid in rigid contact modeling.

The corresponding normal force $\lambda_n$ must satisfy

$$
\lambda_n \ge 0.
$$

The familiar complementarity intuition is

$$
\phi(q)\,\lambda_n = 0,
$$

which encodes the idea that either the bodies are separated and the normal force vanishes, or they are in contact and a compressive force may appear.

This relation is conceptually important even if a given simulator or controller does not solve a strict complementarity problem.

## 3.2 Tangential friction and the Coulomb model

At a contact, the tangential force is limited by friction. In the classical isotropic Coulomb model,

$$
\|f_t\| \le \mu f_n,
$$

where $f_t$ is the tangential force magnitude, $f_n$ is the normal force magnitude, and $\mu$ is the friction coefficient.

This yields two regimes:

- **sticking**: the requested tangential force lies inside the admissible set;
- **slipping**: the tangential load exceeds the admissible set, and relative tangential motion occurs.

In three-dimensional contact, it is common to write

$$
\sqrt{f_x^2 + f_y^2} \le \mu f_z, \qquad f_z \ge 0.
$$

Geometrically, this defines the **friction cone**.

### Why friction matters so much in humanoids

Students often think of balance as a geometric problem: keep the CoM projection inside the support region. In reality, balance is equally a **force feasibility problem**. A humanoid may have a CoM projection that looks safe in a top-down plot, yet still fail because the required tangential forces exceed what the terrain friction allows.

This is especially important for:

- push recovery;
- turning and lateral stepping;
- hand-supported balancing;
- inclined terrain;
- contact transitions involving nonzero tangential impact or shear.

## 3.3 Friction cones versus friction pyramids

Optimization-based controllers often replace the circular friction cone by a polyhedral approximation. For example, one can approximate the cone by linear inequalities of the form

$$
|f_x| \le \mu' f_z, \qquad |f_y| \le \mu' f_z, \qquad f_z \ge 0,
$$

or a richer set of facet inequalities. This is called a **friction pyramid**.

The advantage is computational: linear inequalities fit naturally into quadratic programs (QPs) and linear model predictive control (MPC) formulations. The disadvantage is approximation error. In practice, the approximation is usually acceptable when the controller retains some safety margin.

## 3.4 Contact wrenches

A point force acting at contact point $p$ induces not only a force but also a moment about a reference point. The combined quantity is a **wrench**:

$$
w = \begin{bmatrix} f \\ \tau \end{bmatrix}, \qquad \tau = p \times f.
$$

For a finite foot contact, the actual pressure distribution over the sole is distributed, but one often models its net effect as a resultant wrench subject to feasibility constraints. This viewpoint is crucial because humanoid feet can sustain not only vertical support forces but also moments due to distributed pressure over a finite support area.

## 3.5 Contact geometry: point, line, and surface contact

Contact geometry determines what wrenches can be supported.

- **Point contact without friction** supports only a normal force.
- **Point contact with friction** supports a three-dimensional force but generally no free moment.
- **Surface contact** may support a richer wrench set, including bounded moments due to pressure distribution.

For humanoids, the foot is typically modeled as a finite contact patch on flat terrain, while a fingertip or knuckle contact may be approximated as a point contact with friction.

## 3.6 Impact and the hybrid nature of contact

Walking is not a single smooth dynamical system. It is a **hybrid system**. During stance, the foot may be treated as sticking. During swing, that contact disappears. At touchdown, there may be an impulsive event.

At impact, the generalized velocity can change abruptly. A standard impulsive model writes

$$
M(q)(v^+ - v^-) = J_c(q)^T \Lambda,
$$

where $v^-$ and $v^+$ are the pre- and post-impact generalized velocities and $\Lambda$ is the contact impulse.

For a perfectly plastic sticking impact, one additionally imposes

$$
J_c(q) v^+ = 0.
$$

Even when a practical controller does not explicitly solve impact equations online, the conceptual distinction between within-mode continuous dynamics and mode-switch events is essential.

# 4. Humanoid dynamics under contact

## 4.1 Floating-base equations of motion

The standard constrained rigid-body dynamics equation for a humanoid is

$$
M(q) \dot v + h(q,v) = S^T \tau + J_c(q)^T \lambda.
$$

This equation is the backbone of nearly every modern whole-body controller. It says that generalized acceleration is determined by inertia, bias forces, joint torques, and environmental contact forces.

Several immediate consequences follow.

First, because the base is unactuated, the robot cannot arbitrarily impose its full-body acceleration. It must remain consistent with the external contacts.

Second, the contact forces are not arbitrary either. They are unknown variables that must satisfy unilateral and frictional feasibility.

Third, contact changes the effective capabilities of the robot. A floating-base humanoid in midair is very different from the same robot in double support, and different again when one hand braces against a wall.

## 4.2 Acceleration-level contact constraints

Assume a contact is sticking. Then

$$
J_c(q) v = 0.
$$

Differentiating gives

$$
J_c(q) \dot v + \dot J_c(q,v) v = 0.
$$

The constrained dynamics problem is then to solve for $(\dot v, \lambda)$ given torques and contact mode.

One can write the coupled linear system

$$
\begin{bmatrix}
M & -J_c^T \\
J_c & 0
\end{bmatrix}
\begin{bmatrix}
\dot v \\
\lambda
\end{bmatrix}
=\begin{bmatrix}
S^T\tau - h \\
-\dot J_c v
\end{bmatrix}.
$$

This is the standard KKT-like system associated with rigid-body dynamics and Lagrange multipliers.

### Physical interpretation

The constraints do two things simultaneously:

1. they **remove motion** by preventing relative movement at the sticking contacts;
2. they **introduce forces** as Lagrange multipliers to enforce those constraints.

This is a recurring theme in contact mechanics: each active constraint contributes both a geometric restriction and a reaction force.

## 4.3 Contact force redundancy

In multi-contact settings, the contact force solution is often non-unique. For example, in double support the same net wrench on the center of mass can sometimes be achieved by many different left-foot/right-foot force distributions.

This redundancy is not a nuisance; it is a useful control degree of freedom. It allows the controller to:

- shift load between limbs;
- reduce peak forces at one contact;
- avoid slip by steering forces away from friction boundaries;
- regulate internal forces in bracing tasks.

However, it also means that a purely kinematic solution is insufficient. A controller must choose among many dynamically feasible contact force distributions.

## 4.4 Inverse dynamics perspective

In control, one often specifies desired tasks such as CoM acceleration, swing-foot acceleration, hand acceleration, or posture regulation. The inverse dynamics problem is then to compute torques and contact forces consistent with those tasks and the full-body constraints.

A generic optimization-based inverse dynamics problem may choose decision variables

$$
(\dot v, \tau, \lambda)
$$

and solve

$$
\min_{\dot v, \tau, \lambda} \; \text{task tracking error} + \text{regularization}
$$

subject to

$$
M\dot v + h = S^T\tau + J_c^T\lambda,
$$

$$
J_c\dot v + \dot J_c v = 0,
$$

plus torque limits and friction constraints.

This turns the contact problem into a structured numerical optimization problem rather than a purely symbolic mechanics problem.

## 4.5 Centroidal dynamics reduction

For planning and balance reasoning, the full robot dynamics are often reduced to **centroidal dynamics**. Let $c$ denote the CoM, $m$ the total mass, and $L_c$ the angular momentum about the CoM. Then

$$
m \ddot c = \sum_i f_i + mg,
$$

$$
\dot L_c = \sum_i (p_i - c) \times f_i + \sum_i \tau_i.
$$

These equations isolate the global effect of contact wrenches on the whole body. They ignore many internal details but preserve what matters for support, momentum, and balance.

A standard and very useful relationship is the centroidal momentum map

$$
h_G = A_G(q) v,
$$

where $h_G$ concatenates linear and angular momentum and $A_G(q)$ is the centroidal momentum matrix. This representation, emphasized in centroidal dynamics formulations, is a cornerstone for planning and whole-body control.

> **Research understanding.** In modern humanoid control, centroidal dynamics often plays the role that the full Euler–Lagrange model plays in classical robotics: it is the reduced, structured, control-oriented model that sits between raw rigid-body mechanics and high-level planning.

# 5. Balance and stability under contact

## 5.1 Static stability and the support polygon

The first balance concept students usually learn is the support polygon. If the robot is standing quasi-statically on a flat surface, a sufficient condition for static balance is that the projection of the CoM lies within the convex hull of the support contacts.

This criterion is useful pedagogically, but it is limited.

It assumes:

- flat, coplanar support contacts;
- quasi-static behavior;
- sufficiently high friction;
- no large external disturbances.

As soon as the humanoid moves dynamically, uses hand contacts, or stands on irregular terrain, the support polygon stops being a complete picture.

## 5.2 Center of pressure and zero moment point

For flat-foot locomotion, two classical notions are central.

### Center of pressure (CoP)

The CoP is the point on the support surface where the net pressure distribution is effectively applied. If the foot wrench is known, the CoP can be extracted from the moment components.

### Zero moment point (ZMP)

The ZMP is the point on the support plane where the net tipping moment has zero horizontal components. In flat-ground locomotion with moderate assumptions, CoP and ZMP are closely related and are frequently treated almost interchangeably in practice.

In the linear inverted pendulum model (LIPM), assuming constant CoM height and negligible angular momentum change, the relation becomes

$$
x_{\text{ZMP}} = x_c - \frac{z_c}{g} \ddot x_c,
$$

and similarly for the lateral direction.

This shows that ZMP is not simply a geometric location; it encodes a dynamic consistency relation between CoM position and acceleration.

## 5.3 Capture point and divergent dynamics

A useful refinement of balance reasoning is the **capture point**. Under the LIPM assumptions, define

$$
\omega_0 = \sqrt{\frac{g}{z_c}}, \qquad \xi = x + \frac{\dot x}{\omega_0}.
$$

The capture point $\xi$ is the point at which the robot would need to step to come to rest, given the current state of the simplified model. It captures the unstable component of the dynamics and is highly influential in walking control.

The practical lesson is that dynamic balance is about regulating not only where the CoM is, but also where it is going.

## 5.4 Centroidal wrench feasibility

For general multi-contact situations, a more complete stability concept is **contact wrench feasibility**. The external wrench required by the planned centroidal dynamics must lie in the set of wrenches that can be generated by the active contacts under friction and geometry constraints.

This view generalizes support-polygon reasoning and is indispensable for:

- hand-assisted balancing;
- non-coplanar contacts;
- leaning or bracing behaviors;
- contact-rich loco-manipulation.

In this sense, balance is not merely “keeping the robot upright.” It is maintaining the existence of a feasible contact wrench solution over time.

## 5.5 Why ZMP is still worth teaching

Although modern humanoid controllers often go beyond ZMP, it remains pedagogically valuable because it teaches three important ideas:

1. support is not only about kinematics but also about wrench distribution;
2. there is a useful reduced-order link between CoM acceleration and ground reaction effects;
3. simplified balance models are often the right abstraction layer for planning.

The mistake is not teaching ZMP. The mistake is stopping there.

# 6. Contact sensing, tactile information, and state estimation

## 6.1 Why modeling alone is insufficient

Rigid-body models never fully determine what is happening at the contact interface. Real contact is uncertain because of:

- terrain compliance;
- foot sole deformation;
- model mismatch in friction;
- sensor bias and noise;
- actuator latency and backlash;
- partial contacts and contact patch evolution.

Therefore, a humanoid system must estimate contact state online rather than assuming it is perfectly known.

## 6.2 Common sensing modalities

Humanoid platforms typically use a combination of the following:

- ankle or wrist six-axis force–torque sensors;
- joint torque sensors or motor current estimates;
- IMUs for floating-base acceleration and angular velocity;
- tactile arrays or pressure sensors on the feet or hands;
- vision for terrain geometry and contact anticipation.

Modern simulation platforms expose similar abstractions. MuJoCo includes unified frictional contact handling with configurable contact models and friction dimensions in the solver documentation, while Isaac Lab provides physics-based contact sensing abstractions that report net contact force on selected bodies and can be filtered by interacting geometry. These tools are valuable, but their semantics must be understood before they are trusted in state estimation, contact reasoning, or planning studies.

## 6.3 Contact state estimation

Typical estimation outputs include:

- binary contact/no-contact indicators;
- estimated normal force;
- estimated CoP or pressure distribution;
- slip detection;
- phase identification during gait.

A simple detector might threshold the vertical ground reaction force. A more robust estimator fuses force, joint kinematics, and IMU consistency. For example, if a foot is believed to be in stance, then its velocity relative to the world should be approximately zero. This contact condition can be used as a pseudo-measurement in a state estimator for the floating base.

## 6.4 Contact-aided base estimation

One of the most important implementation patterns in humanoid robotics is to treat a stance foot as a temporary anchor for estimating the floating base. When a contact is reliable, the estimator can use

$$
J_c(q) v \approx 0
$$

as a measurement relation. This sharply reduces drift and can make the difference between a usable and an unusable state-estimation pipeline.

The risk is obvious: if the controller believes a slipping foot is rigidly anchored, the estimator will inject false information and may destabilize the robot.

## 6.5 Practical estimator design

A robust humanoid estimator typically combines:

- IMU propagation for short-horizon inertial consistency;
- joint encoders for kinematic reconstruction;
- contact hypotheses from force and kinematic consistency;
- delayed or low-rate exteroceptive correction from vision or motion capture.

A common engineering principle is to separate **contact detection**, **contact confidence**, and **contact use**. Binary logic alone is fragile. Confidence-weighted fusion is often far more stable.

# 7. Planning with contact

## 7.1 Why humanoid planning is contact planning

For manipulators, it is often sufficient to plan a smooth end-effector path plus collision avoidance. For humanoids, that is rarely enough. The planner must decide not only where the body should move, but also:

- which contacts should exist;
- when contacts should be created or removed;
- whether the contact set can support the required wrench;
- whether the posture is kinematically reachable;
- whether the transitions are dynamically feasible.

Thus, humanoid planning is often best understood as **planning over contact modes**.

## 7.2 Contact sequence planning

At the highest level, a planner may choose a discrete sequence such as

$$
\text{double support} \rightarrow \text{left support} \rightarrow \text{double support} \rightarrow \text{right hand + left foot} \rightarrow \cdots
$$

Each mode determines a new set of active constraints and admissible wrenches. This combinatorial structure is a major source of complexity.

Contact sequence planning is especially difficult when the environment is cluttered or the task is multi-contact loco-manipulation, such as stepping while pushing a door.

## 7.3 Reduced-order planning

A practical strategy is to separate planning into levels.

### High level

Choose contact sequence, terrain footholds, hand placements, and mode timings.

### Mid level

Plan centroidal trajectories, CoM motion, angular momentum evolution, and feasible support wrenches.

### Low level

Generate joint-space motions, contact timings, and kinematic refinements consistent with the full robot geometry and dynamics.

This hierarchy is not merely a convenience. It reflects the fact that full-body contact planning in the raw joint space is too expensive and too opaque for most tasks.

## 7.4 Planning methods

Three broad families appear repeatedly.

### Search-based methods

These explicitly search over contact placements or modes. They are intuitive and can leverage heuristics, but the search tree grows quickly.

### Optimization-based methods

These formulate a trajectory optimization or MPC problem with contact constraints, friction constraints, and task objectives. They can capture dynamics more faithfully, but contact switching introduces nonconvexity or mixed-integer structure.

## 7.5 Contact-implicit planning

A more ambitious formulation is to plan body motion, contact timing, contact forces, and even contact activation together. This is often called **contact-implicit trajectory optimization**. Instead of prescribing a contact schedule in advance, the optimization decides which contacts should exist and when they should be active, subject to dynamics, collision, and complementarity-style contact constraints.

The attraction of contact-implicit planning is obvious: it offers a unified statement of “find a physically feasible behavior.” The difficulty is equally obvious: the resulting optimization is highly nonconvex, may include nonsmooth or mixed-integer structure, and is often expensive for full humanoid models.

For graduate study, the important lesson is conceptual. Contact planning variables are not limited to pose and timing. They often include:

- contact mode;
- contact location and geometry;
- contact force or wrench;
- mode-transition timing;
- reduced-order momentum or centroidal trajectories.

A planning formulation becomes more expressive as more of these variables are optimized jointly, but it also becomes harder to solve reliably.

## 7.6 A conceptual example: stair climbing with a handrail

A contact-aware planner for stair climbing with a handrail must answer several questions.

1. Which handrail contact should be used, and when?
2. Is the handrail grasp or support contact necessary for feasibility, or merely helpful for robustness?
3. Can the chosen stance foot and hand contact jointly support the required centroidal wrench during the step-up phase?
4. Is the upper-body posture kinematically compatible with the lower-body step and with self-collision avoidance?
5. How should the transition be staged so that the swing foot touchdown does not destabilize the upper-body support contact?

The point of this example is that contact planning is fundamentally a **coupled mobility and force feasibility** problem.



# 8. From theory to implementation

## 8.1 A practical implementation stack without full control design

Even before a dedicated control class, students can build a substantial humanoid contact pipeline by focusing on the layers that precede or surround low-level control design:

### Layer 1: geometric and inertial modeling

Build the floating-base rigid-body model, define contact frames, and verify mass properties, collision geometry, and foot or hand contact patches.

### Layer 2: sensing and state estimation

Estimate floating-base state, contact state, and terrain or object interaction cues from force, IMU, encoder, tactile, and vision signals.

### Layer 3: contact-mode logic

Determine which contacts are intended, which are detected, and which are trusted strongly enough to enter estimation or planning.

### Layer 4: reduced-order planning

Generate footholds, contact schedules, and centroidal or CoM references that are dynamically plausible, even if no full whole-body controller is yet implemented.

### Layer 5: simulation and validation infrastructure

Create repeatable scenarios, perturbation tests, logging tools, and diagnostics for checking contact consistency, friction margin, impact timing, and estimator drift.

This stack is already rich enough for a strong graduate project. It also provides the natural prerequisite structure for later classes on humanoid control and humanoid learning.

## 8.2 What to model explicitly in simulation

A simulator is not useful merely because it has rigid-body dynamics. It is useful when it represents the uncertainties that dominate contact behavior.

For humanoid contact tasks, the most important items are often:

- actuator delay and bandwidth, insofar as they affect sensed motion and contact timing;
- contact friction coefficients and their variation;
- foot geometry, hand geometry, and collision shapes;
- terrain height, slope, roughness, and possible compliance uncertainty;
- sensor noise, bias, and estimator latency;
- impact timing and contact detection semantics.

MuJoCo's documentation is especially useful for understanding how the solver handles frictional contact, friction dimensions, and the distinction between elliptic and pyramidal cones. Isaac Lab's documentation is useful for understanding the semantics of physics-based contact sensing and how contact-force signals are exposed to higher-level estimation and planning code.

The lesson is that “high-fidelity” does not always mean visually rich. It means **faithful to the failure modes that matter**.

## 8.3 Choosing a contact model

In simulation, contact is always an approximation. The right approximation depends on the research question.

### For mechanics and estimation studies

Use a stable rigid-contact model with conservative friction and simple foot or hand geometry. Prioritize solver stability, repeatability, and interpretable contact signals.

### For terrain and foothold studies

Represent the terrain features that matter to support quality: height variation, slope, edges, roughness, or deformability. Contact patch interpretation becomes more important than visual realism.

### For contact-rich manipulation or assembly studies

Contact patch fidelity, geometry resolution, and tangential compliance become much more important, because local geometry and partial contact evolution may dominate the outcome.

## 8.4 Numerical conditioning and debugging

A few implementation issues recur constantly.

### Ill-conditioned Jacobians

Near singular configurations or poor contact geometry, the contact Jacobian or reduced planning Jacobians can become ill-conditioned. Regularization is not optional.

### Inconsistent contact assumptions

If the estimator assumes a stance foot is rigidly fixed while the planning logic implicitly requires motion inconsistent with that assumption, the entire stack becomes brittle.

### Overconstrained formulations

Students often write estimators or planners with too many hard assumptions. A practical rule is: hard constraints should represent genuine mechanics or unambiguous sensing relations; many modeling preferences should remain soft.

### Timing mismatch

If the estimator, contact detector, and planner run at different rates without consistent buffering or timestamp handling, contact transitions become unreliable.

## 8.5 Debugging strategy

A productive debugging sequence is:

1. verify free-space rigid-body dynamics without contact;
2. verify a single static stance contact;
3. verify CoP and normal force consistency in quiet standing;
4. verify friction margin under small horizontal pushes;
5. verify one swing-foot touchdown event;
6. only then attempt repeated stepping, hand support, or loco-manipulation planning.

In other words, do not debug “walking” first. Debug the physics of one reliable contact, then one reliable transition, then the sequence.

## 8.6 Sim-to-real understanding

For humanoid contact, the sim-to-real gap is rarely a single issue. It is usually a collection of mismatches:

- contact geometry is simplified;
- friction is different from what was assumed;
- impacts are softer or harder than in simulation;
- sensing delays and estimator confidence are miscalibrated;
- the robot makes partial contacts that were never modeled explicitly.

A strong implementation culture therefore treats transfer as a modeling-and-measurement problem, not only as a software-deployment problem.

# 9. Research directions and open problems

## 9.1 Tactile and distributed contact sensing

Humanoid contact reasoning still relies heavily on sparse force–torque sensing and indirect inference. Rich tactile sensing on feet, shins, forearms, torso, and hands could significantly improve:

- contact localization;
- early slip detection;
- estimation of partial contact patches;
- safer human contact;
- compliant whole-body bracing;
- terrain and object-property estimation.

This is one of the clearest ways in which humanoid contact and dexterous manipulation research are converging.

## 9.2 Foot tactile sensing and terrain understanding

For locomotion, foot tactile sensing remains underexplored relative to ankle force–torque sensing. Yet it can provide information that a single wrench estimate cannot: pressure distribution, local terrain slope, edge contact, patch shape, and possibly terrain class or compliance.

A major research question is how to convert such dense foothold information into compact variables that are directly useful for estimation and planning.

## 9.3 Whole-body tactile sensing for humanoid loco-manipulation

Whole-body tactile sensing extends contact perception beyond fingertips and foot soles. It enables contact-rich behaviors involving forearms, torso, thighs, and shins, especially in cluttered environments or during large-object interaction.

The open problem is not merely to build large-area sensing skins. It is to integrate distributed contact information into a coherent multibody model so that the robot can reason about support, collision, and purposeful bracing within a single framework.

## 9.4 Beyond rigid contact

Rigid contact is a powerful abstraction, but many important behaviors involve compliance:

- rubber soles deform;
- carpets and mats compress;
- handrails flex;
- objects being manipulated deform;
- soft protective covers alter impact and friction.

A major open challenge is how to incorporate compliant or deformable contact into real-time estimation and planning without making the model intractable.

## 9.5 Contact-implicit planning at humanoid scale

Contact-implicit planning is conceptually elegant, but full humanoid problems remain difficult because of high dimensionality, many candidate contact surfaces, and strongly nonconvex mode transitions.

Important open questions include:

- how to scale contact-implicit methods to whole-body humanoids;
- how to combine geometric search with optimization efficiently;
- how to incorporate tactile observations into contact-plan revision;
- how to certify or at least diagnose physical feasibility during mode changes.

# 10. Summary of key takeaways

The main lessons of this manuscript can be summarized as follows.

1. **Contact is the mechanical foundation of humanoid behavior.** Without contact, the floating-base humanoid cannot support itself or manipulate the environment in a physically meaningful way.

2. **Contact introduces both forces and constraints.** The same contact that supports the robot also restricts its motion and imposes unilateral and frictional feasibility conditions.

3. **Humanoid contact is hybrid.** Continuous within-mode dynamics are interrupted by discrete mode changes such as touchdown and liftoff.

4. **The core modeling equation is constrained floating-base dynamics.**

   $$
   M(q) \dot v + h(q,v) = S^T \boldsymbol{\tau} + J_c(q)^T \boldsymbol{\lambda},
   $$

   together with acceleration-level contact constraints.

5. **Balance is a wrench-feasibility problem, not only a geometry problem.** The support polygon and ZMP are useful, but centroidal dynamics and contact wrench feasibility provide the more general viewpoint.

6. **Estimation matters as much as mechanics.** Incorrect contact assumptions can corrupt the state estimate and invalidate otherwise sensible planning logic.

7. **Planning is often planning over contact modes.** Where and when the robot touches the world is as important as the trajectory it follows.

8. **Good implementation requires modeling the right uncertainties.** Friction variation, terrain geometry, contact timing, estimator semantics, and impact interpretation usually matter more than symbolic elegance.

9. **Tactile sensing is a key frontier.** Hands, feet, and distributed body sensing can supply contact information that classical sparse sensors cannot.

10. **A strong humanoid contact stack starts with structure.** Clear models, clear assumptions, and staged validation are more important than premature algorithmic complexity.

# 11. Suggested implementation roadmap for students

A sensible semester project based on these notes would proceed in the following order.

## Stage 1: Single-contact standing

- Build a floating-base humanoid or reduced biped model.
- Verify gravity compensation and quiet standing in double support.
- Compute ground reaction forces and CoP.
- Add friction constraints and verify no-slip margins.

## Stage 2: Contact-aware estimation

- Add force sensing or simulated contact sensing.
- Estimate stance versus swing.
- Fuse IMU and kinematics using stance-foot constraints.

## Stage 3: Reduced-order balance analysis

- Implement CoM, LIPM, or centroidal analyses for quiet standing and simple stepping.
- Add ZMP or capture-point reasoning.
- Test under pushes, friction variation, and terrain changes.

## Stage 4: Contact planning

- Build a footstep planner or simple contact-sequence planner.
- Add a hand-support option for a stair or railing scenario.
- Check kinematic reachability and centroidal wrench feasibility.

## Stage 5: Contact transitions

- Implement touchdown detection and contact-state updates.
- Evaluate robustness to timing errors and terrain height mismatch.
- Diagnose estimator and planner behavior during liftoff and touchdown.

## Stage 6: Multi-contact reasoning or loco-manipulation setup

- Add hand support, a door interaction, or stair climbing with a railing.
- Compare a fixed contact schedule against a more adaptive planner.
- Document where geometric feasibility, force feasibility, and sensing reliability become the limiting factors.

# 12. References for further study

The following references are particularly useful for deeper study and implementation.

1. Russ Tedrake, *Underactuated Robotics* (MIT course notes), especially the chapters on contact, multibody dynamics, and highly articulated legged robots.
2. David E. Orin, Ambarish Goswami, and Sung-Hee Lee, “Centroidal dynamics of a humanoid robot,” *Autonomous Robots*, 2013.
3. Haiyang Dai, Andres Valenzuela, and Russ Tedrake, “Whole-body motion planning with centroidal dynamics and full kinematics,” Humanoids 2014.
4. Stéphane Caron and collaborators on friction cones, centroidal wrench cones, and contact geometry for legged and humanoid robots.
5. Qiang Li, Oliver Kroemer, Zhe Su, Filipe Fernandes Veiga, Mohsen Kaboli, and Helge Ritter, “A Review of Tactile Information: Perception and Action Through Touch,” *IEEE Transactions on Robotics*, 2020.
6. Zhaoyuan Gu, Junheng Li, Wenlan Shen, Wenhao Yu, Zhaoming Xie, Stephen McCrory, Xianyi Cheng, Abdulaziz Shamsah, Robert Griffin, C. Karen Liu, Abderrahmane Kheddar, Xue Bin Peng, Yuke Zhu, Guanya Shi, Quan Nguyen, Gordon Cheng, Huijun Gao, and Ye Zhao, “Humanoid Locomotion and Manipulation: Current Progress and Challenges in Control, Planning, and Learning,” 2025, especially the sections on tactile sensing and multi-contact planning.
7. MuJoCo documentation, especially the sections on modeling and computation of frictional contact.
8. Isaac Lab documentation, especially the sections on contact sensors and physics-based sensor abstractions.