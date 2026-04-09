# Class 12 on Humanoid Contact

## Contents

1. Motivation and scope
2. Mathematical preliminaries and notation
3. Contact mechanics for humanoid robots
4. Humanoid dynamics under contact
5. Balance and stability under contact
6. Contact sensing, tactile information, and state estimation
7. Planning with contact
8. Contact-aware control
9. From theory to implementation
10. Research directions and open problems
11. Summary of key takeaways
12. Suggested implementation roadmap for students
13. References for further study
14. Source note

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

These notes focus on **rigid-body contact modeling for humanoids** and the associated control stack. The emphasis is on principles that remain useful across platforms:

- torque-controlled humanoids;
- quasi-direct-drive humanoids;
- research humanoids in MuJoCo, Isaac Lab, or similar simulators;
- whole-body loco-manipulation systems;
- contact-aware learning pipelines.

We do **not** attempt a complete treatment of complementarity theory, nonsmooth analysis, deformable contact mechanics, or tactile manipulation at the level of a dedicated dexterous manipulation course. Instead, the goal is a unified working understanding that connects the mathematics to the way advanced humanoid systems are actually implemented.

# 2. Mathematical preliminaries and notation

## 2.1 Floating-base coordinates

A humanoid is modeled as a multibody system with a free-floating base and actuated joints. Let

$$
\mathbf{q} = \begin{bmatrix} \mathbf{q}_b \\ \mathbf{q}_a \end{bmatrix},
$$

where $\mathbf{q}_b$ denotes the floating-base configuration and $\mathbf{q}_a \in \mathbb{R}^{n_a}$ denotes the actuated joint coordinates. In practice, $q_b$ may be represented by base position plus orientation, often using a quaternion or rotation matrix internally for numerical robustness.

A velocity representation is often more convenient than differentiating the configuration coordinates directly. Let

$$
\mathbf{v} = \begin{bmatrix} \mathbf{v}_b \\ \dot{\mathbf{q}}_a \end{bmatrix},
$$

where $\mathbf{v}_b \in \mathbb{R}^6$ is the spatial base velocity, containing linear and angular components.

The distinction between $q$ and $v$ matters because floating-base orientation is not always minimally represented in a globally smooth way. Most rigid-body algorithms therefore work with generalized configuration $q$ and generalized velocity $v$ rather than naively using $\dot q$ everywhere.

## 2.2 Actuation selection matrix

Because only the internal joints are actuated, the generalized torque vector enters through a selection matrix $S$:

$$
\mathbf{M}(\mathbf{q})\dot{\mathbf{v}} + \mathbf{h}(\mathbf{q},\mathbf{v}) = \mathbf{S}^T \boldsymbol{\tau} + \sum_i \mathbf{J}_i(\mathbf{q})^T \boldsymbol{\lambda}_i.
$$

Here:

- $\mathbf{M}(\mathbf{q})$ is the generalized mass matrix;
- $\mathbf{h}(\mathbf{q},\mathbf{v})$ collects Coriolis, centrifugal, gravitational, and sometimes passive terms;
- $\boldsymbol{\tau} \in \mathbb{R}^{n_a}$ are joint torques;
- $\mathbf{J}_i(\mathbf{q})$ is the Jacobian of contact $i$;
- $\boldsymbol{\lambda}_i$ is the contact wrench or contact force parameter for that contact.

The presence of $S^T \tau$ rather than an arbitrary generalized force vector is the formal expression of **underactuation**. The robot cannot directly command the base wrench. It can only influence the base through internal actuation and external contacts.

## 2.3 Contact Jacobians

Let a contact point or contact frame be attached to a body of the robot. Its spatial velocity can be written as

$$
\mathbf{v}_c = \mathbf{J}_c(\mathbf{q}) \mathbf{v}.
$$

If the contact is assumed to stick, then the relative velocity with respect to the environment is constrained. For a stationary environment contact, the ideal sticking condition is

$$
\mathbf{J}_c(\mathbf{q}) \mathbf{v} = \mathbf{0}.
$$

Differentiating yields the acceleration-level contact constraint:

$$
\mathbf{J}_c(\mathbf{q}) \dot{\mathbf{v}} + \dot{\mathbf{J}}_c(\mathbf{q},\mathbf{v}) \mathbf{v} = \mathbf{0}.
$$

This is the standard way contact enters inverse dynamics and whole-body control formulations.

## 2.4 A note on frames and wrenches

Humanoid contact calculations are full of frame conventions. You may easily lose track of signs, moments, and Jacobians because frames are not handled consistently. A useful discipline is:

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
\phi(\mathbf{q}) \ge 0.
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
\phi(\mathbf{q})\,\lambda_n = 0,
$$

which encodes the idea that either the bodies are separated and the normal force vanishes, or they are in contact and a compressive force may appear.

This relation is conceptually important even if a given simulator or controller does not solve a strict complementarity problem.

## 3.2 Tangential friction and the Coulomb model

At a contact, the tangential force is limited by friction. In the classical isotropic Coulomb model,

$$
\lVert \mathbf{f}_t \rVert \le \mu f_n,
$$

where $\mathbf{f}_t$ is the tangential contact-force vector, $f_n$ is the normal force magnitude, and $\mu$ is the friction coefficient.

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

A point force acting at contact point $\mathbf{p}$ induces not only a force but also a moment about a chosen reference point $\mathbf{o}$. The combined quantity is a **wrench**:

$$
\mathbf{w} = \begin{bmatrix} \mathbf{f} \\ \boldsymbol{\mu} \end{bmatrix}, \qquad \boldsymbol{\mu} = (\mathbf{p} - \mathbf{o}) \times \mathbf{f}.
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
\mathbf{M}(\mathbf{q})(\mathbf{v}^+ - \mathbf{v}^-) = \mathbf{J}_c(\mathbf{q})^T \boldsymbol{\Lambda},
$$

where $v^-$ and $v^+$ are the pre- and post-impact generalized velocities and $\Lambda$ is the contact impulse.

For a perfectly plastic sticking impact, one additionally imposes

$$
\mathbf{J}_c(\mathbf{q}) \mathbf{v}^+ = \mathbf{0}.
$$

Even when a practical controller does not explicitly solve impact equations online, the conceptual distinction between within-mode continuous dynamics and mode-switch events is essential.

# 4. Humanoid dynamics under contact

## 4.1 Floating-base equations of motion

The standard constrained rigid-body dynamics equation for a humanoid is

$$
\mathbf{M}(\mathbf{q}) \dot{\mathbf{v}} + \mathbf{h}(\mathbf{q},\mathbf{v}) = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c(\mathbf{q})^T \boldsymbol{\lambda}.
$$

This equation is the backbone of nearly every modern whole-body controller. It says that generalized acceleration is determined by inertia, bias forces, joint torques, and environmental contact forces.

Several immediate consequences follow.

First, because the base is unactuated, the robot cannot arbitrarily impose its full-body acceleration. It must remain consistent with the external contacts.

Second, the contact forces are not arbitrary either. They are unknown variables that must satisfy unilateral and frictional feasibility.

Third, contact changes the effective capabilities of the robot. A floating-base humanoid in midair is very different from the same robot in double support, and different again when one hand braces against a wall.

## 4.2 Acceleration-level contact constraints

Assume a contact is sticking. Then

$$
\mathbf{J}_c(\mathbf{q}) \mathbf{v} = \mathbf{0}.
$$

Differentiating gives

$$
\mathbf{J}_c(\mathbf{q}) \dot{\mathbf{v}} + \dot{\mathbf{J}}_c(\mathbf{q},\mathbf{v}) \mathbf{v} = \mathbf{0}.
$$

The constrained dynamics problem is then to solve for $(\dot{\mathbf{v}}, \boldsymbol{\lambda})$ given torques and contact mode.

One can write the coupled linear system

$$
\begin{bmatrix}
\mathbf{M} & -\mathbf{J}_c^T \\
\mathbf{J}_c & \mathbf{0}
\end{bmatrix}
\begin{bmatrix}
\dot{\mathbf{v}} \\
\boldsymbol{\lambda}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{S}^T\boldsymbol{\tau} - \mathbf{h} \\
-\dot{\mathbf{J}}_c \mathbf{v}
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
(\dot{\mathbf{v}}, \boldsymbol{\tau}, \boldsymbol{\lambda})
$$

and solve

$$
\min_{\dot{\mathbf{v}}, \boldsymbol{\tau}, \boldsymbol{\lambda}} \; \text{task tracking error} + \text{regularization}
$$

subject to

$$
\mathbf{M}\dot{\mathbf{v}} + \mathbf{h} = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{J}_c^T\boldsymbol{\lambda},
$$

$$
\mathbf{J}_c\dot{\mathbf{v}} + \dot{\mathbf{J}}_c \mathbf{v} = \mathbf{0},
$$

plus torque limits and friction constraints.

This turns the contact problem into a structured numerical optimization problem rather than a purely symbolic mechanics problem.

## 4.5 Centroidal dynamics reduction

For planning and balance reasoning, the full robot dynamics are often reduced to **centroidal dynamics**. Let $c$ denote the CoM, $m$ the total mass, and $L_c$ the angular momentum about the CoM. Then

$$
m \ddot{\mathbf{c}} = \sum_i \mathbf{f}_i + m\mathbf{g},
$$

$$
\dot{\mathbf{L}}_c = \sum_i (\mathbf{p}_i - \mathbf{c}) \times \mathbf{f}_i + \sum_i \boldsymbol{\mu}_i.
$$

These equations isolate the global effect of contact wrenches on the whole body. They ignore many internal details but preserve what matters for support, momentum, and balance.

A standard and very useful relationship is the centroidal momentum map

$$
\mathbf{h}_G = \mathbf{A}_G(\mathbf{q}) \mathbf{v},
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

The CoP is the point on the support surface where the net pressure distribution is effectively applied. If the foot wrench is known, the CoP can be extracted from the contact-moment components.

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

# 6. Contact sensing and state estimation

## 6.1 Why modeling alone is insufficient

Rigid-body models never fully determine what is happening at the contact interface. Real contact is uncertain because of:

- terrain compliance;
- foot sole deformation;
- model mismatch in friction;
- sensor bias and noise;
- actuator latency and backlash;
- partial contacts and contact patch evolution.

Therefore, humanoid control must estimate contact state online rather than assuming it is perfectly known.

## 6.2 A hierarchy of tactile information

A useful way to organize contact sensing is to distinguish four levels of information:

1. **sensor-level signals**, such as raw taxel values, force–torque measurements, vibration, temperature, or pre-contact proximity;
2. **contact-level information**, such as contact position, contact normal, local curvature, normal and tangential force, contact moment, making/breaking contact events, or slip;
3. **object- or environment-level information**, such as shape, pose, mass distribution, terrain type, foothold geometry, or local material properties;
4. **action-level information**, such as how to choose the next action, how to adapt a running action using tactile feedback, when to terminate an action, and how to infer whether the action succeeded.

This hierarchy is valuable in humanoid robotics because it makes clear that sensing is not only about measuring force. The controller does not act on raw taxels directly. It acts on estimated contact state, inferred environment properties, and action-level decisions built on top of lower-level tactile information.

## 6.3 Sensor modalities and body-region coverage

Humanoid platforms typically combine several sensing modalities:

- ankle or wrist six-axis force–torque sensors;
- joint torque sensors or motor-current-based force estimates;
- IMUs for floating-base acceleration and angular velocity;
- tactile arrays or pressure sensors on feet, hands, arms, or torso;
- vibration-sensitive sensing for contact onset and slip;
- vision for terrain geometry, contact anticipation, and object localization;
- in some systems, proximity or pre-touch sensing to anticipate imminent contact.

An important design principle is that **sensor coverage should follow function**. Hands need high spatial and temporal resolution for dexterous interaction, slip detection, and local geometry estimation. Feet need robust load-bearing sensing, good shear tolerance, and pressure distribution awareness. Whole-body skin on arms or torso usually operates at lower spatial resolution, but must tolerate large loads and large-area contact during bracing, safe human interaction, or object transport.

This immediately leads to an implementation insight: there is no single “best tactile sensor” for humanoids. Sensor design depends on where the sensor is mounted, what force range it must survive, what bandwidth the control loop requires, and whether the downstream objective is grasp control, foothold assessment, whole-body compliance, or contact-aware planning.

## 6.4 Tactile sensing on the hands

On dexterous hands, tactile sensing supports much more than grasp-force measurement. The hand can estimate:

- contact position on the fingertip or palm;
- contact normal and local surface geometry;
- normal and tangential forces;
- incipient slip and gross slip;
- object texture, stiffness, and sometimes temperature-related properties.

For humanoid manipulation, these signals support several functions. First, they improve **grasp controllability** through force regulation and slip-aware grasp adaptation. Second, they enable **interactive perception**, where the hand explores an object in order to refine its local geometry or identify uncertain properties. Third, they provide **dynamic contact-state information** during in-hand manipulation, where the robot must reason about how local contact changes affect both object motion and whole-body balance.

A key research lesson is that tactile hands are not only estimation devices; they are action-conditioning devices. Tactile feedback changes how the robot chooses and executes manipulation strategies.

## 6.5 Tactile sensing on the feet

Foot-contact sensing deserves special emphasis in humanoid robotics because ankle force–torque sensing alone is not always enough. A six-axis sensor at the ankle can estimate the net wrench transmitted through the foot, but it does not directly reveal the full contact patch, local pressure distribution, terrain microgeometry, or foothold quality.

A foot tactile array or pressure-sensitive sole can provide information such as:

- contact patch location and shape;
- pressure distribution over the sole;
- center of pressure evolution;
- local terrain slope;
- terrain class or foothold type;
- early warning of edge contact, partial contact, or incipient slip.

For locomotion, this matters because the robot does not interact with an abstract ground plane. It interacts with a finite, uncertain foothold. In rough terrain, a controller that knows only the total ground reaction wrench may miss the fact that the contact is concentrated on a small edge or that the foothold is softer or more slippery than expected.

From an implementation standpoint, foot tactile sensing is hard because humanoid feet experience large intermittent impacts, high shear, and repeated load cycles. Sensor packaging, durability, wiring, and time synchronization become as important as estimation accuracy.

## 6.6 Whole-body tactile sensing

Whole-body tactile sensing extends the same logic beyond hands and feet to the arms, torso, legs, and possibly joints. This is especially valuable for humanoid robots because many advanced behaviors are not fingertip-only and not foot-only. They involve contact with large body regions.

Whole-body tactile sensing supports:

- safe physical human–robot interaction;
- compliant contact with walls, rails, furniture, or large objects;
- collision awareness in clutter;
- whole-arm or whole-torso bracing;
- tactile exploration of large objects that are not fully visible;
- large-object transport and balancing during whole-body loco-manipulation.

This is conceptually important for the lecture notes because it reframes contact from a narrow “support foot” or “end-effector” issue into a general whole-body interaction problem. Once multiple body regions can become informative contacts, sensing and control can no longer be separated cleanly. The sensed contact pattern itself becomes part of the task representation.

## 6.7 Contact-level estimation

The most immediate outputs of a contact estimator are contact-level quantities. Typical estimation outputs include:

- binary contact/no-contact indicators;
- estimated normal force and tangential force;
- estimated contact point or pressure-weighted center;
- estimated contact normal or local surface orientation;
- estimated CoP or pressure distribution for feet;
- contact events such as making contact, breaking contact, or sliding contact;
- slip or incipient-slip detection.

A simple detector might threshold the vertical ground reaction force. A more robust estimator fuses force, joint kinematics, vibration signatures, and IMU consistency. In tactile manipulation, a pressure-weighted centroid over activated taxels may approximate contact position. For feet, the pressure map over the sole may reveal whether the robot is on a flat support, an edge, or a partially supported patch.

From a mathematical viewpoint, the estimator attempts to infer latent contact variables from noisy measurements and structural constraints. In the simplest case this is threshold logic; in more advanced cases it is a Bayesian filtering or learned inference problem.

## 6.8 Contact-aided base estimation

One of the most important implementation patterns in humanoid robotics is to treat a reliable stance contact as a temporary anchor for estimating the floating base. When a contact is trusted, the estimator can use

$$
\mathbf{J}_c(\mathbf{q}) \mathbf{v} \approx \mathbf{0}
$$

as a measurement relation. This sharply reduces drift and can make the difference between a stable and unstable controller.

The risk is equally important: if the controller believes a slipping foot is rigidly anchored, the estimator will inject false information and may destabilize the robot. Therefore, contact-aided state estimation should never be separated conceptually from contact confidence estimation.

## 6.9 Object- and environment-level inference

Beyond local contacts, tactile sensing can support object- or environment-level inference. For manipulation, the hand may combine multiple contacts over time to infer object pose, local shape, mass-related properties, or contact-relevant material properties such as stiffness and friction. For locomotion, the feet may infer terrain categories, slope, roughness, or foothold geometry.

This point is worth stressing in a graduate course: the robot does not need a full volumetric model of the environment for contact-aware control. It often needs a smaller set of contact-relevant latent variables. For example, the controller may only need to know that the foothold is narrow, compliant, and sloped, or that the object contact is slipping and its local friction is lower than expected.

## 6.10 Action-level tactile feedback and multimodal fusion

The tactile review highlights a point that is highly relevant for humanoids: tactile information is not only used to estimate the current contact state, but also to decide **what to do next**. Action-level tactile information includes:

- selecting the next exploratory or manipulation action;
- initializing action parameters from prior tactile experience;
- adapting a running action through tactile servoing;
- triggering reflex-like responses such as grip-force increase under incipient slip;
- terminating an action when a contact event indicates success or failure;
- verifying whether the action outcome matches the task goal.

This idea carries over naturally to humanoid locomotion and loco-manipulation. A foot tactile event may trigger replanning of the next step. A whole-arm contact may switch a controller from collision avoidance to compliant obstacle clearing. A hand contact on a rail may change whether the next motion is treated as pure locomotion or as multi-contact support.

In practice, the most robust systems fuse tactile sensing with proprioception and vision. Vision predicts candidate contacts and terrain geometry; proprioception gives kinematic consistency and joint effort; tactile sensing confirms what contact actually occurred and whether the interaction is proceeding as intended. This multimodal fusion is one of the key themes for implementation-oriented humanoid systems.


# 7. Planning with contact

## 7.1 Why humanoid planning is contact planning

For manipulators, it is often sufficient to plan a smooth end-effector path plus collision avoidance. For humanoids, that is rarely enough. The planner must decide not only where the body should move, but also:

- which contacts should exist;
- when contacts should be created or removed;
- whether the contact set can support the required wrench;
- whether the posture is kinematically reachable;
- whether the transitions are dynamically feasible.

Thus, humanoid planning is often best understood as **planning over contact modes**.

## 7.2 Planning variables: state, contact, and force

A useful refinement, emphasized in recent humanoid planning surveys, is to distinguish the planning variables explicitly.

A full loco-manipulation planner may need to reason about:

- **state variables**, such as CoM motion, centroidal momentum, joint configuration, and object state;
- **contact variables**, such as contact position, contact timing, and contact mode;
- **force variables**, such as contact force or wrench distribution across active contacts.

This decomposition is valuable because it clarifies why humanoid planning is difficult. The robot is not merely choosing a configuration trajectory. It is simultaneously choosing how to support itself and how to exchange forces with the environment or manipulated objects.

## 7.3 Contact sequence planning

At the highest level, a planner may choose a discrete sequence such as

$$
\text{double support} \rightarrow \text{left support} \rightarrow \text{double support} \rightarrow \text{right hand + left foot} \rightarrow \cdots
$$

Each mode determines a new set of active constraints and admissible wrenches. This combinatorial structure is a major source of complexity.

Contact sequence planning is especially difficult when the environment is cluttered or the task is multi-contact loco-manipulation, such as stepping while pushing a door or climbing while using a handrail.

## 7.4 Multi-contact trajectory planning versus static pose planning

In humanoid applications it is useful to distinguish at least two planning regimes.

### Multi-contact trajectory planning

This regime plans over a horizon and reasons jointly about contact mode, contact location, contact force, and state evolution. It is the right abstraction for dynamic tasks such as carrying an object while walking, traversing with hand support, or pushing a cart while maintaining balance.

### Static pose planning or pose optimization

This regime focuses on the current time step or a small local horizon and solves for a feasible whole-body pose, contact placement, and force distribution. It is lighter computationally and often useful for generating feasible intermediate configurations, initializing trajectory optimization, or checking whether a bracing posture is possible.

The distinction matters in implementation. Trajectory planning offers anticipatory coordination but is computationally expensive. Pose planning is often cheaper and easier to deploy online, but it does not by itself solve long-horizon transition feasibility.

## 7.5 Reduced-order and hierarchical planning

A practical strategy is to separate planning into levels.

### High level

Choose contact sequence, terrain footholds, hand placements, and mode timings.

### Mid level

Plan centroidal trajectories, CoM motion, angular momentum evolution, and feasible support wrenches.

### Low level

Generate joint trajectories and control commands consistent with the full robot kinematics and dynamics.

This hierarchy is not merely a convenience. It reflects the fact that full-body contact planning in raw joint space is too expensive and too opaque for most tasks. It also mirrors how successful humanoid systems are often built: a predictive reduced-order planner feeds a reactive whole-body controller.

## 7.6 Contact-implicit planning and trajectory optimization

A major open direction in humanoid planning is to avoid pre-specifying the full contact schedule. In **contact-implicit planning** or **contact-implicit trajectory optimization (CITO)**, the optimization attempts to solve for body motion, contact position, contact force, and contact mode simultaneously.

Conceptually, the problem becomes

$$
\min_{\mathbf{x}_{0:T},\,\mathbf{u}_{0:T-1},\,\boldsymbol{\lambda}_{0:T-1},\,\text{contact variables}} J
$$

subject to

$$
\mathbf{x}_{t+1} = f(\mathbf{x}_t, \mathbf{u}_t, \boldsymbol{\lambda}_t),
$$

contact feasibility constraints, unilateral constraints, friction constraints, actuator limits, and some representation of contact-mode logic.

The attraction of CITO is obvious: it unifies motion planning and contact planning. The challenge is equally obvious: the problem is highly nonconvex, often nonsmooth, and combinatorial in the number of possible contact modes. For humanoid loco-manipulation, solving such a problem online remains difficult.

## 7.7 Planning methods

Three broad families appear repeatedly.

### Sampling- and search-based methods

These explicitly explore contact placements or mode sequences. They are intuitive, can be paired with task-specific heuristics, and often handle discrete contact choices naturally. Their main weakness is the explosion of the search space in high-dimensional multi-contact settings.

### Optimization-based methods

These formulate trajectory optimization or MPC problems with dynamic constraints, contact constraints, friction constraints, and task objectives. Their strength is dynamic consistency and the ability to encode rich continuous constraints. Their weaknesses are numerical sensitivity, tuning burden, and the possibility of local infeasibility.

### Learning-assisted methods

A growing line of work uses learned policies or learned heuristics to propose contact strategies, warm-start optimizers, or imitate optimized motion. These methods can be flexible and fast at runtime, but they still inherit the limitations of data coverage, safety encoding, and sim-to-real mismatch.

For graduate students, the right mental model is not that one family has “won.” The current field is hybrid. Search handles discrete structure well, optimization handles physical feasibility well, and learning can accelerate difficult subproblems or provide reactive generalization.

## 7.8 Motion planning with contact models

Once a contact sequence is known or hypothesized, the motion planner still needs an interaction model. A common choice is centroidal dynamics, because it preserves the dependence of global motion on contact forces and angular momentum. For active contacts indexed by $i$,

$$
m \ddot{\mathbf{c}} = \sum_i \mathbf{f}_i + m\mathbf{g},
$$

$$
\dot{\mathbf{L}}_c = \sum_i (\mathbf{p}_i - \mathbf{c}) \times \mathbf{f}_i + \sum_i \boldsymbol{\mu}_i.
$$

This model is especially useful for multi-contact MPC. It represents support forces explicitly while remaining smaller than the full rigid-body model. Its limitation is that it still contains nonlinear coupling between state and contact wrench variables, and it does not by itself resolve frequent contact switches as cleanly as simpler locomotion-only models.

## 7.9 A conceptual example: stair climbing with a handrail

A contact-aware planner for stair climbing with a handrail must answer several questions.

1. Which handrail contact should be used, and when?
2. Is the handrail grasp or support contact necessary for feasibility, or merely helpful for robustness?
3. Can the chosen stance foot and hand contact jointly support the required centroidal wrench during the step-up phase?
4. Is the upper-body posture kinematically compatible with the lower-body step and with self-collision avoidance?
5. How should the transition be staged so that the swing-foot touchdown does not destabilize the upper-body support contact?

The point of this example is that contact planning is fundamentally a **coupled mobility and force-feasibility** problem.


# 8. Contact-aware control

## 8.1 Why joint-space position control is not enough

Suppose a humanoid uses a purely stiff joint-space tracking controller and commands the hand to follow a path that accidentally intersects a wall. In free space, the controller behaves acceptably. At contact, however, the same stiffness can create a large unmodeled force spike.

This illustrates a central lesson: **contact tasks are not purely position tasks**. They are coupled motion–force tasks.

## 8.2 Impedance control

A standard remedy is impedance control. Rather than commanding exact position, the controller imposes a desired dynamic relation between displacement and force. In task space, a simple impedance law has the form

$$
\mathbf{F} = \mathbf{K}(\mathbf{x}_d - \mathbf{x}) + \mathbf{D}(\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + \mathbf{M}_d(\ddot{\mathbf{x}}_d - \ddot{\mathbf{x}}).
$$

This does not magically solve contact, but it makes the interaction behavior far more physically reasonable.

Impedance control is attractive because it:

- softens impact and contact uncertainty;
- tolerates moderate modeling errors;
- provides an intuitive behavior-level interface.

Its limitation is that whole-body balance and contact feasibility still have to be coordinated globally.

## 8.3 Admittance control

In admittance control, measured force is converted into motion adjustment. This is often useful when the inner actuation loop is position-controlled and one wants to realize compliant behavior by modifying motion commands based on sensed interaction force.

Impedance and admittance are dual in spirit. The right choice depends on the underlying actuation architecture and what variables can be commanded reliably.

## 8.4 Hybrid force–motion control

Certain directions at a contact are constrained and should be force-regulated, while others are free and should remain motion-regulated. This leads to hybrid force–motion control.

A classic example is a hand sliding along a wall:

- along the wall normal direction, regulate contact force;
- along the tangential direction, regulate motion.

For humanoids, this principle generalizes to support contacts and whole-body tasks. One never truly controls “position only” or “force only” everywhere. The controller partitions task directions according to contact geometry and task intent.

## 8.5 Whole-body control

Whole-body control (WBC) is the dominant framework for torque-controlled humanoids. The idea is to solve for generalized accelerations, torques, and contact forces that satisfy rigid-body dynamics while tracking multiple tasks.

Typical tasks include:

- CoM acceleration or momentum regulation;
- base orientation stabilization;
- swing-foot tracking;
- hand pose or force tracking;
- posture regularization;
- contact force shaping.

A typical optimization-based WBC problem can be written as

$$
\min_{\dot{\mathbf{v}}, \boldsymbol{\tau}, \boldsymbol{\lambda}} \sum_k w_k \lVert \mathbf{A}_k \dot{\mathbf{v}} - \mathbf{b}_k \rVert^2 + \rho_\tau \lVert \boldsymbol{\tau} \rVert^2 + \rho_\lambda \lVert \boldsymbol{\lambda} \rVert^2
$$

subject to

$$
\mathbf{M}\dot{\mathbf{v}} + \mathbf{h} = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{J}_c^T\boldsymbol{\lambda},
$$

$$
\mathbf{J}_c \dot{\mathbf{v}} + \dot{\mathbf{J}}_c \mathbf{v} = \mathbf{0},
$$

plus friction and torque constraints.

A hierarchical controller may instead solve tasks in strict priority, ensuring that lower-priority posture objectives never violate higher-priority balance constraints.

## 8.6 Momentum control

Many humanoid controllers regulate linear and angular momentum, often through centroidal dynamics. A momentum controller effectively determines what net contact wrench is required to realize a desired CoM and angular momentum evolution. A lower-level controller then distributes this wrench across the active contacts while respecting friction and contact geometry.

This split is powerful because it mirrors the physics:

- centroidal dynamics decides what the whole body must do globally;
- whole-body inverse dynamics decides how the limbs and contacts realize it.

## 8.7 Internal forces in multi-contact

In multi-contact settings, some contact force variations do not change the net external wrench on the body. These are **internal forces**. They are common in bracing and grasp-like support configurations.

Examples:

- pressing simultaneously with both hands against opposite walls;
- redistributing load between two feet without changing net CoM acceleration;
- maintaining preload in a handrail support while the net support wrench remains unchanged.

Internal-force regulation matters because it affects:

- slip margin;
- structural load distribution;
- stability of contact transitions;
- safety of environment interaction.

## 8.8 Model predictive control

Model predictive control (MPC) is often used at the centroidal or reduced-order level. Over a finite horizon, it optimizes future contact forces, CoM motion, or footsteps under simplified dynamics and constraints.

MPC is attractive because it naturally handles preview information. A robot that knows where the next foothold is can regulate not only the current state but also the future contact transition.

A common architecture is:

- high-rate whole-body inverse dynamics or QP controller;
- lower-rate MPC generating desired momentum, CoM, or contact-force references.

## 8.9 Tactile feedback in low-level control and action monitoring

The control implications of tactile sensing go beyond state estimation. Tactile information can close the loop at several time scales.

At the fastest time scale, tactile feedback supports **servoing and reflexes**. A hand controller may regulate contact pressure while tracing a surface, a grasp controller may increase normal force when incipient slip is detected, and a support controller may react to unexpected pressure redistribution under the foot.

At a slower time scale, tactile information supports **action monitoring**. The controller may decide that a contact transition has completed, that a foothold is unsafe, that a large object has shifted against the torso, or that a hand support failed to establish the intended load path.

This viewpoint is important for humanoids because it connects tactile sensing to hybrid control logic. A controller is not only solving for continuous torques. It is also deciding when to switch contact modes, when to escalate compliance, when to replan, and when to terminate or retry an action.

## 8.10 Contact transition control

Many failures happen not during steady contact, but during transitions:

- touchdown impact excites unmodeled vibrations;
- liftoff happens before weight has been fully transferred;
- a hand contact is created too quickly and causes an impulse-like disturbance;
- a nominally rigid contact actually slips during load transfer.

A good contact transition strategy therefore includes:

- touchdown velocity shaping;
- gradual force ramping;
- compliance at landing;
- transition-aware estimator logic;
- controller task switching with hysteresis, not instantaneous binary jumps.

> **Implementation principle.** If a humanoid controller looks good in steady-state plots but fails on hardware, the first place to inspect is often not the stance controller itself but the contact transition logic.


# 9. From theory to implementation

## 9.1 A practical control stack

A robust humanoid implementation usually contains at least five layers.

### Layer 1: perception and state estimation

Estimate floating-base state, joint state, contact state, and terrain information.

### Layer 2: contact-mode logic

Determine which contacts are intended, which are detected, and which are trusted.

### Layer 3: planner

Generate footsteps, contact schedules, and reduced-order motion references.

### Layer 4: whole-body controller

Convert references into dynamically feasible accelerations, torques, and contact forces.

### Layer 5: low-level actuation

Track torque or position commands with motor-level control, rate limiting, and safety checks.

The failure mode of many student projects is to implement Layer 4 beautifully while Layers 1, 2, and 5 remain underdeveloped. In practice, those layers are often what determines whether the controller survives first hardware contact.

## 9.2 What to model explicitly in simulation

A simulator is not useful merely because it has rigid-body dynamics. It is useful when it represents the uncertainties that dominate closed-loop behavior.

For humanoid contact tasks, the most important items are often:

- actuator delay and bandwidth;
- torque saturation;
- contact friction coefficients and their variation;
- foot geometry and collision shapes;
- terrain height and compliance uncertainty;
- sensor noise and estimator latency.

MuJoCo's documentation is especially useful for understanding how the solver handles frictional contact, friction dimensions, and the distinction between elliptic and pyramidal cones. Isaac Lab's documentation is useful for understanding the semantics of physics-based contact sensing and how contact-force signals are exposed to the learning or control code.

The lesson is that “high-fidelity” does not always mean visually rich. It means **faithful to the failure modes that matter**.

## 9.3 Choosing a contact model

In simulation, contact is always an approximation. The right approximation depends on the research question.

### For control prototyping

Use a stable rigid contact model with conservative friction and simple foot geometry. Prioritize solver stability and repeatability.

### For locomotion learning

Randomize friction, restitution, delays, and terrain. The goal is not exact prediction of one run but robustness of the policy across plausible conditions.

### For contact-rich manipulation or assembly

Contact patch fidelity, geometry resolution, and tangential compliance become much more important.

## 9.4 Numerical conditioning and debugging

A few implementation issues recur constantly.

### Ill-conditioned Jacobians

Near singular configurations or poor contact geometry, the contact Jacobian or task Jacobians can become ill-conditioned. Regularization is not optional.

### Inconsistent contact assumptions

If the estimator assumes a stance foot is rigidly fixed while the controller simultaneously commands a motion inconsistent with that assumption, the entire stack will become brittle.

### Overconstrained optimization

Students often write a QP with too many hard constraints and then wonder why it becomes infeasible. A practical rule is: dynamics and true contact feasibility should be hard; many task objectives should be soft.

### Timing mismatch

If the planner, estimator, and WBC run at different rates without consistent buffering or timestamp handling, contact transitions become unreliable.

## 9.5 Debugging strategy

A productive debugging sequence is:

1. verify free-space rigid-body dynamics without contact;
2. verify a single static stance contact;
3. verify CoP and normal force consistency in quiet standing;
4. verify friction margin under small horizontal pushes;
5. verify one swing-foot touchdown event;
6. only then attempt continuous walking or loco-manipulation.

In other words, do not debug “walking” first. Debug the physics of one reliable contact, then one reliable transition.

## 9.6 Sim-to-real understanding

The sim-to-real gap in humanoid contact is rarely due to one missing coefficient. It usually arises from **structural mismatch**:

- contact occurs at the wrong time because of estimator delay;
- foot compliance changes impact and CoP evolution;
- the actuator cannot realize the torque-rate implied by the solver;
- unmodeled friction hysteresis or slip changes the support wrench.

A useful engineering mindset is to treat contact robustness as a distributional problem, not a nominal-model problem. This is why modern simulation platforms emphasize domain randomization, contact sensing, and scalable training of robust locomotion and manipulation behaviors.

# 10. Research directions and open problems

## 10.1 Learning contact-rich loco-manipulation

Recent research has increasingly combined model-based trajectory optimization with learning-based tracking or policy synthesis for multi-contact loco-manipulation. The important conceptual shift is that learning is no longer used only for low-level reactive control; it is increasingly used to absorb contact uncertainties and recover from disturbances while preserving a structured task formulation.

A strong research direction is therefore not “replace mechanics with learning,” but rather:

> use mechanics to define the right structure, then use learning to supply robustness, adaptation, and perception-conditioned behavior inside that structure.

## 10.2 Tactile and distributed contact sensing

Humanoid contact control still relies heavily on sparse force–torque sensing and indirect inference. Rich tactile sensing on feet, shins, forearms, and hands could significantly improve:

- contact localization;
- early slip detection;
- estimation of partial contact patches;
- safer human contact;
- compliant whole-body bracing.

This is one of the clearest ways in which humanoid contact and dexterous manipulation research are converging.

## 10.3 Beyond rigid contact

Rigid contact is a powerful abstraction, but many important behaviors involve compliance:

- rubber soles deform;
- carpets and mats compress;
- handrails flex;
- objects being manipulated deform;
- soft protective covers alter impact and friction.

A major open challenge is how to incorporate compliant or deformable contact into real-time control without making the model intractable.

## 10.4 Contact-aware embodied intelligence

Large perception models and vision-language-action systems are drawing significant attention, but physical competence still bottlenecks on contact. A humanoid that understands a command semantically but cannot make and regulate contact reliably is not yet a physically capable agent.

This suggests a broader research thesis:

- perception and language determine **what** the robot should do;
- contact-aware planning and control determine **whether it can do it physically**.

A mature humanoid intelligence stack will require both.

# 11. Summary of key takeaways

The main lessons of this manuscript can be summarized as follows.

1. **Contact is the mechanical foundation of humanoid behavior.** Without contact, the floating-base humanoid cannot support itself or manipulate the environment in a physically meaningful way.

2. **Contact introduces both forces and constraints.** The same contact that supports the robot also restricts its motion and imposes unilateral and frictional feasibility conditions.

3. **Humanoid contact is hybrid.** Continuous within-mode dynamics are interrupted by discrete mode changes such as touchdown and liftoff.

4. **The core modeling equation is constrained floating-base dynamics.**

   $$
   \mathbf{M}(\mathbf{q}) \dot{\mathbf{v}} + \mathbf{h}(\mathbf{q},\mathbf{v}) = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c(\mathbf{q})^T \boldsymbol{\lambda},
   $$

   together with acceleration-level contact constraints.

5. **Balance is a wrench-feasibility problem, not only a geometry problem.** The support polygon and ZMP are useful, but centroidal dynamics and contact wrench feasibility provide the more general viewpoint.

6. **Estimation matters as much as control.** Incorrect contact assumptions can destabilize an otherwise correct controller.

7. **Planning is often planning over contact modes.** Where and when the robot touches the world is as important as the trajectory it follows.

8. **Whole-body control is the natural implementation framework.** It coordinates contact constraints, task objectives, torque limits, and force feasibility in one optimization problem.

9. **Good implementation requires modeling the right uncertainties.** Actuator delays, friction variation, transition logic, and sensor semantics usually matter more than symbolic elegance.

10. **The future lies in structured integration.** The strongest direction is not purely model-based or purely learned, but contact-aware systems that combine physics, optimization, sensing, and data-driven robustness.

# 12. Suggested implementation roadmap for students

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

## Stage 3: Reduced-order balance controller

- Implement a CoM or LIPM-based controller.
- Add ZMP or capture-point reasoning.
- Test under pushes and friction variation.

## Stage 4: Whole-body inverse dynamics

- Solve for $(\dot{\mathbf{v}}, \boldsymbol{\tau}, \boldsymbol{\lambda})$ with one swing-foot task and posture regularization.
- Add torque limits and a friction pyramid.

## Stage 5: Contact transitions

- Implement touchdown detection and force ramping.
- Evaluate robustness to timing errors and terrain height mismatch.

## Stage 6: Multi-contact or loco-manipulation

- Add hand support, a door interaction, or stair climbing with a railing.
- Compare pure model-based tracking against a learning-based residual or policy.

# References for further study

The following references are particularly useful for deeper study and implementation.

1. Russ Tedrake, *Underactuated Robotics* (MIT course notes), especially the chapters on contact, multibody dynamics, and highly articulated legged robots.
2. David E. Orin, Ambarish Goswami, and Sung-Hee Lee, “Centroidal dynamics of a humanoid robot,” *Autonomous Robots*, 2013.
3. Siyuan Feng, Eric Whitman, X. Xinjilefu, and Christopher G. Atkeson, “Optimization-based full body control for the DARPA Robotics Challenge,” *Journal of Field Robotics*, 2015.
4. Justin Carpentier and Nicolas Mansard and related work on whole-body inverse dynamics and constrained rigid-body control for humanoids.
5. Justin Koenemann, Andrea Del Prete, Stéphane Caron, et al., “Whole-body model predictive control applied to the HRP-2 humanoid,” IROS 2015.
6. Haiyang Dai, Andres Valenzuela, and Russ Tedrake, “Whole-body motion planning with centroidal dynamics and full kinematics,” Humanoids 2014.
7. Yu-Chi Lin, Brahayam Ponton, Ludovic Righetti, and Dmitry Berenson, “Efficient humanoid contact planning using learned centroidal dynamics prediction,” 2018.
8. Pietro Ferrari, Luca Rossini, Francesco Ruscelli, et al., “Multi-contact planning and control for humanoid robots,” *Robotics and Autonomous Systems*, 2023.
9. Jean-Pierre Sleiman, Mayank Mittal, and Marco Hutter, “Guided reinforcement learning for robust multi-contact loco-manipulation,” CoRL 2024 / PMLR 2025.
10. MuJoCo documentation, especially the sections on modeling and computation of frictional contact.
11. Isaac Lab documentation, especially the sections on contact sensors and physics-based sensor abstractions.