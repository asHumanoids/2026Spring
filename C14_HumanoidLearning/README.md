# Class 14 on Humanoid Learning

## Contents

1. Motivation and scope  
2. Why humanoid learning is a distinct problem  
3. Formal problem formulations  
4. Policy representations and interfaces to the control stack  
5. Reinforcement learning for humanoid motor skill acquisition  
6. Imitation learning and motion imitation  
7. Learning from robot data and from human data  
8. Hybrid model-learning methods  
9. Versatile skill learning and multi-skill policies  
10. From skill learning to foundation models for humanoids  
11. Multimodal and contact-aware learning  
12. Sim-to-real transfer and deployment  
13. Evaluation, failure modes, and open problems  

---

# 1. Motivation and scope

Humanoid robots promise a particular route to embodied intelligence: they can locomote in spaces designed for humans, manipulate human-scale objects, exploit whole-body contact, and absorb structure from human behavior more directly than robots with strongly different morphologies. The attraction of humanoid learning is therefore not only that learning may produce agile control policies, but that humanoid embodiment may allow a robot to absorb **human skill, human intent, and human semantics** at scale. The survey highlights this point explicitly: humanoids can exploit abundant data available **for and from humans**, making them a promising platform for embodied intelligence. 

At the same time, humanoid learning is unusually difficult. A humanoid is a high-dimensional, floating-base, contact-rich, safety-critical dynamical system operating in partially observable environments. A useful learning algorithm must therefore do more than map observations to actions. It must discover or imitate policies that remain valid under changes of contact, command, terrain, payload, observation quality, and embodiment mismatch. Purely generic machine learning abstractions are rarely enough.

The central question of these notes is therefore:

> How can a humanoid robot learn robust, versatile, and generalizable skills while respecting the physical structure imposed by embodiment, contact, and task semantics?

The answer pursued in modern research is not a single algorithm. Instead, the field is organized around a ladder:

1. **reinforcement learning** for discovering motor skills from trial and error;
2. **imitation learning** for leveraging demonstrations efficiently;
3. **robot-data and human-data pipelines** for scaling behavior acquisition;
4. **hybrid methods** that combine models, optimization, and learning;
5. **multi-skill and multimodal policies** for broader competence;
6. **foundation models** for semantic generalization, planning, and task-level reasoning. 

The present notes emphasize the learning side of this ladder. Contact mechanics, full floating-base control, WBC, MPC, and detailed tactile hierarchies are treated only as prerequisites or interfaces where necessary.

---

# 2. Why humanoid learning is a distinct problem

Humanoid learning is not simply manipulation learning on a larger arm, nor locomotion learning on a biped with more joints. Several structural properties make it distinct.

## 2.1 High dimensionality and embodiment complexity

A humanoid typically contains tens of actuated degrees of freedom, plus an unactuated floating base and possibly dexterous hands. The action may be:

- joint torque,
- joint velocity target,
- joint position target,
- impedance parameter,
- latent skill command,
- or a residual correction on top of a nominal controller.

The observation may include:

- base pose and twist,
- joint positions and velocities,
- contact states or contact estimates,
- terrain features,
- object states,
- visual features,
- tactile features,
- language instructions,
- and internal memory states.

This combination creates both a large representation problem and a difficult exploration problem.

## 2.2 Underactuation and contact switching

Unlike fixed-base manipulators, humanoids regulate the base indirectly through contact. A learned locomotion or loco-manipulation policy must therefore account for hybrid transitions such as foot touchdown, slip, disturbance rejection, grasp creation, and support transfer. The survey emphasizes that loco-manipulation remains especially challenging for humanoids because balance margins are smaller and the interaction between locomotion and manipulation must be coordinated rather than separated. 

## 2.3 Human data is unusually informative

The survey highlights that human data are especially valuable for humanoids because their morphology is comparatively close to the human body, making skill transfer more natural than for many other robot embodiments. This is one of the key reasons humanoids are central to current embodied intelligence research. 

## 2.4 Safety and hardware cost constrain exploration

A failed humanoid rollout can damage the robot, the object, or nearby people. Consequently, large portions of humanoid learning rely on simulation, privileged information during training, offline datasets, demonstration priors, and safety-preserving deployment strategies.

## 2.5 Skill learning is no longer the whole story

The survey places **foundation models** after skill learning for a reason: high-quality motor skills are necessary but not sufficient. A generalist humanoid also requires semantic understanding, task decomposition, and long-horizon reasoning. The modern learning problem therefore spans both **sensorimotor competence** and **task-level generalization**. 

---

# 3. Formal problem formulations

Theoretical depth in humanoid learning begins with the right formalization. Different algorithms solve different abstractions of the same physical problem.

## 3.1 Dynamical system and observation model

Let the humanoid-environment system evolve according to

$$
\mathbf{x}_{t+1} = f\big(\mathbf{x}_t, \mathbf{u}_t, \mathbf{w}_t\big),
$$

where:

- $\mathbf{x}_t$ is the latent physical state,
- $\mathbf{u}_t$ is the low-level control command delivered to the plant,
- $\mathbf{w}_t$ captures disturbances, contact uncertainty, modeling error, and process noise.

The learning system usually does not observe $\mathbf{x}_t$ directly. Instead,

$$
\mathbf{o}_t = h\big(\mathbf{x}_t, \mathbf{v}_t\big),
$$

where $\mathbf{o}_t$ is the observation and $\mathbf{v}_t$ is sensor noise.

A learned policy parameterized by $\theta$ then maps observations and possibly an internal memory state $\mathbf{m}_t$ to an action:

$$
\mathbf{a}_t \sim \pi_\theta(\cdot \mid \mathbf{o}_t, \mathbf{m}_t),
\qquad
\mathbf{m}_{t+1} = g_\theta(\mathbf{m}_t, \mathbf{o}_t, \mathbf{a}_t).
$$

The action $\mathbf{a}_t$ need not equal the physical command $\mathbf{u}_t$. Often there is a policy-to-actuator interface such as

$$
\mathbf{u}_t = \phi\big(\mathbf{a}_t, \mathbf{s}^{\text{ctrl}}_t\big),
$$

where $\phi$ may represent a PD tracker, impedance law, or low-level stabilizer.

## 3.2 MDP and POMDP views

If the full state is observable and Markovian, the problem can be written as an MDP

$$
\mathcal{M} = (\mathcal{S}, \mathcal{A}, P, r, \gamma).
$$

The objective is to maximize expected discounted return:

$$
J(\pi) = \mathbb{E}_{\tau \sim \pi}\left[ \sum_{t=0}^{T-1} \gamma^t r_t \right].
$$

For most real humanoid problems, however, a **POMDP** is more appropriate:

$$
\mathcal{P} = (\mathcal{S}, \mathcal{A}, P, \mathcal{O}, \Omega, r, \gamma),
$$

where $\Omega(\mathbf{o}_t \mid \mathbf{s}_t)$ is the observation model. This matters because contact, terrain, object properties, and actuator state are only partially observed.

## 3.3 Contextual and goal-conditioned formulations

Many humanoid skills are not single-task policies but families of behaviors indexed by a command or goal variable $\mathbf{c}$. The policy is then

$$
\pi_\theta(\mathbf{a}_t \mid \mathbf{o}_t, \mathbf{c}),
$$

where $\mathbf{c}$ may encode:

- desired walking velocity,
- target pose,
- push direction,
- manipulation goal,
- or language/task context.

This yields a **contextual MDP** or **goal-conditioned policy**. Such formulations are fundamental for commanded locomotion, motion tracking, and multi-skill learning.

## 3.4 Hierarchical skill-learning formulations

Humanoid learning often benefits from hierarchy. Let a high-level policy choose a latent skill variable $\mathbf{z}_k$ at a slower time scale, and a low-level policy execute motor commands at every time step:

$$
\mathbf{z}_k \sim \pi^{\text{hi}}(\mathbf{z}_k \mid \mathbf{o}_{t_k}),
$$

$$
\mathbf{a}_t \sim \pi^{\text{lo}}(\mathbf{a}_t \mid \mathbf{o}_t, \mathbf{z}_k), \qquad t_k \le t < t_{k+1}.
$$

Here the latent variable $\mathbf{z}_k$ can represent a maneuver, gait mode, task phase, or abstract skill embedding.

## 3.5 Data-distribution view

For imitation and offline learning, the key object is not only a policy but also a dataset

$$
\mathcal{D} = \{(\mathbf{o}_t, \mathbf{a}_t, r_t, \mathbf{o}_{t+1}, d_t)\}_{t=1}^N.
$$

The dataset may come from:

- robot rollouts,
- motion-capture retargeting,
- teleoperation,
- scripted controllers,
- optimization-based supervisors,
- or human videos with inferred actions.

Distribution mismatch between training data and deployed rollouts is a central issue throughout humanoid learning.

---

# 4. Policy representations and interfaces to the control stack

A major design choice in humanoid learning is **what the policy outputs** and **where it sits relative to the control stack**.

## 4.1 Action-space choices

Common action spaces include:

1. **Torque actions**

$$
\mathbf{u}_t = \mathbf{a}_t \in \mathbb{R}^{n_u}.
$$
   
   This is expressive but hard to train and fragile under sim-to-real mismatch.

3. **Position-target actions**

$$
\mathbf{u}_t = \mathbf{K}_p(\mathbf{q}^{\star}_t - \mathbf{q}_t) - \mathbf{K}_d \dot{\mathbf{q}}_t,
\qquad \mathbf{q}^{\star}_t = \mathbf{a}_t.
$$

   This is common in simulation because it regularizes the behavior through a low-level tracker.

4. **Velocity-target actions**
   Useful for lower-level servo layers or reduced-order locomotion.

5. **Residual actions**

$$
\mathbf{u}_t = \mathbf{u}^{\text{nom}}_t + \Delta \mathbf{u}_t,
\qquad \Delta \mathbf{u}_t = \pi_\theta(\mathbf{o}_t).
$$

   This is important when a model-based or hand-designed controller already provides a stable baseline.

6. **Latent skill actions**

$$
\mathbf{z}_t = \pi_\theta(\mathbf{o}_t),
$$

   followed by a skill decoder or low-level motor controller.

## 4.2 Observation design

A useful humanoid observation is rarely just “all states.” It is usually structured into:

- proprioception,
- command or goal variables,
- exteroception,
- contact features,
- history or memory,
- and sometimes privileged training-only variables.

A common observation partition is

$$
\mathbf{o}_t = \big[\mathbf{o}^{\text{prop}}_t,\; \mathbf{o}^{\text{cmd}}_t,\; \mathbf{o}^{\text{exo}}_t,\; \mathbf{o}^{\text{hist}}_t\big].
$$

Good policy learning depends heavily on the choice of coordinate frames, normalization, and whether the observation is body-centric or world-centric.

## 4.3 Interfaces to WBC and MPC

To avoid overlap with the control lecture, we state only the interface logic.

A learned module may operate:

- **below** WBC/MPC, replacing low-level tracking;
- **beside** WBC/MPC, providing residuals or parameter adaptation;
- **above** WBC/MPC, providing commands, references, or contact suggestions.

The control lecture already develops the predictive-reactive hierarchy in detail. Here the important point is conceptual: learning can enter the stack at different levels, and the correct insertion point strongly affects safety, data efficiency, and interpretability. The survey likewise treats learning as complementing, not simply replacing, the existing model-based hierarchy. 

---

# 5. Reinforcement learning for humanoid motor skill acquisition

The survey identifies reinforcement learning as a major driver of recent humanoid progress, while also emphasizing that pure RL without demonstrations is often prohibitively inefficient for high-DoF loco-manipulation problems. 

## 5.1 Objective and value functions

For a stochastic policy $\pi_\theta$, RL seeks

$$
\max_\theta J(\pi_\theta) = \max_\theta \; \mathbb{E}_{\tau \sim \pi_\theta}\left[\sum_{t=0}^{T-1} \gamma^t r_t\right].
$$

The state-value and action-value functions are

$$
V^\pi(\mathbf{s}) = \mathbb{E}\left[\sum_{t=0}^{\infty} \gamma^t r_t \mid \mathbf{s}_0 = \mathbf{s} \right],
$$

$$
Q^\pi(\mathbf{s},\mathbf{a}) = \mathbb{E}\left[\sum_{t=0}^{\infty} \gamma^t r_t \mid \mathbf{s}_0 = \mathbf{s}, \mathbf{a}_0 = \mathbf{a} \right].
$$

The advantage is

$$
A^\pi(\mathbf{s},\mathbf{a}) = Q^\pi(\mathbf{s},\mathbf{a}) - V^\pi(\mathbf{s}).
$$

These objects underpin policy gradient and actor-critic methods.

## 5.2 Policy gradient perspective

The policy-gradient theorem gives

$$
\nabla_\theta J(\pi_\theta)
= \mathbb{E}_{\pi_\theta}\big[ \nabla_\theta \log \pi_\theta(\mathbf{a}_t \mid \mathbf{o}_t) \; \hat{A}_t \big].
$$

In practice, the policy gradient is estimated from finite rollouts with variance reduction, clipping, generalized advantage estimation, and trust-region style updates. PPO-style methods are common in humanoid locomotion because they are stable and scalable in large-batch simulation.

## 5.3 Reward design for humanoids

A typical reward decomposes as

$$
r_t = \sum_{i=1}^m w_i r^{(i)}_t,
$$

where terms may include:

- velocity tracking,
- posture regularization,
- foot clearance,
- energy penalty,
- action smoothness,
- orientation stabilization,
- hand/object alignment,
- contact consistency,
- and survival.

This decomposition is both useful and dangerous. It enables structured skill design, but it also creates reward hacking and brittle behavior. For humanoids, reward design often becomes a form of hidden modeling.

## 5.4 Motion tracking RL

One of the most successful templates for humanoid RL is **reference motion tracking**. Let a reference trajectory be

$$
\xi^{\text{ref}}_t = (\mathbf{q}^{\text{ref}}_t, \dot{\mathbf{q}}^{\text{ref}}_t, \mathbf{c}^{\text{ref}}_t, \ldots).
$$

A reward may then penalize tracking error:

$$
r^{\text{track}}_t = \exp\!\left(-\|\mathbf{q}_t - \mathbf{q}^{\text{ref}}_t\|_{\mathbf{W}_q}^2 - \|\dot{\mathbf{q}}_t - \dot{\mathbf{q}}^{\text{ref}}_t\|_{\mathbf{W}_{\dot q}}^2\right).
$$

This can produce high-quality behaviors such as walking, running, jumping, dancing, and recovery maneuvers, provided the reference is dynamically meaningful and the policy retains enough freedom to adapt.

## 5.5 Curriculum and domain randomization

Pure exploration is often ineffective for humanoids. Two standard tools are therefore:

1. **curriculum learning**, where task difficulty increases gradually;
2. **domain randomization**, where dynamics, sensors, contact, latency, and environment parameters are randomized.

If $\boldsymbol{\psi}$ denotes environment parameters, then training effectively optimizes

$$
\max_\theta \; \mathbb{E}_{\boldsymbol{\psi} \sim p(\boldsymbol{\psi})}
\big[J(\pi_\theta; \boldsymbol{\psi})\big].
$$

This can be interpreted as robust policy optimization over a distribution of worlds.

## 5.6 Why pure RL is not enough

The survey is explicit that pure RL without demonstrations is often too sample-inefficient for humanoid loco-manipulation. The main reasons are:

- very large state and action spaces,
- sparse or delayed task rewards,
- strong contact discontinuities,
- safety constraints,
- and expensive real-world deployment. 

This motivates imitation learning, hybrid supervision, and better data pipelines.

---

# 6. Imitation learning and motion imitation

Imitation learning reduces the burden of exploration by injecting demonstration structure.

## 6.1 Behavior cloning

Given a demonstration dataset

$$
\mathcal{D} = \{(\mathbf{o}_i, \mathbf{a}_i)\}_{i=1}^N,
$$

behavior cloning solves supervised learning:

$$
\min_\theta \; \mathcal{L}_{\text{BC}}(\theta)
= -\sum_{i=1}^N \log \pi_\theta(\mathbf{a}_i \mid \mathbf{o}_i),
$$

or, for deterministic outputs,

$$
\min_\theta \; \sum_{i=1}^N \|\pi_\theta(\mathbf{o}_i) - \mathbf{a}_i\|^2.
$$

This is simple and often strong when demonstrations are close to deployment conditions.

## 6.2 Covariate shift and DAgger

Behavior cloning suffers from compounding error because the learner is trained on states visited by the expert, but deployed on states visited by its own imperfect policy.

Dataset Aggregation (DAgger) addresses this by iteratively collecting learner rollouts and relabeling them with expert actions, effectively minimizing loss under the learner-induced state distribution.

Let $\rho_\pi$ denote the state distribution induced by policy $\pi$. Then the core issue is that BC minimizes error under $\rho_{\pi_E}$, while deployment occurs under $\rho_{\pi_\theta}$.

## 6.3 Adversarial imitation and inverse objectives

Instead of cloning actions directly, one may seek a policy whose trajectory distribution resembles expert trajectories. In adversarial imitation learning,

$$
\min_\pi \max_D \; \mathbb{E}_{\tau \sim \pi_E}[\log D(\tau)] + \mathbb{E}_{\tau \sim \pi}[\log(1-D(\tau))],
$$

where $D$ is a discriminator distinguishing expert from learner behavior.

This is useful when expert actions are unavailable or when trajectory-level realism matters more than pointwise action matching.

## 6.4 Motion retargeting as a prerequisite for humanoids

For humanoids, imitation usually requires a retargeting stage from human demonstration to robot embodiment. Let $\mathbf{y}^{\text{human}}_t$ denote human features such as keypoints or joint angles, and let $\mathbf{q}^{\text{hum}}_t$ be humanoid joint coordinates. A generic retargeting objective is

$$
\min_{\mathbf{q}^{\text{hum}}_{0:T}} \sum_{t=0}^{T} \Big( \|\Phi(\mathbf{q}^{\text{hum}}_t) - \mathbf{y}^{\text{human}}_t\|_{\mathbf{W}_y}^2 + \|\mathbf{q}^{\text{hum}}_t - \mathbf{q}^{\text{hum}}_{t-1}\|_{\mathbf{W}_q}^2 \Big)
$$

subject to joint limits, balance-related constraints, collision avoidance, and sometimes contact consistency.

Retargeting is not merely kinematic bookkeeping; it is where embodiment mismatch first enters the learning pipeline.

## 6.5 Motion imitation as structured policy learning

Instead of treating demonstration as a dataset of independent samples, motion imitation often conditions the policy on a phase variable, reference motion, or future target segment. This allows a policy to execute a continuum of behaviors while still grounding the training signal in expert data.

The survey emphasizes imitation learning as an efficient route to motor skill acquisition and highlights its importance in the broader move toward versatile and generalizable humanoid behavior. 

---

# 7. Learning from robot data and from human data

A central survey insight is the contrast between **robot experience data** and **human data**. Robot data are often high-quality but expensive; human data are abundant but embodiment-mismatched. 

## 7.1 Robot data

Robot data can include:

- on-policy RL rollouts,
- teleoperation trajectories,
- optimization-generated trajectories,
- controller logs,
- contact-rich recovery episodes.

Advantages:

- actions are physically meaningful for the target robot,
- contact and timing are naturally consistent,
- labels can include privileged simulator quantities.

Disadvantages:

- data collection is expensive,
- hardware exploration is risky,
- diversity may be limited.

## 7.2 Human data

Human data can include:

- motion capture,
- wearable sensing,
- RGB videos,
- egocentric videos,
- teleoperation demonstrations,
- language annotations.

Advantages:

- abundance and diversity,
- broad task coverage,
- rich semantic context.

Disadvantages:

- no direct robot actions,
- morphology mismatch,
- missing force/contact labels,
- uncertainty in intent and environment state.

## 7.3 Cross-embodiment mapping

A useful abstraction is to suppose that human demonstrations live in a source embodiment $\mathcal{E}_H$ and the humanoid in a target embodiment $\mathcal{E}_R$. Learning then requires a mapping

$$
\Psi: \mathcal{Y}_H \rightarrow \mathcal{Y}_R,
$$

between behavior representations. The choice of representation $\mathcal{Y}$ is critical. It may be:

- joint trajectories,
- end-effector trajectories,
- contact events,
- object-relative motion,
- latent intent variables,
- or task tokens.

The more representation focuses on **task-relevant invariants** rather than raw body motion, the easier cross-embodiment transfer often becomes.

## 7.4 Why humanoids are special here

The survey explicitly argues that learning from humans is a unique advantage of humanoids because their morphology is comparatively similar to the human body. This makes human data not merely auxiliary, but potentially central to humanoid learning pipelines. 

---

# 8. Hybrid model-learning methods

The most practical humanoid systems are rarely purely model-based or purely learned. Hybrid methods dominate because they combine structure with adaptability.

## 8.1 Residual learning

Suppose a nominal controller provides $\mathbf{u}^{\text{nom}}_t$. A residual policy learns

$$
\mathbf{u}_t = \mathbf{u}^{\text{nom}}_t + \Delta \mathbf{u}_t,
\qquad
\Delta \mathbf{u}_t = \pi_\theta(\mathbf{o}_t).
$$

This reduces search burden because the policy only learns what the nominal controller fails to capture.

## 8.2 Teacher-student learning

Let a high-quality but expensive teacher policy or optimizer produce actions $\mathbf{a}^{\text{teach}}_t$. A student policy is trained by

$$
\min_\theta \sum_t \|\pi_\theta(\mathbf{o}_t) - \mathbf{a}^{\text{teach}}_t\|^2.
$$

The teacher may be:

- trajectory optimization,
- MPC,
- whole-body control with a high-level planner,
- or a strong privileged-information policy.

## 8.3 Model-based priors inside learning

Even when the final policy is learned, the training loop often uses:

- reference motion generators,
- contact/event detectors,
- reward terms derived from mechanics,
- kinematic constraints,
- retargeting solvers,
- latent world models.

Thus, “learning” in humanoid robotics often means learning *on top of* a substantial amount of robotic structure.

## 8.4 Learning-based aids for planning and contact reasoning

Although contact planning belongs mainly to the contact lecture, the survey notes a learning role that matters for humanoid learning: learned components can predict contact sequences, contact affordances, or the evolution of centroidal quantities, thereby accelerating planning and improving feasibility in contact-rich tasks. This belongs in the learning lecture because it shows how learned representations support structured decision-making without replacing mechanics. 

---

# 9. Versatile skill learning and multi-skill policies

The survey’s Section VII does not stop at single-task RL or BC. It explicitly includes **hybrid methods** and **versatile skill learning**, reflecting the field’s shift from isolated behaviors toward reusable skill libraries and generalist motor policies. 

## 9.1 Conditional multi-skill policies

A multi-skill policy can be written as

$$
\pi_\theta(\mathbf{a}_t \mid \mathbf{o}_t, \mathbf{z}),
$$

where $\mathbf{z}$ indexes a skill, motion style, task mode, or command. Training may maximize

$$
\mathbb{E}_{\mathbf{z} \sim p(\mathbf{z})}
\left[ J\big(\pi_\theta(\cdot \mid \cdot, \mathbf{z})\big) \right].
$$

Such policies can unify walking, turning, crouching, recovery, reaching, or object-carrying within a single network.

## 9.2 Latent skill spaces

Instead of discrete modes, one may learn a latent variable model

$$
p_\theta(\tau \mid \mathbf{z}), \qquad \mathbf{z} \sim p(\mathbf{z}),
$$

with an encoder-decoder or variational formulation. A downstream policy or planner can then select skill latents rather than raw control commands.

## 9.3 Skill composition

Suppose primitive skills are parameterized by $\mathbf{z}_1, \ldots, \mathbf{z}_K$. One would like to compose them into longer-horizon behaviors. This is difficult because humanoid skills interact through whole-body momentum, balance, and contact timing. Composition is therefore not simply symbolic concatenation; it must respect physical continuity.

## 9.4 Why versatility is hard

The move from one-skill-one-policy to generalist policies is hindered by:

- conflicting objectives across tasks,
- representation interference,
- contact-mode diversity,
- scale differences between locomotion and manipulation,
- and catastrophic forgetting.

This is exactly why the survey separates versatile skill learning from the earlier RL/IL categories. 

---

# 10. From skill learning to foundation models for humanoids

One of the strongest reasons to redesign this lecture around the survey is that it gives a natural bridge from low-level skill learning to **foundation-model-driven embodied intelligence**. The survey argues that foundation models are promising because they can exploit internet-scale data for semantic understanding and long-horizon reasoning, while current low-level humanoid execution remains a separate unresolved bottleneck. 

## 10.1 Why foundation models enter humanoid robotics

A humanoid operating in a human world must answer questions such as:

- What task is being requested?
- What object affords the intended action?
- What sequence of subgoals should be executed?
- Which contacts or tools are likely useful?
- How should a long-horizon task be decomposed into executable motor chunks?

These are partly semantic and compositional problems rather than low-level control problems.

## 10.2 Foundation models as high-level conditional priors

A high-level model can be viewed as producing a task context or plan token sequence

$$
\mathbf{z}^{\text{task}}_{0:K} \sim p_\phi\big(\mathbf{z}^{\text{task}}_{0:K} \mid \mathbf{y}\big),
$$

where $\mathbf{y}$ may include language, vision, scene graphs, or dialogue context. A lower-level skill policy then executes conditioned on these tokens:

$$
\mathbf{a}_t \sim \pi_\theta(\mathbf{a}_t \mid \mathbf{o}_t, \mathbf{z}^{\text{task}}_k).
$$

This formalism captures a broad family of hierarchical systems where large models operate at the task/planning layer and motor policies operate at the sensorimotor layer.

## 10.3 LLMs, VLMs, and humanoid foundation models

The survey distinguishes directions such as **LLM/VLM for humanoids** and more specialized **humanoid foundation models**. The essential point is that these models offer semantic generalization and open-world reasoning, but they have not yet solved low-level physically grounded execution end to end. In current systems, they are most credible as planners, skill selectors, affordance predictors, or semantic world models rather than direct torque generators. 

## 10.4 The central unsolved interface

The key research bottleneck is the interface between

- broad semantic competence,
- and real-time contact-rich motor execution.

This interface may be implemented through:

- skill tokens,
- language-conditioned policies,
- hierarchical planners,
- latent motor programs,
- world models with action abstractions,
- or multimodal action representations.

## 10.5 Why this belongs in the learning lecture

This material belongs here, rather than in the control lecture, because the main issue is not the regulation of dynamics but the scaling of **representation**, **data**, and **decision abstraction**. The survey explicitly frames foundation models as part of the learning-side frontier of humanoid robotics. 

---

# 11. Multimodal and contact-aware learning

The contact lecture already develops tactile/contact mechanics in detail, so here the emphasis is narrower: contact and tactile signals as **learning inputs** and **sample-efficiency aids**.

## 11.1 Multimodal policy inputs

A multimodal policy may receive

$$
\mathbf{o}_t = \big[\mathbf{o}^{\text{prop}}_t,\; \mathbf{o}^{\text{vision}}_t,\; \mathbf{o}^{\text{tactile}}_t,\; \mathbf{o}^{\text{language}}_t\big].
$$

Each modality contributes different information:

- proprioception for body state,
- vision for scene and object context,
- tactile/contact sensing for local interaction state,
- language for task specification.

## 11.2 Contact-aware learning

Learning can use contact information in several ways:

- as direct observation inputs,
- as privileged training targets,
- as event labels for phase estimation,
- as auxiliary losses,
- as cues for affordance and skill-switch prediction.

The survey notes that tactile sensing can improve dynamic reasoning about robot-object-environment interactions and may improve learning sample efficiency when integrated well. 

## 11.3 Learning under contact uncertainty

A contact-aware model may predict latent interaction variables such as

$$
\hat{\mathbf{c}}_t = g_\psi(\mathbf{o}_{0:t}),
$$

where $\hat{\mathbf{c}}_t$ may encode contact probability, contact location, support mode, or slip risk. The control stack can then use these inferred variables without requiring the learning system to solve the entire control problem end to end.

## 11.4 Why tactile learning is promising but difficult

The survey’s tactile discussion shows why this remains challenging: tactile data are high-dimensional, simulator support is limited, and dynamic whole-body reasoning under contact is still underdeveloped. Yet this is also one of the clearest routes toward richer embodied intelligence. 

---

# 12. Sim-to-real transfer and deployment

The survey emphasizes that training in simulation and transferring to hardware remains central in humanoid learning. 

## 12.1 Problem statement

Let $p_{\text{sim}}$ and $p_{\text{real}}$ denote simulation and hardware dynamics/observation distributions. Sim-to-real transfer fails when

$$
p_{\text{sim}}(\tau) \not\approx p_{\text{real}}(\tau)
$$

for trajectories relevant to deployment.

## 12.2 Sources of mismatch

Typical gaps include:

- actuator delay and saturation,
- inaccurate contact/friction models,
- terrain variability,
- joint backlash and compliance,
- sensor noise and dropout,
- state-estimation drift,
- payload uncertainty,
- and unmodeled controller implementation details.

## 12.3 Common strategies

Common strategies include:

- domain randomization,
- system identification,
- privileged training and student distillation,
- teacher-student transfer,
- residual deployment on top of safe baselines,
- action filtering and safety projection,
- staged deployment with increasing autonomy.

## 12.4 Deployment as constrained inference

A useful perspective is that deployment is not just “run the trained policy,” but rather infer whether the current observation lies inside the policy’s competence set. This motivates uncertainty estimates, monitors, fall detectors, contact failure detection, and switching logic.

---

# 13. Evaluation, failure modes, and open problems

## 13.1 Evaluation dimensions

A humanoid learning system should be judged along several axes:

- task success,
- robustness to perturbation,
- contact stability,
- energy efficiency,
- sample efficiency,
- generalization across terrain or objects,
- transfer across embodiments,
- and deployment reliability.

## 13.2 Common failure modes

Common failure modes include:

- reward exploitation,
- brittle motion tracking,
- overfitting to simulation artifacts,
- policy collapse under observation delay,
- contact hallucination or missed contact,
- poor retargeting from human motion,
- and inability to compose skills beyond the training envelope.

## 13.3 Open problems from the embodied-intelligence viewpoint

The survey motivates several open directions that are especially relevant here:

1. **single skills to embodied competence** — how to go beyond isolated walking or imitation to whole-body versatile behavior;
2. **contact-rich loco-manipulation learning** — still much harder than periodic locomotion;
3. **human-data representations** — how to preserve intent while producing executable robot behavior;
4. **foundation-model integration** — how to connect semantic understanding with low-level physical competence. 

These problems show that the frontier of humanoid learning is not merely better policy optimization. It is the integration of embodiment, representation, data, semantics, and physical execution.

---

# Closing perspective

Humanoid learning is best understood not as a competition between model-based robotics and machine learning, but as the study of how learning can scale **motor competence**, **data reuse**, and **semantic generalization** in physically grounded embodied systems.

The broad trajectory described in the survey is especially useful for organizing this field. First come the classical planning and control structures that give stability and physical meaning. Then come learning methods for **skill acquisition**: reinforcement learning, imitation learning, human-data transfer, and hybrid methods. Finally comes the effort to reach **generalist embodied intelligence** through versatile multi-skill policies and foundation-model-based task understanding. 

For humanoids, this trajectory is particularly compelling because the embodiment itself creates access to human motion, human tasks, and human semantic environments. That is why humanoid learning is now central to embodied intelligence research: it sits exactly at the interface between physical skill and general reasoning.
