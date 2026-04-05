# Class 11 on Humanoid Movement

## Part 1: Introduction and The Embodiment Gap

### 1.1 Defining the Core Problem

The fundamental challenge in transferring human motion to humanoid robots is the embodiment gap. Human subjects and target humanoids possess distinct topological and morphological differences, including variances in bone length, joint range of motion, kinematic structure, body shape, and mass distribution.

When driving humanoid motion—whether through offline datasets or real-time spatial computing streams (such as optical motion capture from iOS mobile devices)—direct joint-to-joint mapping fails. It introduces critical artifacts like foot sliding, ground penetration, and physically impossible self-intersections. Overcoming this requires casting motion retargeting as an optimization problem governed by rigorous mathematical structures.

### 1.2 Contextualizing the Pipeline

Modern humanoid control relies on a three-tiered architecture:

1. State Representation: Modeling spatial movement on continuous transformation manifolds.
2. Kinematic Optimization: Abstracting variables and costs into modular, differentiable solvers.
3. Applied Retargeting: Scaling and projecting source human data onto the target robot's kinematic chain to generate artifact-free references for downstream hierarchical controllers or reinforcement learning (RL) policies.

## Part 2: Micro Lie Theory for Humanoid State Representation

- [A Micro Lie Theory for State Estimation in Robotics](https://arxiv.org/pdf/1812.01537)
    - Joan Sola, Jeremie Deray, Dinesh Atchuthan 

Humanoid movement does not occur in simple Euclidean space; it involves complex 3D rotations and translations. To handle uncertainties, derivatives, and integrals precisely, we rely on Lie theory.

### 2.1 Smooth Manifolds and Lie Groups

A Lie group $\mathcal{G}$ is a smooth manifold whose elements satisfy the fundamental group axioms: closure under composition, identity $\mathcal{E}$, inverse, and associativity.

- Rotations: Modeled on $SO(3)$, the group of special orthogonal matrices.
- Rigid Motions: Modeled on $SE(3)$, combining rotations and translations into $4 \times 4$ transformation matrices.

The smoothness of the manifold ensures the existence of a unique tangent space at each point, which is a linear space where standard calculus applies.

### 2.2 Tangent Spaces and the Lie Algebra

The tangent space evaluated at the group's identity $\mathcal{E}$ is called the Lie algebra, denoted as $\mathfrak{m}$.

- Mathematically: $\mathfrak{m} \triangleq T_{\mathcal{E}}\mathcal{M}$.
- Elements of the Lie algebra are isomorphic to Cartesian vectors in $\mathbb{R}^{m}$, mapped via the linear operators $(\cdot)^{\wedge}$ (hat) and $(\cdot)^{\vee}$ (vee). For example, in $SO(3)$, a 3D angular velocity vector maps to a $3 \times 3$ skew-symmetric matrix.

### 2.3 Calculus on Manifolds and Uncertainty

To integrate motion or compute geometric errors, we map elements between the linear tangent space and the curved manifold:

- Exponential Map ($exp$): Exactly converts elements of the Lie algebra into elements of the group, intuitively wrapping the tangent element along the manifold's geodesic.
- Logarithmic Map ($log$): The inverse unwrapping operation.

To manipulate local tangent perturbations, we define the right-plus $\oplus$ and right-minus $\ominus$ operators:
$$
\mathcal{Y} = \mathcal{X} \oplus ^{\mathcal{X}}\tau \triangleq \mathcal{X} \circ Exp(^{\mathcal{X}}\tau) \in \mathcal{M}
$$

**Uncertainty Modeling**: State uncertainty is defined rigorously on the tangent space using the expectation operator:

$$
\Sigma_{\mathcal{X}} \triangleq \mathbb{E}[(\mathcal{X} \ominus \overline{\mathcal{X}})(\mathcal{X} \ominus \overline{\mathcal{X}})^{\top}]
$$

This allows us to construct mathematically sound Gaussian variables on manifolds, preventing the singularities (like gimbal lock) associated with Euler angles.

## Part 3: Modular Kinematic Optimization 

- [PyRoki: A Modular Toolkit for Robot Kinematic Optimization](https://arxiv.org/abs/2505.03728)
    - Chung Min Kim, Brent Yi, Hongsuk Choi, Yi Ma, Ken Goldberg, Angjoo Kanazawa

ArchitectureWith the state space formally defined, we can cast inverse kinematics (IK) and trajectory planning as nonlinear least-squares optimization problems.

### 3.1 The Modular Paradigm (PyRoki Framework)

Advanced toolkits like PyRoki separate optimization variables (e.g., joint configurations $q$) from cost functions, creating reusable components that apply seamlessly to single-frame IK, sequential trajectory optimization, and motion retargeting.

### 3.2 Mathematical Modeling of Cost Functions

- Joint Pose Cost: Penalizes the deviation between the current and target base poses. Using the Lie group logarithm ensures geometric fidelity:

$$
\hat{c}_{pose}(q, T_{base\_target}) = log(T_{base\_target}^{-1}T_{base\_i})
$$

```math
\hat{c}_{\text{pose}}(\mathbf{q}, \mathbf{T}_{\text{base\_target}}) = \log\left( \mathbf{T}_{\text{base\_target}}^{-1} \mathbf{T}_{\text{base\_i}}(\mathbf{q}) \right)
```

$$
\hat{c}_{\text{pose}}(\mathbf{q}, \mathbf{T}_{\text{base\_target}}) = \log\left( \mathbf{T}_{\text{base\_target}}^{-1} \mathbf{T}_{\text{base\_i}}(\mathbf{q}) \right)
$$

- Manipulability Cost: Maximizes Yoshikawa's manipulability measure to keep the robot away from singularities, utilizing the manipulator Jacobian $J_{i}(q)$:

$$
\hat{c}_{manip}(q, i) = (\sqrt{det(J_{i}(q)J_{i}(q)^{T})} + \epsilon)^{-1}
$$

- Collision Avoidance: Signed distances $d$ between collision geometries (e.g., capsules/spheres) are computed and converted into costs. A smooth activation function avoids discontinuities at $d=0$:

$$
d_{c} = \begin{cases} 
-d + 0.5\eta & \text{if } d < 0 \\ 
\frac{0.5}{\eta}(-d + \eta)^{2} & \text{if } 0 < d < \eta \\ 
0 & \text{otherwise} 
\end{cases}
$$

### 3.3 Numerical Solvers

Optimization is driven by quasi-Newton approaches, specifically the **Levenberg-Marquardt (LM) optimizer**, which accelerates convergence using cost curvature approximations. By utilizing frameworks like JAX, we can compute block-sparse Jacobian matrices via automatic differentiation, enabling massive parallelization across GPUs and TPUs.

## Part 4: Practical Differential IK with MuJoCo and `mink`

- [Mink: Python inverse kinematics based on MuJoCo](https://github.com/kevinzakka/mink)
    - Kevin Zakka

Translating modular costs into real-time environments requires highly efficient computational backends. The Python library `mink` serves as an industry-standard differential IK solver built explicitly on the MuJoCo physics engine.

### 4.1 Integration of Micro Lie Theory

`mink` natively incorporates the rigid body transformation mathematics derived from Micro Lie Theory. It features a Lie group interface with C-optimized hot paths, ensuring that high-frequency control loops can execute Jacobian computations at sub-millisecond speeds.

### 4.2 Key Capabilities

- **Constraint Enforcement**: It natively supports strict limits on joint positions and velocities, along with real-time collision avoidance between any geometric pair utilizing MuJoCo's collision engine.

- **Closed-Chain Kinematics**: By leveraging equality constraints, it can resolve complex loop closures necessary for stable bipedal locomotion and bimanual manipulation.

## Part 5: General Motion Retargeting (GMR)

- [Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking](https://arxiv.org/pdf/2510.02252)
    - Joao Pedro Araujo, Yanjie Ze, Pei Xu, Jiajun Wu, C. Karen Liu

To resolve the embodiment gap introduced in Part 1, we apply the GMR methodology.

### 5.1 Kinematic Scaling

Most retargeting artifacts are introduced during the scaling of the source motion. GMR calculates a general scaling factor based on human height, which adjusts a custom local scale factor $s_{b}$ defined for each key body (e.g., distinguishing upper and lower body proportions). The target body position in Cartesian space is:

$$
p_{b}^{target} = \frac{h}{h_{ref}}s_{b}(p_{j}^{source} - p_{root}^{source}) + \frac{h}{h_{ref}}s_{root}p_{root}^{source}
$$

### 5.2 Two-Stage Differential IK Optimization

To avoid local optimization minima, GMR adopts a two-stage differential IK process:

- **Stage 1 (Rotation & End-Effectors)**: Computes generalized velocities $\dot{q}$ to minimize orientation errors and end-effector positions.
- **Stage 2 (Fine-Tuning)**: Uses the Stage 1 solution as an initial guess to minimize comprehensive translation and rotation constraints across all key bodies simultaneously.

## Part 6: Applied Impacts and Ecosystem Integration

### 6.1 Enhancing Sim-to-Real Transfer

Artifacts in retargeted reference trajectories—such as physically impossible self-penetration or abrupt velocity spikes—severely reduce the robustness of motion tracking RL policies. By employing the GMR pipeline integrated with a robust solver, policies achieve significantly higher success rates and perceptual fidelity without relying on extensive, trial-and-error reward shaping.

### 6.2 Ecosystem Deployment: The asMagic Case Study

The theoretical models discussed here form the foundational backend for scalable app ecosystems like asMagic. When utilizing the Apple ecosystem (e.g., iPhone sensor suites) for spatial tracking and motion capture, raw human pose data is noisy and kinematically disjoint from target hardware (like a quadruped or a humanoid).

By streaming this data through a GPU-accelerated PyRoki/mink pipeline running GMR scaling, the asMagic system can instantaneously translate human gestures into safe, collision-free, and dynamically feasible robot joint commands. This bridges human intent with embodied AI execution in real-time, proving the applied value of mathematically rigorous kinematic optimization.