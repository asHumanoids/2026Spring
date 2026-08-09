# Class 01 on Perspectives in Understanding Robots

[▶ Open the full-screen slide presentation](https://ashumanoids.github.io/2026Spring/slides/robot-perspectives/)

Use the arrow keys or swipe to navigate. Press `f` for full screen, `o` for the slide overview, and `p` for presenter view.

## Understanding Robot as a Machine, a Graph, and a Manifold

To better understand a robot, let's begin with a three-level framework:
- As a Machine from Mechanical Engineering
- As a Graph from Computer Science
- As a Manifold from Mathematics & Physics

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-1.png>)

## Level 1 in Mechanical Engineering: "The Robot as a Physical Mechanism"
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-2.png>)

At this level, the robot is viewed as a collection of matter subject to Newtonian laws. The focus is on the *constraints of reality*.

- **Core Concept: Kinematic Chains & Power Transmission**.
  - An ME sees links, bearings, gears, and motors. The primary concern is structural integrity, power density, and physical limits.
  - Key Metrics: Stiffness, Backlash, Friction, Payload-to-Weight Ratio.
- **The Representation: CAD (Computer-Aided Design)**.
  - The robot is defined by volume, density, and material properties.
  - Constraint: "This joint cannot rotate past 90 degrees because the metal casing will collide."
- **Connection to Description Formats**:
  - The ME provides the **Inertial Parameters** ($I_{xx}, I_{yy}, I_{zz}$ and Mass $m$).
  - They define **collision geometry** (the physical boundaries).
- **Limitations**: An ME model is often "too detailed" for real-time control (e.g., modeling every screw thread). It lacks the notion of "state" or "computation."

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-3.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-4.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-5.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-6.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-7.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-8.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-9.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-10.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-11.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-12.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-13.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-14.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-15.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-16.png>)

## Level 2 in Computer Science: "The Robot as a Data Structure"

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-17.png>)

At this level, the physical machine is abstracted into a discrete, computable object. The robot is no longer metal; it is information.

- **Core Concept: The Kinematic Tree (Graph Theory).**
  - A CS scientist sees the robot as a Directed Acyclic Graph (DAG) (or a cyclic graph if closed loops exist).
  - **Nodes**: Links (coordinate frames).
  - **Edges**: Joints (transforms $T \in SE(3)$).
- **The Representation: URDF / Object-Oriented Code.**
  - The robot is an instance of a class: 
    - `class Robot { List<Link> links; List<Joint> joints; }`.
  - **Algorithms**: Tree traversal (Recursion) to compute Forward Kinematics.
  - **Constraint**: "Is the parent index valid? Is the transform matrix invertible?"
- **Connection to Description Formats**:
  - The CS perspective handles the **serialization** (XML/JSON parsing) and the **API** (how we query the robot's state).
- **Limitations**: A CS model is often "too rigid." It tends to treat the world as discrete and deterministic (e.g., `if (collision) then (stop))`, struggling with the messy, continuous nature of physics.

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-18.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-19.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-20.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-21.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-22.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-23.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-24.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-25.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-26.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-27.png>)

## Level 3 in Mathematics & Physics: "The Robot as a Manifold"

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-28.png>)

This is the highest level of abstraction, required for **Embodied Intelligence**. Here, the robot is a single point moving on a high-dimensional curved surface. This view unifies the constraints of the ME (geometry) with the state-space of the CS (variables).

- **Core Concept: The Configuration Space Manifold ($\mathcal{Q}$)**.
  - The physicist sees the robot not as "parts" but as a generic system with $n$ degrees of freedom evolving on a manifold.
  - **The State**: A point $q \in \mathcal{Q}$.
  - **The Motion**: A trajectory $\gamma(t): [0, T] \to \mathcal{Q}$.
- **The Representation: Lagrangian Dynamics & Differential Geometry**.
  - **Tangent Bundle** ($T\mathcal{Q}$): The space of positions and velocities $(q, \dot{q})$.
  - **Riemannian Metric**: The Mass Matrix $M(q)$ defines the "metric" (distance) on this manifold. High inertia = "longer distance" effectively.
  - **Gradient Fields**: Control policies are vector fields on this manifold.
  - Equation: $\frac{d}{dt} \frac{\partial L}{\partial \dot{q}} - \frac{\partial L}{\partial q} = \tau$
- **Connection to Description Formats**:
  - The description format defines the manifold's topology and metric tensor.
- **Embodied Intelligence**:
  - To a standard algorithm (CS), moving an arm is changing a variable.
  - To an Embodied AI (Math/Phys), moving an arm is **following the geodesic** (path of least energy) on the curved manifold defined by the robot's mass distribution.


![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-29.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-30.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-31.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-32.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-33.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-34.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-35.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-36.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-37.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-38.png>)

## The Synthesis: Artificial Intelligence

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-39.png>)

We can now define **the Embodiement of Artificial Intelligence** as the ability to traverse the **Manifold (Level 3)** efficiently, computed via **Algorithms (Level 2)**, while respecting the **Physical Constraints (Level 1)**.

- **Why Simulation Needs All Three**:
  - If you ignore **ME**, your simulation allows the robot to pass through itself (collision failure).
    - Hardware failures become "physical constraint violations."
  - If you ignore **CS**, you cannot build a scalable system to compute the state (software failure).
    - Coding bugs become "graph traversal errors."
  - If you ignore **Math/Physics**, your RL agent cannot optimize, because it doesn't understand the gradient of the energy landscape (learning failure).
    - Complex math becomes "exploring the manifold."
- Understanding this is crucial because all modern control and learning algorithms (from RRT* path planning to Proximal Policy Optimization (PPO))operate on this manifold, not on the physical robot itself.

![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-40.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-41.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-42.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-43.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-44.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-45.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-46.png>)
![](<images/Keynote-Week01-1-Lecture-Perspectives in Understanding Robots-images-47.png>)
