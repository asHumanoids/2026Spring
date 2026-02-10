# 00-DVSI — Description, Visualization, Simulation & Interaction

This module is a practical toolbox for working with robots end-to-end:
- **Description**: how we represent robots (URDF, MJCF, SDF)
- **Visualization**: how we inspect models, frames, and motion
- **Simulation**: how we run physics (MuJoCo, Isaac Sim)
- **Interaction**: how we connect the real world (motion capture, teleoperation)

The emphasis is on the engineering details that usually cause the first failures: coordinate frames, joint conventions, units, limits, contacts, and timing.

## 1. Robot Description (URDF / MJCF / SDF)

Robot description files are *structured kinematic + geometric models* of a robot. They typically contain:
- **Kinematics**: links and joints (types, axes, limits)
- **Geometry**: visual meshes and collision shapes
- **Inertial properties**: mass, COM, inertia tensors (needed for dynamics)
- **Frames**: transforms that define how everything is attached

### URDF (common in ROS)
- Tree-structured link/joint model.
- Great for kinematics, TF frames, and visualization.
- Dynamics fields exist, but many pipelines primarily use URDF for kinematics + collision geometry.

Common pitfalls:
- Units are meters/radians, but meshes are frequently authored in millimeters.
- Joint axis is expressed in the joint frame; getting the axis sign wrong flips motion.
- Inertias must be in the link frame; incorrect inertia values can destabilize simulators.

### MJCF (MuJoCo)
- Designed for simulation-first workflows.
- Contains bodies, joints, actuators, contacts, and many simulator-specific parameters.

Common pitfalls:
- Actuation is explicit (e.g., motors/actuators); a “model with joints” may still be un-controllable without actuators.
- Contacts depend heavily on collision geometry quality and contact parameters.

### SDF (Gazebo / Ignition / general simulation ecosystems)
- More expressive than URDF in some areas (multiple models, plugins, sensors).
- Often used as the “simulation ground truth” model format.

### Takeaway
URDF/MJCF/SDF are not just file formats: they represent different assumptions.
- URDF: kinematics + visualization (+ basic dynamics)
- MJCF: simulation-ready model with actuator/contact details
- SDF: simulation and sensors/plugins in a broader world context

## 2. Visualization

Visualization is how you debug *before* you run physics.

What to check visually:
- Link frames and joint axes (does the hinge rotate the way you expect?)
- Joint limits (does the motion stop where it should?)
- Mesh scale and alignment (are feet on the ground plane?)
- Collision geometry (is it sane, watertight enough, and not self-intersecting?)

Typical tools:
- **MuJoCo viewer**: quick inspection of MJCF models, contacts, and playback.
- **Isaac Sim viewport**: scene-centric inspection, sensors, and rendering.
- **ROS tools (RViz / TF)** (if used): frame inspection, markers, and trajectories.

## 3. Simulation

Simulation turns a description into behavior. Two common modes:
- **Kinematic playback**: apply a joint trajectory `q(t)` and visualize.
- **Physics simulation**: integrate dynamics with contacts, actuators, and controllers.

### MuJoCo
Strengths:
- Fast, robust contact dynamics.
- Convenient API for Jacobians, forward kinematics, and interactive viewing.

Key knobs to understand:
- Timestep and integrator stability
- Contact parameters (friction, solver settings)
- Actuator models (torque vs position control; gains)

### Isaac Sim
Strengths:
- Rich scene composition, sensors, rendering, and integration with Omniverse workflows.

Key knobs to understand:
- Physics timestep vs rendering timestep
- Controller update rate (your loop timing matters)
- Collision and friction settings (often different defaults than MuJoCo)

### Takeaway
A stable demo usually requires:
- correct units and inertias
- sane collision meshes
- reasonable controller gains
- consistent simulation/control rates

## 4. Interaction (Motion Capture & Teleoperation)

Interaction connects a human or a real robot to your simulated (or real) system.

Typical pipelines:
- **Motion capture → retargeting → robot reference trajectory**
- **Phone/VR/controller pose → IK target → robot motion**
- **Robot state streaming → visualization / logging / analysis**

Engineering issues that dominate outcomes:
- Coordinate frame conventions (world vs body vs device frames)
- Latency and jitter (filtering, prediction, rate matching)
- Safety constraints (limits, workspace, collision, contact assumptions)

## Learning Goals
- Recognize what information each description format can/can’t represent well.
- Debug robot models with visualization (frames, scale, collisions).
- Understand the “minimum ingredients” needed to run physics stably.
- Build intuition for real-time interaction: transforms, latency, and constraints.

## Exercises: Simulate Your First Humanoid

### 1. Install MuJoCo
- Github: https://github.com/google-deepmind/mujoco
- Documentation: https://mujoco.readthedocs.io/en/stable/overview.html

```python
# Create a new conda env for mujoco
conda create -n asHumanoids python=3.11
conda activate asHumanoids
pip install mujoco mink asmagic matplotlib loop-rate-limiters
```

### 2. Load the Humanoid BallBot of asRoBallet

```python
# Open the interactive GUI viewer provided by mujoco
python -m mujoco.viewer

# Drag and drop scene.xml into the GUI
# You will see the robot fall on the floor as the base is floating

## TODO: fixed the base so that the robot is fixed in the world frame 
# by comment/remove the <freejoint/> below the robot base

## TODO: manually move the arm and head joints 
# by adjusting the bars under the control tab
```

### 3. Interact with asRoBallet via asMagic App
0. comment/remove the <freejoint/> of the base.
1. Connect your iPhone to the same local Wi-Fi, just like your PC.
2. Install asMagic on your iPhone. Open the app and click the Data Streaming icon under Tools. You will see the IP of your iPhone.
3. Run the Jupyter notebook teleop_asRoBallet_mujoco_asmagic.ipynb
4. Hold your iPhone at a comfortable pose and  start streaming your iPhone's AR data by clicking on the switch icon when you are ready to teleoperate the dual arm in MuJoCo.