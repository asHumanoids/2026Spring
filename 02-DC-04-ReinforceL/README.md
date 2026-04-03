# Reinforcement Learning Pipeline for Humanoid Training

A common pipeline for training humanoid robots with reinforcement learning (RL) progresses through three stages:

1. **Train in Isaac Lab**: Start by training the policy in Isaac Lab where data is cheap, safe, and fast to generate. This phase focuses on learning stable locomotion and basic behaviors.
2. **Sim2sim in MuJoCo**: Transfer the Isaac Lab policy into MuJoCo and evaluate/adapt it under different contact dynamics and environment settings. This improves robustness to simulator/model mismatch before hardware deployment.
3. **Sim2real**: Finally, deploy the policy on the real robot, using careful calibration and safety constraints. Real-world fine-tuning may be applied to bridge the remaining gap between simulation and hardware.

## Example Flow Using Unitree G1

- **Train in Isaac Lab**: Train the G1 locomotion policy in Isaac Lab (external training workspace), then export the trained policy checkpoint.
- **Sim2sim in MuJoCo**: Use `unitree_mujoco/` to load and evaluate the exported policy in MuJoCo, then use `unitree_rl_lab/` tooling/configs to run robustness checks and simulator-side adaptation.
- **Sim2real**: Deploy to hardware through `unitree_sdk2/` (C++ SDK) or `unitree_sdk2_python/` (Python SDK), which interface with the real G1 robot.

## Diagram

```
Isaac Lab Training      MuJoCo Sim2Sim                Sim2Real Deployment
------------------      -------------                 -------------------
Isaac Lab workspace     unitree_mujoco/               unitree_sdk2/
   |                    unitree_rl_lab/               unitree_sdk2_python/
   +---- checkpoint ----->   |                               |
                             +--------- validated policy -----+
```

## Quick Run
### Train in Isaac Lab
- Step 1: Install Isaac Sim and Lab following the steps in https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html
- Step 2: Install unitree_rl_lab
```
cd unitree_rl_lab
conda activate env_isaaclab
./unitree_rl_lab.sh -i
```

- Step 3: Train g1-23dof task
```
python scripts/rsl_rl/train.py --headless --task Unitree-G1-23dof-Velocity
```
- Step 4: Test the policy in Isaac Sim. A checkpoint has already in the folder so that you can test without training.
```
python scripts/rsl_rl/play.py --task Unitree-G1-23dof-Velocity \
--checkpoint /home/as/Documents/unitree/unitree_rl_lab/logs/rsl_rl/unitree_g1_23dof_velocity/2025-12-30_13-46-38/model_10000.pt
```

### Sim2Sim: Deploy in mujoco
- Installation
```bash
conda create --name unitree_mujoco python=3.11
conda activate unitree_mujoco

# 1: Install dependencies
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
pip install mujoco pygame

# 2: Install unitree_sdk2 & unitree_sdk2_python
cd ~/Document/unitree
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python

git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
cd ..
export CYCLONEDDS_HOME="$(pwd)/install"
pip3 install cyclonedds --no-binary cyclonedds

cd ..
pip3 install -e .

# 3: Install unitree_mujoco according to repo readme
# https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation
cd ~/Document/unitree
git clone https://github.com/unitreerobotics/unitree_mujoco.git

# Step 4: Compile the robot_controller in unitree_rl_lab
cd ~/Documents/unitree/unitree_rl_lab/deploy/robots/g1_23dof # or other robots
# g1_23dof, modify CMakeLists.txt: set(CMAKE_CXX_STANDARD 17)
mkdir build && cd build
cmake .. && make
```

- Test
```bash
# run unitree_mujoco
# In terminal 1: launch mujoco simulation
# Set the robot at /simulate_python/config.yaml to g1
# Set domain_id to 0
# Set enable_elastic_hand to 1
# Set use_joystck to 1
cd unitree_mujoco/simulate_python
python unitree_mujoco.py

# In terminal 2
# ./g1_ctrl: error while loading shared libraries: libddsc.so.0: cannot open shared object file: No such file or directory
# so files are in the /home/fang/Documents/unitree/unitree_sdk2/thirdparty/lib/x86_64
# add path to 
# Edit config file to specify the policy path in /home/as/Documents/unitree/unitree_rl_lab/deploy/robots/g1_23dof/config/config.yaml
conda activate env_isaaclab
cd unitree_rl_lab/deploy/robots/g1_23dof/build
./g1_ctrl -n lo
# 1. press [L2 + Up] to set the robot to stand up
# 2. Click the mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [R1 + X] to run the policy.
# 4. Click the mujoco window, and then press 9 to disable the elastic band.
```