bwi_common ~ Multi Robot Navigation package
==========

Developing package for two robot navigation system on top of popularly used ROS navigation package.
This package includes
1. 1.6m wide simulated Utexas GDC 3 northeast Hallway
2. Baseline policies and trained policy
 - Chicken strategy
 - Oneshot Bandit Educated Yielding (OBeY) policy
 - Centralized simulation codes
 - De-centralized codes for running on BWI robots.

## Setup
### Prerequisites
 - Ubuntu 16.04
 - ROS Kinetic
 - NVIDIA GPU + CUDA (For acceleration) https://developer.nvidia.com/cuda-downloads
 - Python 2.7 (for ROS)
 - utexas-BWI (https://github.com/utexas-bwi)
### Installation
- Install Python3.6+ version alongside with ROS system
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
```
If you want to use python 3.6 instead, change python 3.7 shebang line at bwi_safety_policy package to Python 3.6 version.
- Install pip for python 3.7
```bash
sudo apt-get install python3-pip python3-yaml
python3.7 -m pip install pip
```
- Install ROS package for python 3.7
```bash
python3.7 -m pip install rospkg catkin_pkg
```
- Install torch and dependencies from https://pytorch.org/
```bash
python3.7 -m pip install torch torchvision
```
- Install gytorch https://gpytorch.ai/
```bash
python3.7 -m pip install gpytorch
```
- Install other packages
```bash
python3.7 -m pip install numpy pandas
```

**Note**: ROS requires python 2.7 version. Please make sure to install pytorch and gpytorch in a separate python 3.6+ environment.

- Make sure to compile the original bwi repo before cloning this repo.
- Clone this repo:

```bash
cd ~/catkin_ws/src
git clone -b multi_robot_navigation https://github.com/utexas-bwi/bwi_common.git
```
- You must unzip the multi_robot_collision_avoidance.tar.gz and overwrite the contents. ( I'm still trying to find a way to erase uploaded folders. )
- Compile packages with a given order.
```bash
cd ~/catkin_ws
catkin build
```

## How to run
```bash
roslaunch bwi_launch multi_robot_simulation.launch
```
If you want to test an adaptive solution you need a waypoint evaluation service. Run below code in the separate bash.  
```bash
rosrun multi_robot_collision_avoidance eval_waypoint_server.py
```
Now run the experiment handling node.
```bash
rosrun multi_robot_navigation exp_handler ${exp_name} ${num_episode}
```
${exp_name} can be one of [ 'raw', 'chicken', 'siren' ]
${num_episode} can be any natural number.

## How to modify
Currently, the parameters should be tuned on the codes itself.
You can change the location of each episode in the 715th line of "multi_robot_navigation/src/exp_hander.cpp".

Definition of each parameter is
SpawnPose: Where robots are spawned. This is not the starting pose of the episode.
InitPose: Where Episode begins.
GoalPose: Goal of each agent. By default, it should be the same as other robot's InitPose.
StandbyPose: Where the robot will wait for other robots to finish its journey.

Also, the siren experiment uses the epsilon greedy algorithm to learn the policy.
You can change the configuration of this GP model from "multi_robot_collision_avoidance/script/config.yaml" file.

## Caution
- I found overlapped init_pose of the robot with the goal of the other robot makes potential trouble.
Please use the below setting.
1. distance of spawn_pose -> init_pose within robot : < 4. m
2. distance of init_pose -> goal_pose within robot  : 12. m
3. distance of init_pose of two robots              : 10. m
4. choose the standby in an open area. (if possible) The episode ends when both robots reach their goal pose. So editing a standby pose does not affect the result.

## Troubleshooting
If you are having trouble with run `eval_waypoint_server.py`, run following.
```
roscd multi_robot_collision_avoidance
cd script
chmod +x eval_waypoint_server.py
```
