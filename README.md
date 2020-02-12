bwi_common ~ Multi Robot Navigation package
==========

Developing package for two robot navigation system on top of popularly used ROS navigation package.
This package includes
1. Chicken strategy
2. Oneshot Bandit Educated Yielding (OBeY) policy
3. Centralized simulation codes
4. De-centralized codes for running on BWI robots.

## Setup
### Prerequisites
 - Ubuntu 16.04
 - ROS Kinetic
 - NVIDIA GPU + CUDA (For acceleration) https://developer.nvidia.com/cuda-downloads
 - Python 2.7 (for ROS)
 - utexas-BWI (https://github.com/utexas-bwi)
### Getting Started
- Install Python3.6+ version alongside with ROS system
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
```
If you wants to use python 3.6 instead, change python 3.7 shebang line at bwi_safety_policy package to python 3.6 version.
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

**Note**: ROS requires python 2.7 version. Please make sure to install pytorch and gpytorch in a seperate python 3.6+ environment.

- Clone this repo:

```bash
git clone https://github.com/utexas-bwi/bwi_lab.git
cd ~/catkin_ws/src
git clone -b multi_robot_navigation https://github.com/utexas-bwi/bwi_lab.git
```
