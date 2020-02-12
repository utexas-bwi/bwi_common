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
 - NVIDIA GPU + CUDA CuDNN (For acceleration)
 - Python 2.7 (for ROS)
 - Python 3.6+ (for pytorch)
### Getting Started
- Install torch and dependencies from https://pytorch.org/
- Install gytorch https://gpytorch.ai/
- Clone this repo:

```bash
git clone https://github.com/utexas-bwi/bwi_lab.git
cd ~/catkin_ws/src
git clone -b multi_robot_navigation https://github.com/utexas-bwi/bwi_lab.git
