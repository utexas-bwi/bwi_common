bwi_common / Gazebo-SLAM branch
==========
Testing new *Wider* Gazebo map and the slam generated map.

Make sure to check ![bwi/Gazebo-SLAM](https://github.com/utexas-bwi/bwi/tree/Gazebo-SLAM) branch

## How to run test
1. launch multi_robot_simulation environment
```
roslaunch bwi_launch multi_robot_simulation.launch
```
2. run reset_env.py
```
bwi_common: $ python reset_env.py
```
3. In a rviz GUI, give a goal_pose to each robots and observe any anomalies including mis-localization.
