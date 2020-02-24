### How to use refine_segmap.py

Script require numpy 1.13 or higher.

1. You should change constant values in pix2pose.
By default, the values are set to be used in simulation/3ne/3ne.yaml.
if you wants to use it in real world, change constants with real/3/3.yaml file.

2. change start.pose.position.x, start.pose.position.y to desire value.
script will determine whether certain points are reachable from this point.

3. Change the hallway_idx.
