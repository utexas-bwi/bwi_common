bwi_common
==========

Common packages and data for BWI projects.

[![Build Status](https://travis-ci.org/utexas-bwi/bwi_common.svg?branch=master)](https://travis-ci.org/utexas-bwi/bwi_common)


Prism2 instructions to get texture to render in env:
0. Git pull origin prism2
1. Go to http://data.nvision2.eecs.yorku.ca/3DGEMS/ and download the furniture, electronics, decoration, and miscellaneous tars.
2. Extract the folders into ~/.gazebo/models. If you don't have a models folder within ~/.gazebo you can make one by doing mkdir ~/.gazeobo/models. Make sure that when you extract, only the models within furniture, electronics, etc are put in ~/.gazebo/models. Don't put the furniture and electornics folders themselves inside ~/.gazebo/models.
3. catkin build utexas_gdc
4. roslaunch bwi_launch simulation_v2.launch
