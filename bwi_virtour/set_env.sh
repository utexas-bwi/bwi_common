#!/bin/bash
ip=`hostname -I | cut -d ' ' -f1`
export ROS_IP=$ip
export ROS_MASTER_URI=http://hypnotoad.csres.utexas.edu:11311
