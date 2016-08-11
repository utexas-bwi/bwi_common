#!/bin/bash
rosnode list | grep /bwi_kr > /dev/null
if [ $? -eq 0 ]; then
  echo "bwi_kr_execution already running";
else
  roslaunch bwi_kr_execution bwi_kr_execution.launch
fi
