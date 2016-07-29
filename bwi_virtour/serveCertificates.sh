#!/bin/bash
WS=~/catkin_ws_scavenger/
source $WS/devel/setup.bash
roscd bwi_logging;cd log_files;python -m SimpleHTTPServer
