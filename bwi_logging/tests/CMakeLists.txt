### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python nose tests, not needing a running ROS core.
catkin_add_nosetests(test_directory.py)

# these dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

# parse check all the launch/*.launch files
roslaunch_add_file_check(../launch)
