### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python nose tests, not needing a running ROS core.
catkin_add_nosetests(test_transitions.py)

##  Unit tests using nose, but needing a running ROS core.
find_package(rostest REQUIRED)
add_rostest(stop_base_passthru.test)
add_rostest(stop_base_requests.test)
