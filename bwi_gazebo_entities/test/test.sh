rosrun xacro xacro.py $1 > test.urdf && cat test.urdf && rosrun urdfdom check_urdf test.urdf && roslaunch bwi_gazebo_entities display.launch model:=test.urdf
rm -f test.urdf
