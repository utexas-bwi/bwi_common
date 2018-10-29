cmake_minimum_required(VERSION 2.8.3)
project(bwi_perception)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	set(CMAKE_CXX_STANDARD 11)
endif()


find_package( OpenCV REQUIRED)

find_package(catkin REQUIRED
		actionlib_msgs
		cv_bridge
		geometry_msgs
		message_generation
		pcl_ros
		pcl_conversions
		roscpp
		rospy
		sensor_msgs
		std_msgs
		std_srvs
		visualization_msgs
)

add_service_files(
		FILES
		ButtonDetection.srv
		DetectHorizontalPlanes.srv
		ExtractTabletopScene.srv
		FeatureExtraction.srv
		GetCloud.srv
		GetPCD.srv
		PerceiveLargestHorizontalPlane.srv
		PerceiveTabletopScene.srv
		ProcessVision.srv
		SetObstacles.srv
)

generate_messages(
		DEPENDENCIES
		actionlib_msgs
		geometry_msgs
		std_msgs
		std_srvs
		sensor_msgs
		visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS actionlib_msgs
        LIBRARIES libbwi_perception
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


add_library(libbwi_perception src/libbwi_perception/BoundingBox.cpp src/libbwi_perception/tabletop.cpp src/libbwi_perception/convenience.cpp)
target_link_libraries(libbwi_perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(libbwi_perception ${bwi_perception_EXPORTED_TARGETS})

add_executable(horizontal_surface_detector src/horizontal_surface_detector.cpp)
add_dependencies(horizontal_surface_detector ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(horizontal_surface_detector libbwi_perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(image_logging_server src/image_logging_server.cpp)
add_dependencies(image_logging_server ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(image_logging_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(obstacle_cloud_manager src/obstacle_cloud_manager.cpp)
add_dependencies(obstacle_cloud_manager ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(obstacle_cloud_manager ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_feature_server src/pointcloud_feature_server.cpp)
add_dependencies(pointcloud_feature_server ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(pointcloud_feature_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tabletop_scene_perception_node src/tabletop_scene_perception_node.cpp)
add_dependencies(tabletop_scene_perception_node ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(tabletop_scene_perception_node libbwi_perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(table_change_detection_node src/table_change_detection_node.cpp)
add_dependencies(table_change_detection_node ${bwi_perception_EXPORTED_TARGETS})
target_link_libraries(table_change_detection_node libbwi_perception ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(button_detection_srv_node src/button_detection_srv_node.cpp)
target_link_libraries(button_detection_srv_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(button_detection_srv_node ${bwi_perception_EXPORTED_TARGETS})


install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
