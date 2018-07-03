cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

set(CMAKE_CXX_STANDARD 11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        bwi_perception
        roscpp
        pcl_ros
        pcl_conversions
        std_msgs
        sensor_msgs
        )

find_package(Boost REQUIRED COMPONENTS python)

find_package(PythonLibs REQUIRED)

find_package(Eigen3 REQUIRED)

find_library(mysql NAMES libmysqlcppconn8.so)


link_directories(
        ${catkin_LINK_DIRS}
)

add_service_files(
        FILES
        AddObject.srv
        GetCloud.srv
        RemoveObject.srv
        ResolveObjectCorrespondences.srv
)
catkin_python_setup()

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)



catkin_package(
        INCLUDE_DIRS include /usr/local/mysql/connector-c++-8.0/include
        LIBRARIES libknowledge_rep ${mysql}
        CATKIN_DEPENDS
        pcl_ros
        pcl_conversions
        roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        /usr/local/mysql/connector-c++-8.0/include
        ${Eigen3_INCLUDE_DIRS}
)


add_library(libknowledge_rep src/libknowledge_rep/LongTermMemoryConduit.cpp src/libknowledge_rep/MemoryConduit.cpp src/libknowledge_rep/convenience.cpp)
add_dependencies(libknowledge_rep ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(libknowledge_rep ${mysql} ${catkin_LIBRARIES})

add_library(_libknowledge_rep_wrapper_cpp src/libknowledge_rep/python_wrapper.cpp)
add_dependencies(_libknowledge_rep_wrapper_cpp ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(_libknowledge_rep_wrapper_cpp
        libknowledge_rep
        ${mysql}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(memory_conduit_proxy_node src/memory_conduit_proxy_node.cpp)
add_dependencies(memory_conduit_proxy_node ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(memory_conduit_proxy_node libknowledge_rep ${catkin_LIBRARIES})

### TEST TARGETS

add_executable(test_ltmc src/test_ltmc.cpp)
add_dependencies(test_ltmc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_ltmc libknowledge_rep ${catkin_LIBRARIES})

add_executable(test_mc src/test_mc.cpp)
add_dependencies(test_mc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_mc libknowledge_rep ${catkin_LIBRARIES})

add_executable(initialize_planner_test_configuration src/initialize_planner_test_configuration.cpp)
add_dependencies(initialize_planner_test_configuration ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(initialize_planner_test_configuration libknowledge_rep ${catkin_LIBRARIES})

add_executable(add_inspection_point src/add_inspection_point.cpp)
add_dependencies(add_inspection_point ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(add_inspection_point libknowledge_rep ${catkin_LIBRARIES})

add_executable(csv_reader src/csv_reader.cpp)
target_link_libraries(csv_reader libknowledge_rep)

set_target_properties(_libknowledge_rep_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

catkin_add_env_hooks(${PROJECT_NAME} SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(TARGETS libknowledge_rep
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS
        initialize_planner_test_configuration
        test_ltmc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
