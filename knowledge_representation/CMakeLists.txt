cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)


if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    set(CMAKE_CXX_STANDARD 11)
endif()
find_package(BISON)
find_package(FLEX)

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

set(PostgreSQL_ADDITIONAL_VERSIONS "10.5" "9.3" "9.3.15")
find_package(PostgreSQL)

find_library(libpqxx NAMES pqxx)

find_library(mysql NAMES libmysqlcppconn8.so)


# Check to see if we have the right version of MySQL Cpp Connector installed
if(mysql)
    set(MYSQL_AVAILABLE true)
    set(DB_INCLUDES /usr/local/mysql/connector-c++-8.0/include)
    set(DB_LIBS ${mysql})
    add_definitions(-DUSE_MYSQL)
elseif(POSTGRESQL_FOUND OR libpqxx)
    message(WARNING "MySQL Connector C++ 8.0 was not found. Using PostgreSQL.")
    set(POSTGRES_AVAILABLE true)
    set(DB_INCLUDES ${PostgreSQL_INCLUDE_DIRS})
        set(DB_LIBS ${PostgreSQL_LIBRARIES} ${libpqxx})
    set(DB_LIBS ${mysql})
    add_definitions(-DUSE_POSTGRESQL)
else()
    message(FATAL_ERROR "Neither MySQL nor PostgreSQL found. Please install one.")
endif()


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
        INCLUDE_DIRS include ${DB_INCLUDES}
        LIBRARIES libknowledge_rep ${DB_LIBS}
        CATKIN_DEPENDS
        pcl_ros
        pcl_conversions
        roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ${DB_INCLUDES}
)


if ("${MYSQL_AVAILABLE}" OR "${POSTGRES_AVAILABLE}")


BISON_TARGET(parser src/libknowledge_rep/parser.yy ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.cpp COMPILE_FLAGS --defines=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.h)
FLEX_TARGET(scanner src/libknowledge_rep/lexer.l  ${CMAKE_CURRENT_BINARY_DIR}/Lexer.cpp)


ADD_FLEX_BISON_DEPENDENCY(scanner parser)
if (MYSQL_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/mysql/LongTermMemoryConduitMySQL.cpp)
    # Symlink the LTMC header into place. We have to put this header in the public
    # interface in order to instantiate the template of the base class. This class
    # will not be used alone, so methods in its interface can't really be used.
    ADD_CUSTOM_TARGET(link_target ALL
            COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src/libknowledge_rep/mysql/LongTermMemoryConduitMySQL.h ${PROJECT_SOURCE_DIR}/include/knowledge_representation/LongTermMemoryConduit.h)
elseif(POSTGRESQL_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/postgresql/LongTermMemoryConduit.cpp)
endif()
add_library(libknowledge_rep
        ${DB_SOURCES}
        src/libknowledge_rep/MemoryConduit.cpp
        src/libknowledge_rep/convenience.cpp
        src/libknowledge_rep/Compiler.cpp
        ${BISON_parser_OUTPUTS}
        ${FLEX_scanner_OUTPUTS} )

add_dependencies(libknowledge_rep ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(libknowledge_rep ${DB_LIBS} ${catkin_LIBRARIES})

add_library(_libknowledge_rep_wrapper_cpp src/libknowledge_rep/python_wrapper.cpp)
add_dependencies(_libknowledge_rep_wrapper_cpp ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(_libknowledge_rep_wrapper_cpp
        libknowledge_rep
        ${DB_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(memory_conduit_proxy_node src/memory_conduit_proxy_node.cpp)
add_dependencies(memory_conduit_proxy_node ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(memory_conduit_proxy_node libknowledge_rep ${catkin_LIBRARIES})

add_executable(show_me src/show_me.cpp)
add_dependencies(show_me ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(show_me libknowledge_rep ${catkin_LIBRARIES})

### TEST TARGETS

catkin_add_gtest(test_ltmc test/ltmc.cpp)
add_dependencies(test_ltmc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_ltmc libknowledge_rep ${catkin_LIBRARIES})

catkin_add_gtest(test_mc test/mc.cpp)
add_dependencies(test_mc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_mc libknowledge_rep ${catkin_LIBRARIES})

catkin_add_gtest(test_parser test/parser.cpp)
add_dependencies(test_parser ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_parser libknowledge_rep ${catkin_LIBRARIES})


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
        test_ltmc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif ()
