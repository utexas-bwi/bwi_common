
add_subdirectory(asp)
add_subdirectory(executors)
add_subdirectory(reasoners)
add_subdirectory(planners)

SET( actasp_SRC  ${actasp_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/Action.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/AnswerSet.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/MultiPolicy.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GraphPolicy.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/action_utils.cpp
PARENT_SCOPE)
