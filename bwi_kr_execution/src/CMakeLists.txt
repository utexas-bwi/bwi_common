

SET( spexec_SRC ${spexec_SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}/plan_executor_node.cpp
	PARENT_SCOPE)
	
SET( anyexec_SRC ${anyexec_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/any_plan_executor.cpp
  PARENT_SCOPE)
	
SET( lexec_SRC ${lexec_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/learning_executor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/learning/DefaultTimes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/learning/SarsaActionSelector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/learning/ActionLogger.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/learning/RLActionExecutor.cpp
  PARENT_SCOPE)
