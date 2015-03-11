

SET( spexec_SRC ${spexec_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/single_plan_executor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/msgs_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/RemoteReasoner.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/StaticFacts.cpp  
	PARENT_SCOPE)
	
SET( anyexec_SRC ${anyexec_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/any_plan_executor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/msgs_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/RemoteReasoner.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/StaticFacts.cpp  
  PARENT_SCOPE)
	
SET( lexec_SRC ${lexec_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/learning_executor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/msgs_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/RemoteReasoner.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/learning/DefaultTimes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/learning/QLearningActionSelector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/RemoteReasoner.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/StaticFacts.cpp  
  PARENT_SCOPE)
	
SET( krreasoner_SRC ${krreasoner_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/knowledge_reasoning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/msgs_utils.cpp
PARENT_SCOPE)
