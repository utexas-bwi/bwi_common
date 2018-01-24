

SET( kractions_SRC ${KREXEC_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/ActionFactory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LogicalNavigation.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CallGUI.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OpenSimulatedDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoThrough.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GotoObject.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ApproachDoor.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SearchPerson.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/AskPerson.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Knock.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DeliverMessage.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ChangeFloor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SimulatedChangeFloor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CallElevator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/CallSimulatedElevator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../StaticFacts.cpp
	PARENT_SCOPE)
