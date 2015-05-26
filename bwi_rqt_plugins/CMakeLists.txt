cmake_minimum_required(VERSION 2.8.3)
project(bwi_rqt_plugins)
find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
