cmake_minimum_required(VERSION 2.8.3)
project(bwi_rqt_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  bwi_msgs
  rqt_gui)

catkin_package(
  CATKIN_DEPENDS
    bwi_msgs
    rqt_gui)

catkin_python_setup()

install(FILES python_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/question_dialog_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
