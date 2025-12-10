cmake_minimum_required(VERSION 3.0.2)
project(mpc_planner_dashboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
  mpc_planner_msgs
)

## This tells catkin to use setup.py and install the Python package
catkin_python_setup()

catkin_package()

## Install plugin.xml so ${prefix}/plugin.xml exists
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install the .ui file(s) to the share dir
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install the Python package (src/mpc_planner_dashboard) into PYTHONPATH
install(DIRECTORY src/mpc_planner_dashboard
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
