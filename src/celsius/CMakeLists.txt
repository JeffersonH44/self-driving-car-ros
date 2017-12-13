cmake_minimum_required(VERSION 2.8.3)
project(celsius)

find_package(catkin REQUIRED rospy std_msgs message_generation)

add_message_files(DIRECTORY msg
  FILES
    CelsiusReport.msg
    CelsiusControl.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
 CATKIN_DEPENDS
    message_runtime
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/interface.py
  __init__.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/celsius
)

install(FILES
  README.md
  launch/fusion_ac.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/celsius
)

catkin_add_nosetests(test)
