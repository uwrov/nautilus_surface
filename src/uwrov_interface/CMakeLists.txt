cmake_minimum_required(VERSION 3.0.2)
project(uwrov_interface)

catkin_package()
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/launch_interface.bash
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})