cmake_minimum_required(VERSION 2.8.3)
project(roswww)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
