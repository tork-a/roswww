cmake_minimum_required(VERSION 2.8.3)
project(roswww)
find_package(catkin REQUIRED rostest)
catkin_package()
catkin_python_setup()

install(
  PROGRAMS
    script/webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_rostest(test/launch.test)
