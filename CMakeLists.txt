cmake_minimum_required(VERSION 2.8.3)
project(roswww)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

catkin_install_python(
  PROGRAMS script/webserver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch test www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS
  PATTERN "test/test_*.py" EXCLUDE

)
catkin_install_python(
  PROGRAMS test/test_roswww.py test/test_single.py test/test_basic.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/launch.test)
  add_rostest(test/single.test)
endif()
