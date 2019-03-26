cmake_minimum_required(VERSION 2.8)

project(tf_throttle)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_msgs
  tf2_ros
)

catkin_package(CATKIN_DEPENDS roscpp tf2 tf2_msgs tf2_ros)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tf_throttle.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)