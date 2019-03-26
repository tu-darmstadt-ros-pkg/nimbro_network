cmake_minimum_required(VERSION 2.8)

project(nimbro_log_transport)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp rosgraph_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS)

add_message_files(FILES
  LogBlock.msg
  LogMsg.msg
  )

generate_messages(DEPENDENCIES
  rosgraph_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rosgraph_msgs std_msgs
  DEPENDS Boost
)

add_compile_options("-Wall")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(log_sender src/log_sender.cpp)
target_link_libraries(log_sender ${catkin_LIBRARIES})
add_dependencies(log_sender ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(log_receiver src/log_receiver.cpp)
target_link_libraries(log_receiver ${catkin_LIBRARIES})
add_dependencies(log_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS log_receiver log_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)