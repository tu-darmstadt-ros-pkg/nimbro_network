cmake_minimum_required(VERSION 2.8)

project(nimbro_cam_transport)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  )

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(X264 x264 REQUIRED)
pkg_check_modules(LIBAVUTIL libavutil REQUIRED)
pkg_check_modules(LIBAVCODEC libavcodec REQUIRED)
pkg_check_modules(LIBSWSCALE libswscale REQUIRED)

link_directories(${X264_LIBRARY_DIRS} ${LIBAVUTIL_LIBRARY_DIRS} ${LIBAVCODEC_LIBRARY_DIRS} ${LIBSWSCALE_LIBRARY_DIRS})

add_executable(cam_sender src/rgb_to_yuv420.cpp src/sender.cpp)
target_include_directories(cam_sender PUBLIC ${X264_INCLUDE_DIRS})
target_link_libraries(cam_sender ${catkin_LIBRARIES} ${X264_LIBRARIES})

add_executable(cam_receiver src/receiver.cpp)
target_include_directories(cam_receiver PUBLIC ${LIBAVUTIL_INCLUDE_DIRS} ${LIBAVCODEC_INCLUDE_DIRS} ${LIBSWSCALE_INCLUDE_DIRS})
target_link_libraries(cam_receiver ${catkin_LIBRARIES} ${LIBAVUTIL_LIBRARIES} ${LIBAVCODEC_LIBRARIES} ${LIBSWSCALE_LIBRARIES})

install(TARGETS cam_receiver cam_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
