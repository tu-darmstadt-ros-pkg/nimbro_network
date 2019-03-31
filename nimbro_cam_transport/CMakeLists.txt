cmake_minimum_required(VERSION 2.8)

project(nimbro_cam_transport)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  )

generate_dynamic_reconfigure_options(cfg/H264Publisher.cfg)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(X264 x264 REQUIRED)
pkg_check_modules(LIBAVUTIL libavutil REQUIRED)
pkg_check_modules(LIBAVCODEC libavcodec REQUIRED)
pkg_check_modules(LIBSWSCALE libswscale REQUIRED)

link_directories(${X264_LIBRARY_DIRS} ${LIBAVUTIL_LIBRARY_DIRS} ${LIBAVCODEC_LIBRARY_DIRS} ${LIBSWSCALE_LIBRARY_DIRS})

add_library(h264_encoder src/rgb_to_yuv420.cpp src/encoder.cpp)
target_include_directories(h264_encoder PRIVATE ${X264_INCLUDE_DIRS})
target_link_libraries(h264_encoder PUBLIC ${catkin_LIBRARIES} PRIVATE ${X264_LIBRARIES})
add_dependencies(h264_encoder ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(nimbro_cam_transport_pub src/image_transport_publisher.cpp)
target_link_libraries(nimbro_cam_transport_pub h264_encoder ${catkin_LIBRARIES})

add_executable(cam_sender src/sender.cpp)
target_link_libraries(cam_sender h264_encoder ${catkin_LIBRARIES})

add_library(h264_decoder src/decoder.cpp)
target_include_directories(h264_decoder PRIVATE ${LIBAVUTIL_INCLUDE_DIRS} ${LIBAVCODEC_INCLUDE_DIRS} ${LIBSWSCALE_INCLUDE_DIRS})
target_link_libraries(h264_decoder PUBLIC ${catkin_LIBRARIES} PRIVATE ${LIBAVUTIL_LIBRARIES} ${LIBAVCODEC_LIBRARIES} ${LIBSWSCALE_LIBRARIES})

add_library(nimbro_cam_transport_sub src/image_transport_subscriber.cpp)
target_link_libraries(nimbro_cam_transport_sub h264_decoder ${catkin_LIBRARIES})

add_executable(cam_receiver src/receiver.cpp)
target_link_libraries(cam_receiver h264_decoder ${catkin_LIBRARIES})

install(TARGETS cam_receiver cam_sender h264_decoder h264_encoder nimbro_cam_transport_pub nimbro_cam_transport_sub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
