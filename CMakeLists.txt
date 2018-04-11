cmake_minimum_required(VERSION 2.8.3)
project(video_stream_opencv_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  camera_info_manager
  sensor_msgs
)


find_package(OpenCV)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)


include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB TRANSFERVIDEO_SOURCES_CXX "src/transferVideo/src/*.cxx")
file(GLOB TRANSFERVIDEO_SOURCES_CPP "src/transferVideo/src/*.cpp")


#${TRANSFERVIDEO_SOURCES_CXX} ${TRANSFERVIDEO_SOURCES_CPP}

add_executable(video_stream_sub src/video_stream_sub.cpp  ${TRANSFERVIDEO_SOURCES_CXX} ${TRANSFERVIDEO_SOURCES_CPP} )
target_link_libraries(video_stream_sub fastrtps fastcdr  ${catkin_LIBRARIES}   ${OpenCV_LIBS}   )


install(TARGETS video_stream_sub
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


