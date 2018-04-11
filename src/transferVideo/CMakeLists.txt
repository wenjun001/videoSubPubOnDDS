cmake_minimum_required(VERSION 2.8.12)
project("transferVideo")

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(OpenCV REQUIRED)

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

message(STATUS "Configuring transferVideo ...")
file(GLOB TRANSFERVIDEO_CXX "src/*.cxx")
file(GLOB TRANSFERVIDEO_CPP "src/*.cpp")


add_executable(transferVideo src/videoPubSubMain.cc ${TRANSFERVIDEO_CXX} ${TRANSFERVIDEO_CPP})
target_link_libraries(transferVideo fastrtps fastcdr ${OpenCV_LIBRARIES} )






































































