cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(puppeteer_control src/puppeteercontrol.cpp)
rosbuild_add_executable(straight_control src/straight_driving.cpp)
rosbuild_add_executable(kinematic_control src/kinematic_controller.cpp)
rosbuild_add_executable(kinematic_control_3D src/kinematic_controller_3D.cpp)
rosbuild_add_executable(position_control src/position_control.cpp)
#rosbuild_add_executable(closed_loop_control src/closed_loop_control.py)
rosbuild_add_executable(wiimote_control src/wiimote_control.cpp include/wiiuse.h)
target_link_libraries(wiimote_control ${PROJECT_SOURCE_DIR}/lib/libwiiuse.so)
rosbuild_add_executable(kalman_controller src/kalman_kinematic_control.cpp)
rosbuild_add_executable(multi_kalman_controller src/multi_kalman_control.cpp)