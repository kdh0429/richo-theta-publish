cmake_minimum_required(VERSION 3.0.2)
project(richo-theta-publish)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge

)

find_package( OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES richo-theta-publish
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)



###########
## Build ##
###########
set(Libpthread "/usr/lib/x86_64-linux-gnu/libpthread.so")
set(Libgstapp "/usr/lib/x86_64-linux-gnu/libgstapp-1.0.so")
set(Libgstbase "/usr/lib/x86_64-linux-gnu/libgstbase-1.0.so")
set(Libgstreamer "/usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so")
set(Libgobject "/usr/lib/x86_64-linux-gnu/libgobject-2.0.so")
set(Libglib "/usr/lib/x86_64-linux-gnu/libglib-2.0.so")
set(Libuvc "/home/dyros_rm/libuvc-theta/build/libuvc.so")
set(Libusb "/usr/lib/x86_64-linux-gnu/libusb-1.0.so")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/local/lib"
  "/usr/local/include/gstreamer-1.0"
  "/usr/local/include"
  "/usr/include/glib-2.0"
  "/usr/lib/x86_64-linux-gnu/glib-2.0/include"
  "/usr/include/libusb-1.0"
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(richo src/main.cpp src/thetauvc.c)


## Specify libraries to link a library or executable target against
target_link_libraries(richo ${catkin_LIBRARIES})
target_link_libraries(richo ${Libgstapp} )
target_link_libraries(richo ${Libgstbase} )
target_link_libraries(richo ${Libgstreamer} )
target_link_libraries(richo ${Libgobject})
target_link_libraries(richo ${Libglib} )
target_link_libraries(richo ${Libuvc} )
target_link_libraries(richo ${Libusb} )
target_link_libraries(richo ${Libpthread})
target_link_libraries(richo ${OpenCV_LIBRARIES})





