cmake_minimum_required(VERSION 2.8.3)
project(xm_object_cui)

set(OpenCV_DIR /usr/local/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  PCL REQUIRED 
  OpenCV REQUIRED 
  xm_msgs
  rospy
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  tf
)

#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
add_message_files(
   FILES
   num.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS src  src/include /usr/local/include
 CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES objectMaster
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  src
  src/include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 

)
#link_directories(${PCL_LIBRARY_DIRS} src)
#add_definitions(${PCL_DEFINITIONS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(objectMaster
#   src/${PROJECT_NAME}/objectMaster.cpp
# )

## Declare a cpp executable
# add_executable(objectMaster_node src/objectMaster_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(objectMaster_node objectMaster_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(objectMaster_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS objectMaster objectMaster_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#add_library(detect src/detect.cpp)
add_library(Filter src/Filter.cpp)
add_library(rosgetimage src/rosgetimage.cpp)
add_library(Train src/Train.cpp)
add_library(detect src/detect.cpp)
#add_library(tld_utils src/tld/src/tld_utils.cpp)
#add_library(LKTracker src/tld/src/LKTracker.cpp)
#add_library(ferNN src/tld/src/FerNNClassifier.cpp)
#add_library(tld src/tld/src/TLD.cpp)
#add_library(tracking src/tld/src/tracking.cpp)
add_executable(getobject src/getobject.cpp )
#add_executable(main_test1   src/main_test1.cpp)
add_executable(main_test   src/main_test.cpp)

link_directories(
  ${catkin_LIBRARIES}  /usr/local/lib
  #${PCL_LIBRARY_DIRS}
 # ${OpenCV_LIBRARIES}
 # src
)
target_link_libraries(getobject  Train rosgetimage Filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(main_test1 rosgetimage Filter detect  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libhpdf.so)
target_link_libraries(main_test  rosgetimage  Filter  detect ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libhpdf.so)
#target_link_libraries(main_test1   tracking tld LKTracker ferNN tld_utils ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} libhpdf.so)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_objectMaster.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
set(CMAKE_BUILD_TYPE Release)
