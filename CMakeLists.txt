cmake_minimum_required(VERSION 2.8.3)
project(camera_test)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
  pcl_conversions 
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_message_files(
   FILES
   coke.msg
)

generate_messages()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_test
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/camera_test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(image_converter src/image_converter.cpp)
 target_link_libraries(image_converter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(color_detection src/color_detection.cpp)
 target_link_libraries(color_detection ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(face_detection src/face_detection.cpp)
 target_link_libraries(face_detection ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(Contours src/Contours.cpp)
 target_link_libraries(Contours ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(Convex_hull src/Convex_hull.cpp)
 target_link_libraries(Convex_hull ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(homo src/homo.cpp)
 target_link_libraries(homo ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(astra_camera_tf1 src/astra_camera_tf1.cpp)
 target_link_libraries(astra_camera_tf1 ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(astra_camera_tf2 src/astra_camera_tf2.cpp)
 target_link_libraries(astra_camera_tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES})

 add_executable(untitled src/untitled.cpp)
 target_link_libraries(untitled ${Boost_LIBRARIES} ${catkin_LIBRARIES})


 add_executable(pcl src/pcl.cpp)
 target_link_libraries(pcl ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(move_coke_tf src/move_coke_tf.cpp)
 target_link_libraries(move_coke_tf ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(pass_filter src/pass_filter.cpp)
 target_link_libraries(pass_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pass_filter_color src/pass_filter_color.cpp)
 target_link_libraries(pass_filter_color ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_camera_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
