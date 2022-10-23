cmake_minimum_required(VERSION 3.0.2)
project(my_simulator)

set(CMAKE_BUILD_TYPE "Release")
set(PYTHON_EXECUTABLE "/usr/bin/python3")
add_subdirectory(ThirdParty/mujoco_simulate)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
set(CMAKE_MODULE_PATH 
      ${CMAKE_MODULE_PATH} 
      ${PROJECT_SOURCE_DIR}/cmake)
# message(${CMAKE_MODULE_PATH})
find_package(Eigen3)
find_package(mujoco REQUIRED)
set(MUJOCO_INCLUDE_DIRS "/usr/local/include/mujoco")
set(MUJOCO_LIBRARIES "mujoco::mujoco")

catkin_package(
  LIBRARIES run_anymal run_anymal_mujoco
  CATKIN_DEPENDS my_utils my_robot_core
)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(ThirdParty_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ThirdParty/mujoco_simulate)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${MUJOCO_INCLUDE_DIRS}
  ${ThirdParty_INCLUDE_DIR}
)


# file(GLOB_RECURSE dart_anymal_sources "src/Dart/ANYmal/*.cpp")
# file(GLOB_RECURSE dart_anymal_headers "include/my_simulator//Dart/ANYmal/*.hpp")

# message(STATUS "#############")
# message(STATUS ${mujoco_anymal_sources})
# message(STATUS ${mujoco_anymal_headers})
# message(STATUS "#############")
# add_executable(run_anymal ${dart_anymal_sources}
#                           ${dart_anymal_headers})
# target_link_libraries(run_anymal ${DART_LIBRARIES}
#                                   my_robot_core
#                                   my_utils)


file(GLOB_RECURSE mujoco_anymal_sources "src/Mujoco/ANYmalC/*.cpp")
file(GLOB_RECURSE mujoco_anymal_headers "include/my_simulator/Mujoco/ANYmalC/*.hpp")

add_executable(run_anymal_mujoco ${mujoco_anymal_sources}
                                 ${mujoco_anymal_headers})

target_link_libraries(run_anymal_mujoco 
                        ${MUJOCO_LIBRARIES}
                        ${CMAKE_DL_LIBS}
                        libsimulate
                        my_robot_core
                        my_utils
                        glfw
                        Threads::Threads)

target_include_directories(run_anymal_mujoco PUBLIC ${PROJECT_INCLUDE_DIR})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_simulator.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/my_simulator_node.cpp)

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
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
#   LIBRARY DESTINATION ${INSTALL_LIB_DIR}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${INSTALL_INCLUDE_DIR}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_simulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
