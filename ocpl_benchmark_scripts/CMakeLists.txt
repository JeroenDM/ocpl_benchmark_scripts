cmake_minimum_required(VERSION 3.0.2)
project(ocpl_benchmark_scripts)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_visual_tools
  ocpl_benchmark
  ocpl_planning
  ocpl_ros
  simple_moveit_wrapper
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocpl_benchmark_scripts
#  CATKIN_DEPENDS ocpl_benchmark ocpl_planning ocpl_ros simple_moveit_wrapper
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_benchmark1
  src/benchmark1.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark1
  PROPERTIES OUTPUT_NAME benchmark1 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark1
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_benchmark2
  src/benchmark2.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark2
  PROPERTIES OUTPUT_NAME benchmark2 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark2
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_benchmark3
  src/benchmark3.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark3
  PROPERTIES OUTPUT_NAME benchmark3 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark3
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_benchmark4
  src/benchmark4.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark4
  PROPERTIES OUTPUT_NAME benchmark4 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark4
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_benchmark5
  src/benchmark5.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark5
  PROPERTIES OUTPUT_NAME benchmark5 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark5
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_benchmark6
  src/benchmark6.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark6
  PROPERTIES OUTPUT_NAME benchmark6 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark6
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_benchmark7
  src/benchmark7.cpp
)
set_target_properties(${PROJECT_NAME}_benchmark7
  PROPERTIES OUTPUT_NAME benchmark7 PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_benchmark7
  ${catkin_LIBRARIES}
)




add_executable(${PROJECT_NAME}_kuka_on_rail
  src/kuka_on_rail.cpp
)
set_target_properties(${PROJECT_NAME}_kuka_on_rail
  PROPERTIES OUTPUT_NAME kuka_on_rail PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_kuka_on_rail
  ${catkin_LIBRARIES}
)


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
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ocpl_benchmark_scripts.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
