cmake_minimum_required(VERSION 2.8.3)
project(diagnostic_recorder)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

find_package(Boost REQUIRED COMPONENTS
  filesystem)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_aggregator
  pluginlib
  roscpp)
find_package(MongoDB)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diagnostic_recorder
  CATKIN_DEPENDS diagnostic_aggregator pluginlib
  DEPENDS Boost MongoDB)


###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${MongoDB_INCLUDE_DIR})

add_library(diagnostic_recorder
  src/analyzer_recorder_group.cpp
  src/base_recorder.cpp
  src/generic_analyzer_recorder.cpp
  src/storages/filesystem_storage.cpp
  src/storages/mongodb_storage.cpp)
target_link_libraries(diagnostic_recorder ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${MongoDB_LIBRARIES})


#############
## Install ##
#############

install(TARGETS diagnostic_recorder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-build/include_what_you_use,-runtime/references,-whitespace/braces,-whitespace/line_length")
  roslint_cpp()
  roslint_add_test()
endif()
