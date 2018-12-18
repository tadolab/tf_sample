cmake_minimum_required(VERSION 2.8.3)
project(tf_sample)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf
)

add_library(tf1_sample
  src/tf1_sample.cpp include/tf_sample/tf1_sample.h
)

add_library(tf2_sample
  src/tf2_sample.cpp include/tf_sample/tf2_sample.h
)

target_link_libraries(tf1_sample
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(tf2_sample
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_sample_node
  src/tf_sample_node.cpp
)

target_link_libraries(tf_sample_node
  tf1_sample
  tf2_sample
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS tf_sample_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
