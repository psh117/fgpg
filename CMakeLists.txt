cmake_minimum_required(VERSION 2.8.3)
project(fgpg)

add_compile_options(-std=c++14)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message("${CMAKE_INSTALL_PREFIX}/include")
message("${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fgpg
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_lib
  src/mesh_sampling.cpp 
  src/grasp_point_generator.cpp
  src/geometrics.cpp
  src/hsv2rgb.cpp
  src/grasp_coverage_evaluator.cpp
)

add_executable(${PROJECT_NAME} 
  src/fgpg.cpp 
)

add_dependencies(${PROJECT_NAME}
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
)

add_executable(evaluator
  src/evaluator.cpp
)
add_dependencies(evaluator
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_lib
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  fcl
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  fcl
  yaml-cpp
  ${PROJECT_NAME}_lib
)
target_link_libraries(evaluator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  fcl
  yaml-cpp
  ${PROJECT_NAME}_lib
)

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
