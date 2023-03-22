cmake_minimum_required(VERSION 2.8.3)
project(fgpg)

add_compile_options(-std=c++17)

# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message("${CMAKE_INSTALL_PREFIX}/include")
message("${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl QUIET)

if (fcl_FOUND)
  # convert cmake target to variables for catkin_package DEPENDS
  if(TARGET ${FCL_LIBRARIES})
    get_target_property(LIBFCL_LOCATION ${FCL_LIBRARIES} LOCATION)
    get_target_property(LIBFCL_INTERFACE_LINK_LIBRARIES ${FCL_LIBRARIES} INTERFACE_LINK_LIBRARIES)
    get_target_property(LIBFCL_INCLUDE_DIRS ${FCL_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
    set(LIBFCL_LIBRARIES ${LIBFCL_LOCATION} ${LIBFCL_INTERFACE_LINK_LIBRARIES})
  else()
    set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
    set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBFCL_PC REQUIRED fcl)
  set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
  # find *absolute* paths to LIBFCL_LIBRARIES
  set(LIBFCL_LIBRARIES)
  foreach(_lib ${LIBFCL_PC_LIBRARIES})
    find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
    list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
  endforeach()
endif()

message("LIBFCL_LIBRARIES: ${LIBFCL_LIBRARIES}")
message("LIBFCL_INCLUDE_DIRS: ${LIBFCL_INCLUDE_DIRS}")

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
  ${LIBFCL_INCLUDE_DIRS}
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
