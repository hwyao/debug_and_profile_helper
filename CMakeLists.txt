cmake_minimum_required(VERSION 3.0.2)
project(debug_and_profile_helper)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_compile_options(-g) # add debug information

## if this file is built by catkin, DBGNPROF_COMPILE_IN_ROS is default ON, otherwise OFF
if (DEFINED CATKIN_DEVEL_PREFIX)
  set(DBGNPROF_COMPILE_IN_ROS_DEFAULT ON)
else()
  set(DBGNPROF_COMPILE_IN_ROS_DEFAULT OFF)
endif()
# option: DBGNPROF_COMPILE_IN_ROS, if ON, build with ROS support
# This option will influence:
# 1. Enable CMakeLists.txt to find and link ROS libraries
# 2. Enable DBGNPROF_COMPILE_IN_ROS Flag in the library, to build LoggerROS object
option(DBGNPROF_COMPILE_IN_ROS "Build with ROS support and preferences." ${DBGNPROF_COMPILE_IN_ROS_DEFAULT})

## if this is the top-level CMakeLists.txt, and the DBGNPROF_COMPILE_IN_ROS is OFF, build example
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT DBGNPROF_COMPILE_IN_ROS)
  set(DBGNPROF_BUILD_EXAMPLE_DEFAULT ON)
else()
  set(DBGNPROF_BUILD_EXAMPLE_DEFAULT OFF)
endif()
# option: DBGNPROF_BUILD_EXAMPLE, if ON, build example
option(DBGNPROF_BUILD_EXAMPLE "Build example executables." ${DBGNPROF_BUILD_EXAMPLE_DEFAULT})

## display options
message(STATUS 
  "[debug_and_profile_helper] options:
  DBGNPROF_COMPILE_IN_ROS:    ${DBGNPROF_COMPILE_IN_ROS}
   DBGNPROF_BUILD_EXAMPLE:    ${DBGNPROF_BUILD_EXAMPLE}"
)

include(FetchContent)

## import spdlog
find_package(spdlog 1.15.0 QUIET)
if (NOT spdlog_FOUND)
    message(STATUS "[debug_and_profile_helper] spdlog not found, using FetchContent to get it")
    FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.15.0 
    )
    set(SPDLOG_BUILD_SHARED ON)
    FetchContent_MakeAvailable(spdlog)
    unset(SPDLOG_BUILD_SHARED)
else()
    message(STATUS "[debug_and_profile_helper] spdlog 1.15.0 found locally")
endif()

## import Eigen
find_package(Eigen3 3.3.7 QUIET)
if (NOT EIGEN3_FOUND)
  message(STATUS "[debug_and_profile_helper] Eigen 3.3.7+ not found, using FetchContent to get it. 
    To speed up the build, consider installing Eigen by 
           `sudo apt install libeigen3-dev`
    Use noetic + focal as the target platform for apt or install Eigen 3.3.7+ manually.")
  find_package(Eigen3 3.3.7 REQUIRED)
else()
  message(STATUS "[debug_and_profile_helper] Eigen 3.3.7 found locally")
endif()

## copy the header files for config files
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/debug_and_profile_helper/config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/debug_and_profile_helper/config.h
)

## if this file is built by catkin, enable catkin package settings
if (DBGNPROF_COMPILE_IN_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
  )

  catkin_package(
    INCLUDE_DIRS 
      include
      ${CMAKE_CURRENT_BINARY_DIR}/include
    LIBRARIES 
      debug_and_profile_helper
    CATKIN_DEPENDS
      roscpp
    DEPENDS
      EIGEN3
  )
endif()

###########
## Build ##
###########

## Specify additional locations of header files, set up the library in two conditions
add_library(debug_and_profile_helper SHARED
  src/helper_class.cpp
)
if (DBGNPROF_COMPILE_IN_ROS)
  target_link_libraries(debug_and_profile_helper PUBLIC
    ${catkin_LIBRARIES}
  )
  target_include_directories(debug_and_profile_helper PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
endif()

## Link the library with spdlog and, set include directories and compile features
target_link_libraries(debug_and_profile_helper PUBLIC
  Eigen3::Eigen
  spdlog::spdlog 
)
target_include_directories(debug_and_profile_helper PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)
target_compile_features(debug_and_profile_helper PUBLIC 
  cxx_std_17
)

## if BUILD_EXAMPLE is ON, build example
if (DBGNPROF_BUILD_EXAMPLE)
  # the list of example names in the example folder
  list(APPEND example_names
    1-raw_LoggerFile_usage
  )

  # the list of example names with ROS functionalities
  if (DBGNPROF_COMPILE_IN_ROS)
    list(APPEND example_names
      2-raw_LoggerROS_usage
      3-macro_combine_standalone
    )
  endif()

  # build each example
  foreach(example_name IN LISTS example_names)
    add_executable(${example_name} example/${example_name}.cpp)
    target_link_libraries(${example_name} PUBLIC debug_and_profile_helper)
  endforeach()
endif()

#############
## Install ##
#############

if (DBGNPROF_COMPILE_IN_ROS) # Install when ROS environment is activated
  # Install Python scripts
  catkin_install_python(PROGRAMS
    scripts/record_as_rosbag.py
    scripts/log_file_parser.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

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

else() # Install when ROS environment is not activated

endif()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_debug_and_profile_helper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
