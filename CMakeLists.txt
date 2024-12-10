cmake_minimum_required(VERSION 3.12...3.28)
project(debug_and_profile_helper)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#add_compile_options(-g) # add debug information

## if this file is built by catkin, USE_ROS is default ON, otherwise OFF
if (DEFINED CATKIN_DEVEL_PREFIX)
  set(USE_ROS_DEFAULT ON)
else()
  set(USE_ROS_DEFAULT OFF)
endif()
option(USE_ROS "Build with ROS support" ${USE_ROS_DEFAULT})

## if this is the top-level CMakeLists.txt, set the BUILD_EXAMPLES on
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(BUILD_EXAMPLES_DEFAULT ON)
else()
  set(BUILD_EXAMPLES_DEFAULT OFF)
endif()
option(BUILD_EXAMPLES "Build examples" ${BUILD_EXAMPLES_DEFAULT})

## display options
message(STATUS 
  "[debug_and_profile_helper] options:
             USE_ROS:    ${USE_ROS}
      BUILD_EXAMPLES:    ${BUILD_EXAMPLES}"
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
    FetchContent_MakeAvailable(spdlog)
else()
    message(STATUS "[debug_and_profile_helper] spdlog 1.15.0 found locally")
endif()

## import Eigen
find_package(Eigen 3.4.0 QUIET)
if (NOT Eigen_FOUND)
  message(STATUS "[debug_and_profile_helper] Eigen not found, using FetchContent to get it")
  FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
  )
  FetchContent_MakeAvailable(eigen)
else()
  message(STATUS "[debug_and_profile_helper] Eigen 3.4.0 found locally")
endif()

## if this file is built by catkin, enable catkin package settings
if (USE_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES debug_and_profile_helper
    CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
  )
endif()

###########
## Build ##
###########

## Specify additional locations of header files, set up the library.
if (USE_ROS)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  add_library(debug_and_profile_helper SHARED
    src/helper_class.cpp
  )

  target_compile_features(debug_and_profile_helper PUBLIC 
    cxx_std_17)
  target_compile_definitions(debug_and_profile_helper PUBLIC 
    USE_ROS)

  target_link_libraries(debug_and_profile_helper
    ${catkin_LIBRARIES}
  )
else()
  add_library(debug_and_profile_helper STATIC
    src/helper_class.cpp
  )

  target_compile_features(debug_and_profile_helper PUBLIC
    cxx_std_17)

  target_include_directories(debug_and_profile_helper PUBLIC
    include
  )
endif()

## Link the library with spdlog in both cases
target_link_libraries(debug_and_profile_helper PRIVATE 
  spdlog::spdlog 
)

## if BUILD_EXAMPLES is ON, build examples
if (BUILD_EXAMPLES)
  # the list of example names in the example folder
  list(APPEND example_names
    1-raw_LoggerFile_usage
  )

  # build each example
  foreach(example_name IN LISTS example_names)
    add_executable(${example_name} example/${example_name}.cpp)
    target_link_libraries(${example_name} PUBLIC
      debug_and_profile_helper
      Eigen3::Eigen)
  endforeach()
endif()

#############
## Install ##
#############

# if (USE_ROS)

# else()
  
# endif()

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_debug_and_profile_helper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
