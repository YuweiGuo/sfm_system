cmake_minimum_required(VERSION 2.8.3)
project(sfm_ba)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

add_subdirectory(SfMLib)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
#  message_generation
  message_filters
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#add_message_files(
#   FILES
#   BARequestMsg.msg
#)

include_directories(
  SfMLib
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
add_executable(sfm_ba
               src/main.cpp
               src/sfm_ba.cpp
               src/sfm_ba.h
               src/setting.h
)
target_link_libraries(sfm_ba
  SfMBALibrary
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}


# ich habe die Position von ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} umgetauscht, dann geht's
)
