cmake_minimum_required(VERSION 2.8.3)
project(icslat_vo)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")    
SET(VERSION_REQ_OCV "3.1")
SET(VERSION_REQ_CUDA "8.0")


##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" )
   SET(VERSION_REQ_OCV "2.4")
   SET(VERSION_REQ_CUDA "")

   unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
   option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
   message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
   ##to prevent from opencv_dep_cudart dependencies error...
   ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.
endif()


# Add plain cmake packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
)
find_package(ZED 2.0 REQUIRED)
find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)


add_message_files(
   FILES
   msgVO.msg
   msgObstacle.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
  sensor_msgs
  image_transport
  cv_bridge
)


# Include dirs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${Boost_INCLUDE_DIRS})


# Set link libraries
LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES}
)
link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})


# Set build flags
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


################################################################################
# TESTS

## Declare executables
add_executable(vo_node src/main.cpp src/vo_at.h)
add_definitions(-std=c++0x -g -O3)


## Specify libraries to link a library or executable target against
target_link_libraries(vo_node ${LINK_LIBS}
                               ${ZED_LIBRARIES}
                               ${OpenCV_LIBRARIES}
                               ${Boost_LIBRARIES}
                               ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY})


## Add dependencies
add_dependencies(vo_node ${PROJECT_NAME}_gencpp)
