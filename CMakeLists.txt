cmake_minimum_required(VERSION 3.10)
project(degeneracy_detection)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")

if (BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
else()
  add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
  set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "Release")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if($ENV{ROS_VERSION} EQUAL 1)
# ROS1
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_omp
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/voxel_grid_covariance_omp.cpp
  src/ndt_omp.cpp
  src/gicp_omp.cpp
)

add_executable(align
  src/align.cpp
)
add_dependencies(align
  ndt_omp
)
target_link_libraries(align
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
)

#############
## INSTALL ##
#############

install(
  TARGETS
    ndt_omp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install headers
install(DIRECTORY include/pclomp
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
else()

# ROS2
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/voxel_grid_covariance_omp.cpp
  src/ndt_omp.cpp
  src/gicp_omp.cpp
)

target_link_libraries(ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(ndt_omp OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

ament_auto_package()
endif()
