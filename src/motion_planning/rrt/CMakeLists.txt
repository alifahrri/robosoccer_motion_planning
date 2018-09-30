cmake_minimum_required(VERSION 2.8.3)
project(rrt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)
if(${PYTHON_VERSION_MAJOR} EQUAL 3)
    FIND_PACKAGE(Boost COMPONENTS python3 system core)
elseif()
    FIND_PACKAGE(Boost COMPONENTS python system core)
endif()

find_package(PkgConfig)
PKG_CHECK_MODULES(lapacke REQUIRED lapacke)
PKG_CHECK_MODULES(nanoflann REQUIRED nanoflann)

INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(include)

add_executable(${PROJECT_NAME}_integrator2d src/integrator2d_node.cpp include/integrator2d.hpp include/integrator2drrt.hpp  include/kronecker.hpp include/lyapunovsolver.hpp include/integrator.hpp include/statespace.hpp include/statespacesolver.hpp include/lqrsolver.hpp include/vectorwrapper.hpp include/schursorted.hpp include/timer.hpp include/logger.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/polynomial.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/states.hpp include/environment.hpp include/collision.hpp include/random.hpp)

add_executable(${PROJECT_NAME}_integrator2d_test src/integrator2d_test.cpp include/integrator2d.hpp include/integrator2drrt.hpp include/kronecker.hpp include/lyapunovsolver.hpp include/integrator.hpp include/statespace.hpp include/statespacesolver.hpp include/lqrsolver.hpp include/vectorwrapper.hpp include/schursorted.hpp include/timer.hpp include/logger.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/polynomial.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/states.hpp include/environment.hpp include/collision.hpp include/random.hpp)

add_executable(${PROJECT_NAME}_integrator2d_dataset src/integrator2d_dataset_node.cpp include/integrator2drrt.hpp include/integrator2d.hpp include/kronecker.hpp include/lyapunovsolver.hpp include/integrator.hpp include/statespace.hpp include/statespacesolver.hpp include/lqrsolver.hpp include/vectorwrapper.hpp include/schursorted.hpp include/timer.hpp include/logger.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/polynomial.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/states.hpp include/environment.hpp include/collision.hpp include/random.hpp)

add_executable(${PROJECT_NAME}_geometric3d src/geometric3d_demo.cpp include/integrator2d.hpp include/kronecker.hpp include/lyapunovsolver.hpp include/integrator.hpp include/statespace.hpp include/statespacesolver.hpp include/lqrsolver.hpp include/vectorwrapper.hpp include/schursorted.hpp include/timer.hpp include/logger.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/polynomial.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/collision.hpp include/random.hpp)

add_executable(${PROJECT_NAME}_geometric2d src/geometric2d_demo.cpp include/integrator2d.hpp include/kronecker.hpp include/lyapunovsolver.hpp include/integrator.hpp include/statespace.hpp include/statespacesolver.hpp include/lqrsolver.hpp include/vectorwrapper.hpp include/schursorted.hpp include/timer.hpp include/logger.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/polynomial.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/collision.hpp include/random.hpp)

target_link_libraries(${PROJECT_NAME}_integrator2d_test ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_dataset ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_geometric3d ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_geometric2d ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rrt_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
