cmake_minimum_required(VERSION 2.8.3)
project(swarm_planner)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g3 -O3 -m64 -Ofast -flto -msse2 -msse3 -march=native  -mfpmath=sse")

set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/include)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#SIPP
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/AA-SIPP-m)
include_directories(${PROJECT_SOURCE_DIR}/third_party/AA-SIPP-m/include)
set( SIPP_SOURCE_FILES
  third_party/AA-SIPP-m/src/tinyxml2.cpp
  third_party/AA-SIPP-m/src/xmlLogger.cpp
  third_party/AA-SIPP-m/src/mission.cpp
  third_party/AA-SIPP-m/src/map.cpp
  third_party/AA-SIPP-m/src/task.cpp
  third_party/AA-SIPP-m/src/config.cpp
  third_party/AA-SIPP-m/src/dynamicobstacles.cpp
  third_party/AA-SIPP-m/src/aa_sipp.cpp
  third_party/AA-SIPP-m/src/constraints.cpp
)


#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio1210)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})

#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime
)

#MATPLOTLIB-CPP
find_package(PythonLibs 2.7)
find_package(Threads)

#BUILD
add_executable(random_map_generator src/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(swarm_traj_planner_rbp src/swarm_traj_planner_rbp.cpp)
target_include_directories(swarm_traj_planner_rbp PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(swarm_traj_planner_rbp
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

add_executable(swarm_traj_planner_rbp_test_all src/swarm_traj_planner_rbp_test_all.cpp)
target_include_directories(swarm_traj_planner_rbp_test_all PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(swarm_traj_planner_rbp_test_all
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

add_executable(swarm_traj_planner_scp src/swarm_traj_planner_scp.cpp)
target_include_directories(swarm_traj_planner_scp PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(swarm_traj_planner_scp
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

#add_executable(swarm_traj_planner_rbp_flat src/swarm_traj_planner_rbp_flat.cpp)
#target_include_directories(swarm_traj_planner_rbp_flat PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(swarm_traj_planner_rbp_flat
#  ${catkin_LIBRARIES}
#  ${OCTOMAP_INCLUDE_DIRS}
#  ${PYTHON_LIBRARIES}
#  m
#  ilocplex
#  cplex
#  concert
#  pthread
#  dl
#)

add_executable(hetero_traj_planner_rbp
  src/hetero_traj_planner_rbp.cpp
  ${SIPP_SOURCE_FILES}
)
target_include_directories(hetero_traj_planner_rbp PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(hetero_traj_planner_rbp
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)

add_executable(swarm_traj_planner_sfc src/swarm_traj_planner_sfc.cpp)
target_include_directories(swarm_traj_planner_sfc PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(swarm_traj_planner_sfc
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)