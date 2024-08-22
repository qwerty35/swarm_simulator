cmake_minimum_required(VERSION 3.8)
project(swarm_planner)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

include_directories(${PROJECT_SOURCE_DIR}/include)

#YAML
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
set(dynamicEDT3D_DIR "/opt/ros/iron/share/dynamicEDT3D")
set(DYNAMICEDT3D_LIBRARIES "/opt/ros/iron/lib/libdynamicedt3d.so")
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR /opt/ibm/ILOG/CPLEX_Studio2211)
set(CPLEX_INCLUDE_DIR ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})
set(CPLEX_LIBRARIES
  m
  ilocplex
  cplex
  concert
  pthread
  dl)

#ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(ROS2_PACKAGES
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  ament_index_cpp
  octomap_ros
  octomap_msgs)

#BUILD
add_executable(rbp_traj_planner src/rbp_traj_planner.cpp)
target_link_libraries(rbp_traj_planner
  ${YAML_CPP_LIBRARIES}
  ${CPLEX_LIBRARIES}
)
ament_target_dependencies(rbp_traj_planner ${ROS2_PACKAGES})
install(TARGETS rbp_traj_planner
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config mission log map
        DESTINATION share/${PROJECT_NAME})

ament_package()
