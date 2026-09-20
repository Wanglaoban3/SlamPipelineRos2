cmake_minimum_required(VERSION 3.10)
project(hdl_graph_slam_ros2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
# jammy libgeographic-dev ships no CMake config file -> locate directly
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp)
find_library(GEOGRAPHICLIB_LIBRARY NAMES Geographic GeographicLib)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FloorCoeffs.msg
  msg/ScanMatchingStatus.msg
  srv/SaveMap.srv
  srv/LoadGraph.srv
  srv/DumpGraph.srv
  DEPENDENCIES std_msgs geometry_msgs
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(hdl_graph_slam_core STATIC
  src/hdl_graph_slam/graph_slam.cpp
  src/hdl_graph_slam/keyframe.cpp
  src/hdl_graph_slam/information_matrix_calculator.cpp
  src/hdl_graph_slam/map_cloud_generator.cpp
  src/hdl_graph_slam/registrations.cpp
  src/g2o/robust_kernel_io.cpp
)
target_include_directories(hdl_graph_slam_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(hdl_graph_slam_core
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_PCG}
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(hdl_graph_slam_core
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs geographic_msgs
  ndt_omp_ros2 fast_gicp
)
target_compile_options(hdl_graph_slam_core PRIVATE ${OpenMP_CXX_FLAGS})

set(G2O_VARIABLES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS})

add_executable(prefiltering_node src/nodes/prefiltering_node.cpp)
target_link_libraries(prefiltering_node hdl_graph_slam_core)

add_executable(floor_detection_node src/nodes/floor_detection_node.cpp)
target_link_libraries(floor_detection_node hdl_graph_slam_core)

add_executable(scan_matching_odometry_node src/nodes/scan_matching_odometry_node.cpp)
target_link_libraries(scan_matching_odometry_node hdl_graph_slam_core)

add_executable(hdl_graph_slam_node src/nodes/hdl_graph_slam_node.cpp)
target_link_libraries(hdl_graph_slam_node hdl_graph_slam_core ${GEOGRAPHICLIB_LIBRARY} fmt)

foreach(target prefiltering_node floor_detection_node scan_matching_odometry_node hdl_graph_slam_node)
  ament_target_dependencies(${target}
    rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs geographic_msgs
    message_filters pcl_ros pcl_conversions tf2 tf2_ros tf2_eigen
    ndt_omp_ros2 fast_gicp
  )
  target_include_directories(${target} PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GEOGRAPHICLIB_INCLUDE_DIR}
  )
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${target} "${cpp_typesupport_target}")
endforeach()

install(TARGETS prefiltering_node floor_detection_node scan_matching_odometry_node hdl_graph_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
