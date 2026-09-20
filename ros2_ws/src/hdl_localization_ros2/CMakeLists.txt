cmake_minimum_required(VERSION 3.10)
project(hdl_localization_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ScanMatchingStatus.msg
  DEPENDENCIES std_msgs geometry_msgs
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(hdl_localization_core STATIC
  src/pose_estimator.cpp
)
target_include_directories(hdl_localization_core PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(hdl_localization_core ${PCL_LIBRARIES} gtsam)
ament_target_dependencies(hdl_localization_core rclcpp)

add_executable(hdl_localization_node src/hdl_localization_node.cpp)
target_link_libraries(hdl_localization_node hdl_localization_core gtsam)

ament_target_dependencies(hdl_localization_node
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs
  pcl_ros pcl_conversions tf2 tf2_ros tf2_eigen tf2_geometry_msgs ndt_omp_ros2
)
target_include_directories(hdl_localization_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(hdl_localization_node "${cpp_typesupport_target}")
target_compile_options(hdl_localization_node PRIVATE ${OpenMP_CXX_FLAGS})

install(TARGETS hdl_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
