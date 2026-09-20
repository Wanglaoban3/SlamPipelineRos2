#!/usr/bin/env bash
# Fast per-file syntax check (no linking, no colcon) for the three ported packages.
# Uses include dirs harvested from existing flags.make if present, else a default set.
set -u
WS=/root/ws
ROS=/opt/ros/humble/include

# default include set (covers everything the three packages need)
DEFAULT_INC="-I$ROS -I$ROS/rclcpp -I$ROS/rcl -I$ROS/rcutils -I$ROS/rcpputils \
-I$ROS/rmw -I$ROS/builtin_interfaces -I$ROS/rosidl_runtime_cpp -I$ROS/rosidl_runtime_c \
-I$ROS/rosidl_typesupport_interface -I$ROS/rosidl_typesupport_cpp -I$ROS/rosidl_typesupport_c \
-I$ROS/rcl_interfaces -I$ROS/std_msgs -I$ROS/std_srvs -I$ROS/sensor_msgs -I$ROS/geometry_msgs \
-I$ROS/nav_msgs -I$ROS/visualization_msgs -I$ROS/geographic_msgs -I$ROS/message_filters \
-I$ROS/tf2 -I$ROS/tf2_ros -I$ROS/tf2_eigen -I$ROS/tf2_geometry_msgs -I$ROS/tf2_msgs \
-I$ROS/ament_index_cpp -I$ROS/libstatistics_collector -I$ROS/statistics_msgs \
-I$ROS/rosgraph_msgs -I$ROS/unique_identifier_msgs -I$ROS/rosidl_typesupport_fastrtps_cpp \
-I$ROS/rosidl_typesupport_fastrtps_c -I$ROS/rosidl_typesupport_introspection_cpp \
-I$ROS/rosidl_typesupport_introspection_c -I$ROS/libyaml_vendor -I$ROS/rcl_yaml_param_parser \
-I$ROS/tracetools -I$ROS/rosidl_default_generators -I$ROS/pcl_conversions -I$ROS/pcl_ros \
-I$ROS/class_loader -I$ROS/composition_interfaces -I$ROS/pcl_msgs -I$ROS/sensor_msgs \
-I/usr/include/pcl-1.12 -I/usr/include/eigen3 -I/usr/local/include \
-I/usr/include/x86_64-linux-gnu/qt5 -I/usr/include/x86_64-linux-gnu/qt5/QtCore \
-I/usr/include/x86_64-linux-gnu/qt5/QtGui -I/usr/include/x86_64-linux-gnu/qt5/QtWidgets \
-I/usr/include/jsoncpp -I/usr/include/vtk-9.1 -I/usr/include/freetype2 \
-I/root/ws/install/ndt_omp_ros2/include -I/root/ws/install/include -I/root/ws/install/fast_gicp/include"

# harvest richer include sets from existing build dirs when available
INC1="$DEFAULT_INC"
if [ -f $WS/build/hdl_graph_slam_ros2/CMakeFiles/hdl_graph_slam_core.dir/flags.make ]; then
  INC1="$INC1 $(grep -m1 'CXX_INCLUDES' $WS/build/hdl_graph_slam_ros2/CMakeFiles/hdl_graph_slam_core.dir/flags.make | sed 's/CXX_INCLUDES = //')"
fi
if [ -d $WS/build/hdl_graph_slam_ros2/rosidl_generator_cpp ]; then
  INC1="$INC1 -I$WS/build/hdl_graph_slam_ros2/rosidl_generator_cpp"
fi
INC2="$DEFAULT_INC -I$WS/src/hdl_localization_ros2/include"
if [ -f $WS/build/hdl_localization_ros2/CMakeFiles/hdl_localization_core.dir/flags.make ]; then
  INC2="$INC2 $(grep -m1 'CXX_INCLUDES' $WS/build/hdl_localization_ros2/CMakeFiles/hdl_localization_core.dir/flags.make | sed 's/CXX_INCLUDES = //')"
fi
if [ -d $WS/build/hdl_localization_ros2/rosidl_generator_cpp ]; then
  INC2="$INC2 -I$WS/build/hdl_localization_ros2/rosidl_generator_cpp"
fi

SRC=/root/ws/src
FAILED=0
check() {
  local inc=$1; shift
  for f in "$@"; do
    if ! g++ -fsyntax-only -std=gnu++17 -fopenmp $inc "$f" 2>/tmp/syn_err.txt; then
      echo "FAIL: $f"
      grep -m4 'error:' /tmp/syn_err.txt
      FAILED=1
    else
      echo "OK: $(basename $f)"
    fi
  done
}

echo "===== hdl_graph_slam_ros2 ====="
check "$INC1 -I$SRC/hdl_graph_slam_ros2/include" \
  $SRC/hdl_graph_slam_ros2/src/hdl_graph_slam/*.cpp \
  $SRC/hdl_graph_slam_ros2/src/g2o/*.cpp \
  $SRC/hdl_graph_slam_ros2/src/nodes/*.cpp

echo "===== hdl_localization_ros2 ====="
check "$INC2" \
  $SRC/hdl_localization_ros2/src/*.cpp

if [ $FAILED -eq 0 ]; then
  echo "ALL_SYNTAX_OK"
else
  echo "SYNTAX_ERRORS_PRESENT"
fi
