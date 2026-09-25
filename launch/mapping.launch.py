import os
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = '/mnt/h/projects/slam-pipeline-ros2/config/graph_slam.yaml'


def generate_launch_description():
    params = [CONFIG]

    prefiltering = Node(
        package='hdl_graph_slam_ros2',
        executable='prefiltering_node',
        output='screen',
        parameters=params,
    )

    floor_detection = Node(
        package='hdl_graph_slam_ros2',
        executable='floor_detection_node',
        output='screen',
        parameters=params,
    )

    # pure scan-matching odometry (ICP per config) -> /odom
    scan_matching_odometry = Node(
        package='hdl_graph_slam_ros2',
        executable='scan_matching_odometry_node',
        output='screen',
        parameters=params,
    )

    # backend: pose graph + loop closure + floor/IMU factors, consumes /odom + /filtered_points
    hdl_graph_slam = Node(
        package='hdl_graph_slam_ros2',
        executable='hdl_graph_slam_node',
        output='screen',
        parameters=params,
    )

    return LaunchDescription([
        prefiltering,
        floor_detection,
        scan_matching_odometry,
        hdl_graph_slam,
    ])
