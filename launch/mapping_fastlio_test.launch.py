import os
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = '/mnt/h/projects/slam-pipeline-ros2/config/graph_slam.yaml'
FASTLIO_CONFIG = '/root/data/fastlio_nuscenes.yaml'


def generate_launch_description():
    params = [CONFIG]

    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[FASTLIO_CONFIG],
    )

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

    hdl_graph_slam = Node(
        package='hdl_graph_slam_ros2',
        executable='hdl_graph_slam_node',
        output='screen',
        parameters=params,
    )

    return LaunchDescription([
        fastlio,
        prefiltering,
        floor_detection,
        hdl_graph_slam,
    ])
