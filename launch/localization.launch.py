import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

CONFIG = '/mnt/h/projects/slam-pipeline-ros2/config/localization.yaml'
ODOM_TO_TF = '/mnt/h/projects/slam-pipeline-ros2/scripts/odom_to_tf.py'


def generate_launch_description():
    hdl_localization = Node(
        package='hdl_localization_ros2',
        executable='hdl_localization_node',
        output='screen',
        parameters=[CONFIG],
    )

    # FAST-LIO /Odometry -> TF robot_odom->base_link (motion prediction for the UKF)
    odom_to_tf = ExecuteProcess(
        cmd=['python3', ODOM_TO_TF],
        output='screen',
    )

    return LaunchDescription([hdl_localization, odom_to_tf])
