import os
from launch import LaunchDescription
from launch_ros.actions import Node

FASTLIO_CONFIG = '/root/data/fastlio_nuscenes.yaml'
EKF_CONFIG = '/mnt/h/projects/slam-pipeline-ros2/config/ekf.yaml'


def generate_launch_description():
    fastlio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[FASTLIO_CONFIG],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[EKF_CONFIG],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[EKF_CONFIG],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/filtered'),
            ('odometry/gps', 'odometry/gps'),
        ],
    )

    # base_link -> gps identity TF: navsat_transform needs it to finalize its datum transform
    static_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gps'],
    )

    return LaunchDescription([fastlio, ekf, navsat, static_gps_tf])
