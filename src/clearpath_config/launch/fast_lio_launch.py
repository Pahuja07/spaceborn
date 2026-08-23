import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    clearpath_config_share = get_package_share_directory('clearpath_config')
    spark_fast_lio_share = get_package_share_directory('spark_fast_lio')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_use = LaunchConfiguration('rviz', default='true')
    
    config_file = os.path.join(clearpath_config_share, 'config', 'velodyne_clearpath.yaml')
    rviz_cfg = os.path.join(spark_fast_lio_share, 'rviz', 'velodyne_mit.rviz')

    fast_lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        namespace='a200_0000',
        output='screen',
        remappings=[
            ('lidar', 'sensors/lidar3d_0/points'),
            ('imu', 'sensors/imu_0/data'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[config_file, {
            'use_sim_time': use_sim_time,
            'common.lidar_frame': 'lidar3d_0_laser',
            'common.imu_frame': 'imu_0_link',
            'common.map_frame': 'odom',
            'common.base_frame': 'a200_0000/base_link',
            'common.visualization_frame': 'base',
            'gravity_alignment.enable_gravity_alignment': False
        }]
    )

    # Static Transform for the LiDAR
    tf_lidar = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        namespace='a200_0000',
        arguments=['0', '0', '0.5', '0', '0', '0', 'a200_0000/base_link', 'lidar3d_0_laser'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )
    
    # Static Transform for the IMU
    tf_imu = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        namespace='a200_0000',
        arguments=['0', '0', '0', '0', '0', '0', 'a200_0000/base_link', 'imu_0_link'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        namespace='a200_0000',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_use)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        fast_lio_node,
        tf_lidar,
        tf_imu,
        rviz_node
    ])
