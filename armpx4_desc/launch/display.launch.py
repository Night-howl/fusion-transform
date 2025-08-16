from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    armpx4_desc_dir = get_package_share_directory('armpx4_desc')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(armpx4_desc_dir, 'urdf', 'arm_drone.urdf.xacro')
    rviz_config = os.path.join(armpx4_desc_dir, 'rviz', 'rviz_config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        Node(
            package='armpx4_tf',
            executable='arm_tf',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': 'empty.sdf -r'}.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description']
        ),
    ])
