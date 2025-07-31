import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('crab_model'))
    xacro_file = os.path.join(pkg_path,'description','crab_assembly.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    #robot_description_config = Command(['xacro',xacro_file,'use_ros2_control:=',use_ros2_control,'sim_mode:=',use_sim_time])
    
    # rviz_config_file = os.path.join(pkg_path, 'description', 'crab.rviz')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )



    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value = 'false',
            description = 'Use ros2_control if true'
        ),

        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])