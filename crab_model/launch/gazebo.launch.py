import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():

    # Specify the name of the package
    pkg_name = 'crab_model'
    package_share_dir = get_package_share_directory(pkg_name)

    # RViz Configuration
    rviz_config_file = os.path.join(package_share_dir, 'config', 'rviz_config_file.rviz')



    # Set GZ_SIM_RESOURCE_PATH
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_share_dir, 'meshes'),
            ':', # Path separator
            os.path.join(os.getenv('HOME', ''), '.gz', 'sim', 'models'), # User models
        ]
    )


    # Robot State Publisher Launch
    # Ensure the internal launch file also sets use_sim_time=true if it launches robot_state_publisher directly
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_share_dir,'launch','rsp.launch.py'
                )]),
                # Pass use_sim_time and use_ros2_control arguments
                launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


    # Use ros_gz_sim to launch Gazebo
    # '-r' runs the simulation immediately
    # '-s' runs server only (no GUI) - often preferred for launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items() 
    )



    # Spawn Entity
    # Use the 'create' executable from ros_gz_sim to spawn the robot
    # It reads the 'robot_description' topic published by robot_state_publisher
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # Use the standard topic name
            '-name', 'crab',             # Name of the entity in Gazebo
            '-allow_renaming', 'true',     # Allow renaming if the name already exists
            '-x', '0.0',                  # Initial X position
            '-y', '0.0',                  # Initial Y position
            '-z', '0.0'                   # Initial Z position
        ],
        output='screen'
    )

    # Controller Spawning
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen', # Added for better debugging
    )

    # Controller Spawner way to load the PID controller
    # pid_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["pid_position_controller", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )


    # Using ExecuteProcess for loading controllers
    pid_position_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'pid_position_controller'],
        output='screen'
    )

    # ROS/GAZEBO Bridge
    bridge_params = os.path.join(package_share_dir, 'config', 'gz_bridge.yaml')
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    # Event Handlers for Sequencing
    # Delay PID controller until joint_state_broadcaster is running
    delayed_pid_position_controller_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[pid_position_controller_spawner],
            )
        )

    # Delay RViz until joint_state_broadcaster is running (or maybe until PID controller is active?)
    delayed_rviz_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Assemble Launch Description
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delayed_pid_position_controller_spawner,
        delayed_rviz_node,
        bridge_node,
        gz_resource_path
    ])