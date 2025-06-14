"""
###########################################################################################################################################################
Launch File: launch_rover.launch.py

Description:
This ROS 2 launch file sets up a complete simulation environment using Gazebo and ROS-GZ bridge.
It performs the following actions:
- Loads the robot description from a separate launch file (rsp.launch.py).
- Declares and loads a default Gazebo world (with option to override via command line).
- Launches the Gazebo simulator using ros_gz_sim.
- Spawns the robot entity into the simulation using the robot_description topic.
- Bridges topics between Gazebo and ROS using ros_gz_bridge with a configuration file.
- Bridges the robot's camera image topic using ros_gz_image.
###########################################################################################################################################################
"""


import os

# Get the path to a package's share directory

from ament_index_python.packages import get_package_share_directory 

# Core launch functionalities
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# For launching ROS 2 nodes
from launch_ros.actions import Node



def generate_launch_description():

	# Name of the package containing the robot description and related resources
	package_name='roverproject'

	# Location of the meshes used in rviz2  
	roverproject_mesh_path = os.path.join(get_package_share_directory(package_name), 'meshes')


	# Include the robot state publisher launch file (rsp.launch.py)
    # Pass arguments to use simulation time and disable ros2_control
	rsp = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory(package_name),'launch','rsp.launch.py'
		)]),
		launch_arguments={
			'use_sim_time': 'false', 
			'use_ros2_control': 'true'
		}.items()
	)

	joystick = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([os.path.join(
			get_package_share_directory(package_name),'launch','joystick.launch.py'
		)]),
		launch_arguments={
			'use_sim_time': 'true'
		}.items()
	)

	twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
	twist_mux = Node(
		package="twist_mux",
		executable="twist_mux",
		parameters=[twist_mux_params, {'use_sim_time': True}],
		remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
	)

	twist_stamper = Node(
		package='twist_stamper',
		executable='twist_stamper',
		parameters=[{'use_sim_time': True}],
		remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
					('/cmd_vel_out','/diff_cont/cmd_vel')]
	)

	robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
	controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

	controller_manager = Node(
		package="controller_manager",
		executable="ros2_control_node",
		parameters=[{'robot_description': robot_description},
                    controller_params_file],
	)

	delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


	diff_drive_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["diff_cont"],
	)

	delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

	joint_broad_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["joint_broad"],
	)

	delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

	# Combine and return all launch elements
	return LaunchDescription([
		#banner,
        rsp,
        joystick,
        twist_mux,
        twist_stamper,
		delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
