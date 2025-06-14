"""
Launch File: rsp.launch.py

Description:
This ROS 2 launch file initializes the Robot State Publisher for a robot defined in a XACRO file.
It performs the following actions:
- Declares the `use_sim_time` argument to determine if simulated time should be used.
- Loads and processes the robot's URDF from a `.xacro` file.
- Launches the `robot_state_publisher` node with the generated URDF and simulation time setting.

This launch file is typically included in larger simulation or control stacks and is responsible
for publishing the `tf` transforms of the robot based on the URDF model.
"""

import os

# For locating shared package resources
from ament_index_python.packages import get_package_share_directory


# Core launch functionality
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument


# To launch ROS 2 nodes
from launch_ros.actions import Node

# To process XACRO files into URDF
import xacro


def generate_launch_description():

    # Get the value of 'use_sim_time' from command-line or default
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Locate the package and robot xacro file
    pkg_path = os.path.join(get_package_share_directory('roverproject'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    # Process the xacro file to generate the robot's URDF
    #robot_description_config = xacro.process_file(xacro_file)
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    
    # Set parameters for the robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toxml(), 
        'use_sim_time': use_sim_time
    }

    # Define the node that will publish the robot state (TFs)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Return the full launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
