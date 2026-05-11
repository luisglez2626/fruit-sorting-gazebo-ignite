# ==============================================================================
# File: system.launch.py
# Purpose: The master launch file for the collaborative arm project. It includes 
#          the Gazebo simulation launch file and starts the kinematics, vision, 
#          and GUI application nodes simultaneously.
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Retrieves the path to the simulation launch file
    gazebo_pkg_dir = get_package_share_directory('collaborative_arm_gazebo')
    gazebo_launch_path = os.path.join(gazebo_pkg_dir, 'launch', 'start_gazebo.launch.py')

    # Includes the simulation launch file
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # Defines the Inverse Kinematics solver node (C++)
    ik_solver = Node(
        package='collaborative_arm_kinematics',
        executable='ik_solver_node',
        name='ik_solver_node',
        output='screen'
    )

    # Defines the computer vision node (Python)
    vision_detector = Node(
        package='collaborative_arm_vision',
        executable='vision_detector',
        name='vision_detector',
        output='screen'
    )

    # Defines the GUI application node (Python)
    gui_application = Node(
        package='collaborative_arm_application',
        executable='cartesian_jogger',
        name='visual_jogger',
        output='screen'
    )

    return LaunchDescription([
        # Starts the Gazebo simulation and the IK solver immediately
        gazebo_simulation,
        ik_solver,
        
        # Adds a slight delay to the vision and GUI nodes to ensure Gazebo 
        # is fully loaded and TF frames are being broadcasted first.
        TimerAction(period=6.0, actions=[vision_detector]),
        TimerAction(period=8.0, actions=[gui_application])
    ])