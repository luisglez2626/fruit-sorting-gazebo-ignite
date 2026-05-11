# ==============================================================================
# File: start_gazebo.launch.py
# Purpose: Launches Gazebo and spawns the robot. 
#          Bridging /world/default/pose/info without mapping to /tf to avoid
#          TF_NO_FRAME_ID spam from Ignition's internal links.
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg = 'collaborative_arm_description'
    gazebo_pkg = 'collaborative_arm_gazebo'
    
    urdf_path = os.path.join(get_package_share_directory(desc_pkg), 'urdf', 'collaborative_arm.urdf.xacro')
    world_path = os.path.join(get_package_share_directory(gazebo_pkg), 'worlds', 'fruit_sorting.world')
    
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'collaborative_arm', '-z', '0.0'],
        output='screen'
    )

    # Added explicit mappings to pull the apple positions into ROS 2 for Ground Truth
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/left_cam/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/left_cam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/right_cam/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/right_cam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        bridge_node,
        TimerAction(period=10.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=12.0, actions=[load_arm_controller]),
        TimerAction(period=14.0, actions=[load_gripper_controller]),
    ])