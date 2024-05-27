import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Absolute path to the URDF file
    urdf_file = '/home/tomersi2/ros2_ws/src/my_robot_controller/description/my_robot.urdf'

    # Launch description
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': open(urdf_file).read()}]
        ),

        # Static Transform Publisher for base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # Node for motor_control
        Node(
            package='my_robot_controller',
            executable='motor_control',  
            name='motor_control_node',
            output='screen'
        )
    ])




