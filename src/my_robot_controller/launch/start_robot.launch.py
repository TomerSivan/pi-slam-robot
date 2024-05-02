import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(os.getcwd(), 'src', 'my_robot_controller', 'description', 'my_robot.urdf')

    return LaunchDescription([
        # Load URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': open(urdf_file).read()}]
        )
    ])