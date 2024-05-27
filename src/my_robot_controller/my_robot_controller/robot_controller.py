import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
import subprocess
import os
import signal

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.launch_processes = []
        self.declare_parameter('cmd_vel', '/cmd_vel')
        self.check_topic = self.get_parameter('cmd_vel').get_parameter_value().string_value
        
        self.create_timer(1.0, self.check_publishers)
    
    def check_publishers(self):
        publishers_count = self.count_publishers(self.check_topic)
        self.get_logger().info(f'Number of publishers on {self.check_topic}: {publishers_count}')
        
        if publishers_count > 0 and not self.launch_processes:
            self.get_logger().info('At least one publisher detected. Launching processes.')
            self.launch_processes.append(subprocess.Popen(['ros2', 'launch', 'sllidar_ros2', 'sllidar_a1_launch.py'], preexec_fn=os.setsid))
            self.launch_processes.append(subprocess.Popen(['ros2', 'launch', 'rf2o_laser_odometry', 'rf2o_laser_odometry.launch.py'], preexec_fn=os.setsid))
            self.launch_processes.append(subprocess.Popen(['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py'], preexec_fn=os.setsid))
        elif publishers_count == 0 and self.launch_processes:
            self.get_logger().info('No publishers detected. Terminating processes.')
            for process in self.launch_processes:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            self.launch_processes = []

    def destroy_node(self):
        self.get_logger().info('Destroying node and terminating processes.')
        for process in self.launch_processes:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()