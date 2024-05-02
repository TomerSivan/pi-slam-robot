import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import xml.etree.ElementTree as ET
import math
from geometry_msgs.msg import TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_description', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.1, self.publish_urdf)

    def publish_urdf(self):
        urdf_file_path = '/home/tomersi2/ros2_ws/src/my_robot_controller/description/my_robot.urdf'
        try:
            with open(urdf_file_path, 'r') as urdf_file:
                urdf_string = urdf_file.read()
                msg = String()
                msg.data = urdf_string
                self.publisher_.publish(msg)
                self.publish_tf(urdf_string)
        except FileNotFoundError:
            self.get_logger().error(f"URDF file '{urdf_file_path}' not found.")

    def publish_tf(self, urdf_string):
        if urdf_string:
            robot = ET.fromstring(urdf_string)
            for joint in robot.findall('.//joint'):
                joint_name = joint.get('name')
                parent_link = joint.find('parent').get('link')
                child_link = joint.find('child').get('link')
                origin = joint.find('origin')
                rpy = [float(x) for x in origin.get('rpy').split()]
                xyz = [float(x) for x in origin.get('xyz').split()]
                transform = self.euler_to_transform(xyz, rpy)
                self.tf_broadcaster_.sendTransform(transform)

    def euler_to_transform(self, xyz, rpy):
        transform = TransformStamped()
        transform.header.frame_id = "world"
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.child_frame_id = "base_link"  # Assuming the base link is the root of the URDF
        transform.transform.translation.x = xyz[0]
        transform.transform.translation.y = xyz[1]
        transform.transform.translation.z = xyz[2]
        quat = self.euler_to_quaternion(*rpy)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        return transform

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    state_publisher = StatePublisher()
    rclpy.spin(state_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()