import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros

class OdometryToBaseFootprintNode(Node):
    def __init__(self):
        super().__init__('odometry_to_base_footprint')

        # Subscribe to the odometry topic
        self.odometry_subscription = self.create_subscription(
            Odometry,
            'odometry_topic_from_slam_toolbox',
            self.odometry_callback,
            10)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info('Odometry to Base Footprint node initialized')

    def odometry_callback(self, msg):
        # Publish Odometry message
        self.odom_publisher.publish(msg)
        self.get_logger().info('Odometry message published')

        # Extract pose from odometry message
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Prepare transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = pose_stamped.pose.position.x
        transform.transform.translation.y = pose_stamped.pose.position.y
        transform.transform.translation.z = pose_stamped.pose.position.z
        transform.transform.rotation = pose_stamped.pose.orientation

        # Publish transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Transform from odom to base_footprint published')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToBaseFootprintNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()