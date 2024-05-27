import rclpy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('static_transform_publisher')

    broadcaster = StaticTransformBroadcaster(node)

    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = node.get_clock().now().to_msg()
    static_transform_stamped.header.frame_id = 'odom'
    static_transform_stamped.child_frame_id = 'laser'
    static_transform_stamped.transform.translation.x = 0.0  # Adjust these values as needed
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0
    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transform_stamped)
    node.get_logger().info("Static transform from 'odom' to 'laser' published")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()