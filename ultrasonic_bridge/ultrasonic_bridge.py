import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge')

        # Subscriber to raw distance
        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.listener_callback,
            10
        )

        # Publisher for visualization
        self.range_publisher = self.create_publisher(Range, 'ultrasonic_range', 10)

        self.get_logger().info('🚀 Ultrasonic bridge node with RViz started')

    def listener_callback(self, msg):
        distance_cm = msg.data
        self.get_logger().info(f"Received raw distance: {distance_cm:.2f} cm")

        # Convert cm to meters and apply filter
        distance_m = max(0.0, distance_cm / 100.0)

        # Create Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ultrasonic_frame'  # used in RViz
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # ~15 deg cone
        range_msg.min_range = 0.02
        range_msg.max_range = 4.00
        range_msg.range = distance_m

        self.range_publisher.publish(range_msg)
        self.get_logger().info(f"Published to RViz: {distance_m:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
