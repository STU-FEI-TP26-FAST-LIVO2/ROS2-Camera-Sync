#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class MockTriggerNode(Node):
    def __init__(self):
        super().__init__('mock_trigger_node')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('topic', '/trigger_pulse')

        self.rate_hz = max(0.1, self.get_parameter('rate_hz').get_parameter_value().double_value)
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Header, self.topic, 50)
        self.seq = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(f'Mock trigger node started at {self.rate_hz:.2f} Hz on {self.topic}.')

    def tick(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.seq)
        self.pub.publish(msg)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
