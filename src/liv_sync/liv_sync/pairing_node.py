#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2

from liv_sync.shared_stamp import stamp_to_ns


class PairingNode(Node):
    def __init__(self):
        super().__init__('pairing_node')

        self.declare_parameter('trigger_topic', '/trigger_pulse')
        self.declare_parameter('image_topic', '/my_camera/basler_cam/image_raw')
        self.declare_parameter('cloud_topic', '/hesai/pandar')
        self.declare_parameter('max_dt_ms', 200.0)
        self.declare_parameter('trigger_buffer_size', 1000)

        self.max_dt_ns = int(self.get_parameter('max_dt_ms').get_parameter_value().double_value * 1e6)
        self.trigger_buffer = deque(maxlen=self.get_parameter('trigger_buffer_size').get_parameter_value().integer_value)

        self.create_subscription(Header, self.get_parameter('trigger_topic').get_parameter_value().string_value, self.trigger_cb, 100)
        self.create_subscription(Image, self.get_parameter('image_topic').get_parameter_value().string_value, self.image_cb, 20)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').get_parameter_value().string_value, self.cloud_cb, 20)

        self.get_logger().info(f'Pairing node started. max_dt_ms={self.max_dt_ns / 1e6:.1f}')

    def trigger_cb(self, msg: Header):
        self.trigger_buffer.append({
            'seq': msg.frame_id,
            'stamp_ns': stamp_to_ns(msg.stamp)
        })

    def find_best_past_trigger(self, target_ns):
        if not self.trigger_buffer:
            return None

        best = None
        best_dt = None
        for trig in self.trigger_buffer:
            dt = target_ns - trig['stamp_ns']
            if dt < 0:
                continue
            if dt > self.max_dt_ns:
                continue
            if best is None or dt < best_dt:
                best = trig
                best_dt = dt

        if best is None:
            return None
        return best, best_dt

    def image_cb(self, msg: Image):
        image_ns = stamp_to_ns(msg.header.stamp)
        result = self.find_best_past_trigger(image_ns)
        if result is None:
            self.get_logger().warn('No valid past trigger available for image.')
            return

        trig, dt_ns = result
        self.get_logger().info(
            f'[IMAGE] trigger_seq={trig["seq"]} image_time={image_ns} dt={(dt_ns / 1e6):.3f} ms'
        )

    def cloud_cb(self, msg: PointCloud2):
        cloud_ns = stamp_to_ns(msg.header.stamp)
        result = self.find_best_past_trigger(cloud_ns)
        if result is None:
            self.get_logger().warn('No valid past trigger available for cloud.')
            return

        trig, dt_ns = result
        self.get_logger().info(
            f'[CLOUD] trigger_seq={trig["seq"]} cloud_time={cloud_ns} dt={(dt_ns / 1e6):.3f} ms'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PairingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
