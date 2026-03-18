#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from liv_sync.shared_stamp import ns_to_sec_nsec


class STM32TriggerNode(Node):
    def __init__(self):
        super().__init__('stm32_trigger_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic', '/trigger_pulse')
        self.declare_parameter('expect_stamp_ns', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.expect_stamp_ns = self.get_parameter('expect_stamp_ns').get_parameter_value().bool_value

        self.pub = self.create_publisher(Header, self.topic, 100)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f'Opened serial port {port} at {baudrate} baud.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.001, self.read_serial)

    def parse_line(self, line: str):
        if not line.startswith('TRIG'):
            return None

        parts = [p.strip() for p in line.split(',') if p.strip()]
        seq = '0'
        stamp_ns = None

        if len(parts) >= 2:
            seq = parts[1]
        if self.expect_stamp_ns and len(parts) >= 3:
            try:
                stamp_ns = int(parts[2])
            except ValueError:
                stamp_ns = None

        return seq, stamp_ns

    def read_serial(self):
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parsed = self.parse_line(line)
                if parsed is None:
                    continue

                seq, stamp_ns = parsed
                msg = Header()
                if stamp_ns is None:
                    msg.stamp = self.get_clock().now().to_msg()
                else:
                    sec, nanosec = ns_to_sec_nsec(stamp_ns)
                    msg.stamp.sec = sec
                    msg.stamp.nanosec = nanosec
                msg.frame_id = seq
                self.pub.publish(msg)
                self.get_logger().info(f'Received trigger seq={seq} stamp_ns={stamp_ns}')

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = STM32TriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
