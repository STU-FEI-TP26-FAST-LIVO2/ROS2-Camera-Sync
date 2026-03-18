#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from liv_sync.shared_stamp import SharedStampWriter, stamp_to_ns


class LidarStampBridgeNode(Node):
    def __init__(self):
        super().__init__('lidar_stamp_bridge_node')

        self.declare_parameter('pointcloud_topic', '/hesai/pandar')
        self.declare_parameter('shared_timestamp_path', '/dev/shm/liv_sync_stamp')
        self.declare_parameter('status_period_sec', 2.0)

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.shared_timestamp_path = self.get_parameter('shared_timestamp_path').get_parameter_value().string_value
        self.status_period_sec = self.get_parameter('status_period_sec').get_parameter_value().double_value

        self.writer = SharedStampWriter(self.shared_timestamp_path)
        self.last_stamp_ns = None
        self.last_seq = 0
        self.write_count = 0

        self.sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_cb, 20)
        self.status_timer = self.create_timer(self.status_period_sec, self.status_cb)

        self.get_logger().info(
            f'Lidar stamp bridge started. topic={self.pointcloud_topic} shm={self.shared_timestamp_path}'
        )

    def cloud_cb(self, msg: PointCloud2):
        stamp_ns = stamp_to_ns(msg.header.stamp)
        write_time_ns = self.get_clock().now().nanoseconds
        self.last_seq += 1
        self.writer.write(stamp_ns=stamp_ns, write_time_ns=write_time_ns, seq=self.last_seq)
        self.last_stamp_ns = stamp_ns
        self.write_count += 1

    def status_cb(self):
        if self.last_stamp_ns is None:
            self.get_logger().warn('No LiDAR frames received yet; shared timestamp not updated.')
            return
        self.get_logger().info(
            f'LiDAR->SHM active. writes={self.write_count} last_stamp_ns={self.last_stamp_ns} last_seq={self.last_seq}'
        )

    def destroy_node(self):
        try:
            self.writer.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarStampBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
