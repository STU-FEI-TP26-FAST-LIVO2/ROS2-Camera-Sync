#!/usr/bin/env python3
import csv
import json
import os
from collections import deque
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from liv_sync.shared_stamp import stamp_to_ns


class DatasetLoggerNode(Node):
    def __init__(self):
        super().__init__('dataset_logger_node')

        self.declare_parameter('dataset_root', os.path.expanduser('~/ros2_tim_projekt/dataset'))
        self.declare_parameter('trigger_topic', '/trigger_pulse')
        self.declare_parameter('image_topic', '/my_camera/basler_cam/image_raw')
        self.declare_parameter('cloud_topic', '/hesai/pandar')
        self.declare_parameter('max_dt_ms', 200.0)
        self.declare_parameter('trigger_buffer_size', 2000)
        self.declare_parameter('session_name', '')

        dataset_root = self.get_parameter('dataset_root').get_parameter_value().string_value
        session_name = self.get_parameter('session_name').get_parameter_value().string_value.strip()
        if not session_name:
            session_name = datetime.now().strftime('session_%Y%m%d_%H%M%S')

        self.dataset_root = os.path.join(dataset_root, session_name)
        self.images_dir = os.path.join(self.dataset_root, 'images')
        self.clouds_dir = os.path.join(self.dataset_root, 'clouds')
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.clouds_dir, exist_ok=True)

        self.csv_path = os.path.join(self.dataset_root, 'pairs.csv')
        self.bridge = CvBridge()
        self.max_dt_ns = int(self.get_parameter('max_dt_ms').get_parameter_value().double_value * 1e6)
        self.trigger_buffer = deque(maxlen=self.get_parameter('trigger_buffer_size').get_parameter_value().integer_value)
        self.latest_cloud_by_trigger = {}
        self.used_image_seq = set()

        self.create_subscription(Header, self.get_parameter('trigger_topic').get_parameter_value().string_value, self.trigger_cb, 100)
        self.create_subscription(Image, self.get_parameter('image_topic').get_parameter_value().string_value, self.image_cb, 20)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').get_parameter_value().string_value, self.cloud_cb, 20)

        self.init_csv()
        self.get_logger().info(f'Dataset logger started. Saving to: {self.dataset_root}')

    def init_csv(self):
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'seq',
                'image_file',
                'cloud_file',
                'cloud_meta_file',
                'trigger_ns',
                'image_ns',
                'cloud_ns',
                'dt_img_us',
                'dt_cloud_us'
            ])

    def trigger_cb(self, msg: Header):
        self.trigger_buffer.append({
            'seq': msg.frame_id,
            'stamp_ns': stamp_to_ns(msg.stamp)
        })

    def find_best_past_trigger(self, target_ns):
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

    def cloud_cb(self, msg: PointCloud2):
        cloud_ns = stamp_to_ns(msg.header.stamp)
        result = self.find_best_past_trigger(cloud_ns)
        if result is None:
            return

        trig, dt_ns = result
        seq = trig['seq']
        cloud_file = f'cloud_{int(seq):06d}.bin'
        meta_file = f'cloud_{int(seq):06d}.json'

        cloud_path = os.path.join(self.clouds_dir, cloud_file)
        meta_path = os.path.join(self.clouds_dir, meta_file)

        with open(cloud_path, 'wb') as f:
            f.write(bytes(msg.data))

        metadata = {
            'stamp_ns': cloud_ns,
            'height': int(msg.height),
            'width': int(msg.width),
            'point_step': int(msg.point_step),
            'row_step': int(msg.row_step),
            'is_bigendian': bool(msg.is_bigendian),
            'is_dense': bool(msg.is_dense),
            'fields': [
                {
                    'name': field.name,
                    'offset': int(field.offset),
                    'datatype': int(field.datatype),
                    'count': int(field.count),
                }
                for field in msg.fields
            ],
        }
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2)

        self.latest_cloud_by_trigger[seq] = {
            'cloud_file': cloud_file,
            'cloud_meta_file': meta_file,
            'cloud_ns': cloud_ns,
            'dt_cloud_us': dt_ns / 1e3
        }

    def image_cb(self, msg: Image):
        image_ns = stamp_to_ns(msg.header.stamp)
        result = self.find_best_past_trigger(image_ns)
        if result is None:
            self.get_logger().warn('No valid past trigger available for image.')
            return

        trig, dt_ns = result
        seq = trig['seq']
        trig_ns = trig['stamp_ns']

        if seq in self.used_image_seq:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        image_file = f'img_{int(seq):06d}.png'
        image_path = os.path.join(self.images_dir, image_file)
        cv2.imwrite(image_path, cv_image)

        cloud_info = self.latest_cloud_by_trigger.get(seq, None)
        cloud_file = cloud_info['cloud_file'] if cloud_info else ''
        cloud_meta_file = cloud_info['cloud_meta_file'] if cloud_info else ''
        cloud_ns = cloud_info['cloud_ns'] if cloud_info else ''
        dt_cloud_us = cloud_info['dt_cloud_us'] if cloud_info else ''

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                seq,
                image_file,
                cloud_file,
                cloud_meta_file,
                trig_ns,
                image_ns,
                cloud_ns,
                dt_ns / 1e3,
                dt_cloud_us
            ])

        self.used_image_seq.add(seq)
        self.get_logger().info(f'Saved pair seq={seq} image={image_file} cloud={cloud_file}')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
