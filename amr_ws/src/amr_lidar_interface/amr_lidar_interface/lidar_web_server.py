#!/usr/bin/env python3

import math
import os
import threading
import time
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarWebNode(Node):
    def __init__(self):
        super().__init__('amr_lidar_web_node')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('max_points', 720)
        self.declare_parameter('disconnect_timeout', 3.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8010)

        self.scan_topic = (
            self.get_parameter('scan_topic').get_parameter_value().string_value
        )
        self.max_points = (
            self.get_parameter('max_points').get_parameter_value().integer_value
        )
        self.disconnect_timeout = (
            self.get_parameter('disconnect_timeout').get_parameter_value().double_value
        )
        self.web_host = (
            self.get_parameter('web_host').get_parameter_value().string_value
        )
        self.web_port = (
            self.get_parameter('web_port').get_parameter_value().integer_value
        )

        self._lock = threading.Lock()
        self._last_scan_ts = 0.0
        self._scan_data = {
            'connected': False,
            'topic': self.scan_topic,
            'frame_id': '',
            'angle_min': 0.0,
            'angle_max': 0.0,
            'angle_increment': 0.0,
            'range_min': 0.0,
            'range_max': 0.0,
            'scan_time': 0.0,
            'valid_count': 0,
            'total_count': 0,
            'points': [],
        }

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._check_connection)
        self.get_logger().info(f'Lidar web bridge listening on topic: {self.scan_topic}')

    def _on_scan(self, msg: LaserScan):
        points = []
        total_count = len(msg.ranges)
        max_points = max(1, int(self.max_points))
        step = max(1, total_count // max_points)

        for index in range(0, total_count, step):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + (msg.angle_increment * index)
            points.append({
                'a': angle,
                'r': distance,
            })

        with self._lock:
            self._last_scan_ts = time.time()
            self._scan_data = {
                'connected': True,
                'topic': self.scan_topic,
                'frame_id': msg.header.frame_id,
                'angle_min': float(msg.angle_min),
                'angle_max': float(msg.angle_max),
                'angle_increment': float(msg.angle_increment),
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max),
                'scan_time': float(msg.scan_time),
                'valid_count': len(points),
                'total_count': total_count,
                'points': points,
            }

    def _check_connection(self):
        with self._lock:
            if time.time() - self._last_scan_ts > self.disconnect_timeout:
                self._scan_data['connected'] = False

    def get_scan_data(self):
        with self._lock:
            data = dict(self._scan_data)
            data['points'] = list(self._scan_data['points'])
            return data


lidar_node: Optional[LidarWebNode] = None

app = FastAPI(title='AMR Lidar Interface')
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/api/lidar/scan')
def get_lidar_scan():
    if lidar_node is None:
        return {'connected': False, 'points': []}
    return lidar_node.get_scan_data()


@app.get('/api/lidar/status')
def get_lidar_status():
    data = get_lidar_scan()
    return {
        'connected': data.get('connected', False),
        'topic': data.get('topic', 'scan'),
        'frame_id': data.get('frame_id', ''),
        'valid_count': data.get('valid_count', 0),
        'total_count': data.get('total_count', 0),
    }


STATIC_DIR = os.path.join(
    get_package_share_directory('amr_lidar_interface'),
    'static',
)
app.mount('/', StaticFiles(directory=STATIC_DIR, html=True), name='static')


def main(args=None):
    global lidar_node

    import uvicorn

    rclpy.init(args=args)
    lidar_node = LidarWebNode()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = lidar_node.web_host
    port = lidar_node.web_port

    try:
        lidar_node.get_logger().info(f'Lidar web interface: http://{host}:{port}')
        uvicorn.run(app, host=host, port=port)
    finally:
        executor.shutdown()
        lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
