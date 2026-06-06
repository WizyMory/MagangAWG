#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
from collections import deque
from datetime import datetime
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


class LidarCSVLogger:
    """Menulis timeline aktivitas lidar ke CSV untuk troubleshooting teknisi."""

    def __init__(self, log_dir: str):
        self._log_dir = os.path.expanduser(log_dir)
        os.makedirs(self._log_dir, exist_ok=True)
        fname = datetime.now().strftime('%Y-%m-%d') + '_lidar.csv'
        self._path = os.path.join(self._log_dir, fname)
        self._lock = threading.Lock()

        if not os.path.exists(self._path):
            with open(self._path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'tag', 'level', 'message'])

    @property
    def path(self):
        return self._path

    def log(self, tag: str, level: str, message: str):
        ts = datetime.now().isoformat(timespec='microseconds')
        with self._lock:
            try:
                with open(self._path, 'a', newline='') as f:
                    csv.writer(f).writerow([ts, tag, level, message])
            except Exception:
                pass

    def close(self):
        pass


class LidarWebNode(Node):
    def __init__(self):
        super().__init__('amr_lidar_web_node')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('max_points', 720)
        self.declare_parameter('disconnect_timeout', 3.0)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8010)
        self.declare_parameter(
            'log_dir',
            os.path.expanduser('~/amr_ws/src/amr_lidar_interface/logs'),
        )
        self.declare_parameter('log_history_limit', 300)
        self.declare_parameter('running_log_interval', 30.0)

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
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.log_history_limit = max(
            20,
            self.get_parameter('log_history_limit').get_parameter_value().integer_value,
        )
        self.running_log_interval = max(
            5.0,
            self.get_parameter('running_log_interval').get_parameter_value().double_value,
        )

        self.logger_csv = LidarCSVLogger(self.log_dir)
        self._log_lock = threading.Lock()
        self._event_log = deque(maxlen=self.log_history_limit)
        self._lock = threading.Lock()
        self._started_ts = time.time()
        self._last_scan_ts = 0.0
        self._last_running_log_ts = 0.0
        self._last_empty_scan_warning_ts = 0.0
        self._disconnect_warning_logged = False
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
        self._log_event(
            'START',
            'running',
            (
                f'Lidar web bridge listening topic={self.scan_topic} '
                f'max_points={self.max_points} timeout={self.disconnect_timeout}s'
            ),
        )
        self._log_event('LOGGER', 'running', f'Lidar log file: {self.logger_csv.path}')

    def _log_event(self, tag: str, level: str, message: str):
        ts = datetime.now().isoformat(timespec='microseconds')
        event = {
            'ts': ts,
            'ts_display': ts.replace('T', ' ')[:19],
            'tag': tag,
            'level': level,
            'message': message,
        }

        with self._log_lock:
            self._event_log.append(event)

        self.logger_csv.log(tag, level, message)

        if level == 'error':
            self.get_logger().error(f'[{tag}] {message}')
        elif level == 'warning':
            self.get_logger().warn(f'[{tag}] {message}')
        else:
            self.get_logger().info(f'[{tag}] {message}')

    def _on_scan(self, msg: LaserScan):
        try:
            points = []
            invalid_count = 0
            out_of_range_count = 0
            total_count = len(msg.ranges)
            max_points = max(1, int(self.max_points))
            step = max(1, total_count // max_points)

            for index in range(0, total_count, step):
                distance = float(msg.ranges[index])
                if not math.isfinite(distance):
                    invalid_count += 1
                    continue
                if distance < msg.range_min or distance > msg.range_max:
                    out_of_range_count += 1
                    continue

                angle = msg.angle_min + (msg.angle_increment * index)
                points.append({
                    'a': angle,
                    'r': distance,
                })

            now = time.time()
            with self._lock:
                was_connected = self._scan_data.get('connected', False)
                self._last_scan_ts = now
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

            self._disconnect_warning_logged = False

            if not was_connected:
                self._last_running_log_ts = now
                self._log_event(
                    'CONNECT',
                    'running',
                    (
                        f'LaserScan received frame={msg.header.frame_id or "-"} '
                        f'valid={len(points)}/{total_count}'
                    ),
                )
            elif now - self._last_running_log_ts >= self.running_log_interval:
                self._last_running_log_ts = now
                self._log_event(
                    'RUNNING',
                    'running',
                    (
                        f'Lidar scan running frame={msg.header.frame_id or "-"} '
                        f'valid={len(points)}/{total_count}'
                    ),
                )

            if total_count == 0 and now - self._last_empty_scan_warning_ts >= 10.0:
                self._last_empty_scan_warning_ts = now
                self._log_event('SCAN', 'warning', 'LaserScan received with empty ranges')
            elif (
                total_count > 0
                and len(points) == 0
                and now - self._last_empty_scan_warning_ts >= 10.0
            ):
                self._last_empty_scan_warning_ts = now
                self._log_event(
                    'SCAN',
                    'warning',
                    (
                        'LaserScan has no valid sampled points '
                        f'invalid={invalid_count} out_of_range={out_of_range_count}'
                    ),
                )
        except Exception as exc:
            self._log_event('SCAN', 'error', f'Failed to process LaserScan: {exc}')

    def _check_connection(self):
        should_log_warning = False
        elapsed = 0.0
        with self._lock:
            now = time.time()
            last_scan_ts = self._last_scan_ts or self._started_ts
            elapsed = now - last_scan_ts
            if elapsed > self.disconnect_timeout:
                was_connected = self._scan_data.get('connected', False)
                self._scan_data['connected'] = False
                should_log_warning = was_connected or not self._disconnect_warning_logged
                self._disconnect_warning_logged = True

        if should_log_warning:
            self._log_event(
                'DISCONNECT',
                'warning',
                f'No LaserScan data for {elapsed:.1f}s on topic={self.scan_topic}',
            )

    def get_scan_data(self):
        with self._lock:
            data = dict(self._scan_data)
            data['points'] = list(self._scan_data['points'])
            return data

    def get_event_log(self, limit: int = 100, level: Optional[str] = None):
        limit = max(1, min(int(limit), self.log_history_limit))
        level_filter = (level or '').strip().lower()

        with self._log_lock:
            events = list(self._event_log)

        if level_filter:
            events = [
                event for event in events
                if event.get('level', '').lower() == level_filter
            ]

        return events[-limit:]


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
    latest_event = None
    if lidar_node is not None:
        events = lidar_node.get_event_log(limit=1)
        latest_event = events[-1] if events else None

    return {
        'connected': data.get('connected', False),
        'topic': data.get('topic', 'scan'),
        'frame_id': data.get('frame_id', ''),
        'valid_count': data.get('valid_count', 0),
        'total_count': data.get('total_count', 0),
        'latest_event': latest_event,
    }


@app.get('/api/lidar/logs')
def get_lidar_logs(limit: int = 100, level: Optional[str] = None):
    if lidar_node is None:
        return {'events': [], 'log_file': ''}
    return {
        'events': lidar_node.get_event_log(limit=limit, level=level),
        'log_file': lidar_node.logger_csv.path,
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
        lidar_node._log_event('WEB', 'running', f'Lidar web interface: http://{host}:{port}')
        uvicorn.run(app, host=host, port=port)
    except Exception as exc:
        lidar_node._log_event('WEB', 'error', f'Lidar web interface stopped by error: {exc}')
        raise
    finally:
        if lidar_node is not None:
            lidar_node._log_event('SHUTDOWN', 'warning', 'Lidar web server shutting down')
        executor.shutdown()
        if lidar_node is not None:
            lidar_node.logger_csv.close()
            lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
