#!/usr/bin/env python3

import json
import math
import os
import threading
import time
import uuid
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half),
        'w': math.cos(half),
    }


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_to_dict(pose) -> dict:
    return {
        'x': float(pose.position.x),
        'y': float(pose.position.y),
        'yaw': float(quaternion_to_yaw(pose.orientation)),
    }


def rotate_vector_by_quaternion(point, q):
    x, y, z = point
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    tx = 2.0 * ((qy * z) - (qz * y))
    ty = 2.0 * ((qz * x) - (qx * z))
    tz = 2.0 * ((qx * y) - (qy * x))

    return {
        'x': x + (qw * tx) + ((qy * tz) - (qz * ty)),
        'y': y + (qw * ty) + ((qz * tx) - (qx * tz)),
        'z': z + (qw * tz) + ((qx * ty) - (qy * tx)),
    }


def status_to_text(status: int) -> str:
    return {
        GoalStatus.STATUS_UNKNOWN: 'unknown',
        GoalStatus.STATUS_ACCEPTED: 'accepted',
        GoalStatus.STATUS_EXECUTING: 'executing',
        GoalStatus.STATUS_CANCELING: 'canceling',
        GoalStatus.STATUS_SUCCEEDED: 'succeeded',
        GoalStatus.STATUS_CANCELED: 'canceled',
        GoalStatus.STATUS_ABORTED: 'aborted',
    }.get(status, 'unknown')


class PoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class StationRequest(PoseRequest):
    name: str


class MotorRequest(BaseModel):
    enabled: bool


class OperatorWebNode(Node):
    def __init__(self):
        super().__init__('amr_operator_web_node')

        default_station_file = os.path.expanduser('~/.ros/amr_operator_stations.json')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('pose_topic', 'amcl_pose')
        self.declare_parameter('path_topic', 'plan')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('goal_pose_topic', 'goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('compute_path_action', 'compute_path_to_pose')
        self.declare_parameter('goal_send_mode', 'topic')
        self.declare_parameter('cancel_goal_service', '/navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('motor_enable_service', '/amr_motor/enable')
        self.declare_parameter('clear_local_costmap_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('clear_global_costmap_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('battery_voltage_topic', '/amr_motor/bus_voltage')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8030)
        self.declare_parameter('max_laser_points', 1000)
        self.declare_parameter('pose_timeout', 10.0)
        self.declare_parameter('station_file', default_station_file)
        self.declare_parameter('station_tolerance_m', 0.35)
        self.declare_parameter('station_yaw_tolerance_deg', 180.0)
        self.declare_parameter('battery_empty_voltage', 23.0)
        self.declare_parameter('battery_full_voltage', 26.2)

        self.map_topic = self.get_parameter('map_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.compute_path_action = self.get_parameter('compute_path_action').value
        self.goal_send_mode = self.get_parameter('goal_send_mode').value
        self.cancel_goal_service = self.get_parameter('cancel_goal_service').value
        self.motor_enable_service = self.get_parameter('motor_enable_service').value
        self.clear_local_costmap_service = self.get_parameter('clear_local_costmap_service').value
        self.clear_global_costmap_service = self.get_parameter('clear_global_costmap_service').value
        self.battery_voltage_topic = self.get_parameter('battery_voltage_topic').value
        self.web_host = self.get_parameter('web_host').value
        self.web_port = int(self.get_parameter('web_port').value)
        self.max_laser_points = int(self.get_parameter('max_laser_points').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.station_file = os.path.expanduser(self.get_parameter('station_file').value)
        self.station_tolerance_m = float(self.get_parameter('station_tolerance_m').value)
        self.station_yaw_tolerance = math.radians(
            float(self.get_parameter('station_yaw_tolerance_deg').value)
        )
        self.battery_empty_voltage = float(self.get_parameter('battery_empty_voltage').value)
        self.battery_full_voltage = float(self.get_parameter('battery_full_voltage').value)

        self._lock = threading.Lock()
        self._map = None
        self._robot_pose = None
        self._initial_pose = None
        self._goal_pose = None
        self._path = []
        self._path_status = 'idle'
        self._laser_points = []
        self._laser_frame = ''
        self._laser_error = ''
        self._stations = self._load_stations()
        self._last_pose_ts = 0.0
        self._last_path_ts = 0.0
        self._last_scan_ts = 0.0
        self._last_battery_ts = 0.0
        self._goal_status = 'idle'
        self._motor_enabled = False
        self._motor_message = 'unknown'
        self._costmap_message = 'unknown'
        self._battery_voltage = None
        self._battery_percent = None
        self._feedback = {
            'distance_remaining': 0.0,
            'navigation_time': 0.0,
            'estimated_time_remaining': 0.0,
        }
        self._goal_handle = None

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(Path, self.path_topic, self._on_path, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_subscription(Float32, self.battery_voltage_topic, self._on_battery_voltage, 10)
        self.create_subscription(
            GoalStatusArray,
            f'{self.navigate_action}/_action/status',
            self._on_action_status,
            10,
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.cancel_goal_client = self.create_client(CancelGoal, self.cancel_goal_service)
        self.motor_client = self.create_client(SetBool, self.motor_enable_service)
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            self.clear_local_costmap_service,
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            self.clear_global_costmap_service,
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)
        self.planner_client = ActionClient(self, ComputePathToPose, self.compute_path_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'Operator web bridge active: map={self.map_topic}, pose={self.pose_topic}, '
            f'path={self.path_topic}, scan={self.scan_topic}, web_port={self.web_port}'
        )

    def _load_stations(self):
        try:
            with open(self.station_file, 'r', encoding='utf-8') as station_fp:
                data = json.load(station_fp)
        except FileNotFoundError:
            return []
        except Exception as exc:
            self.get_logger().warn(f'Failed to load stations from {self.station_file}: {exc}')
            return []

        stations = data if isinstance(data, list) else data.get('stations', [])
        cleaned = []
        for item in stations:
            try:
                cleaned.append({
                    'id': str(item.get('id') or uuid.uuid4().hex[:8]),
                    'name': str(item.get('name', 'Station')).strip()[:32] or 'Station',
                    'x': float(item['x']),
                    'y': float(item['y']),
                    'yaw': float(item.get('yaw', 0.0)),
                })
            except Exception:
                continue
        return cleaned

    def _save_stations_locked(self):
        try:
            folder = os.path.dirname(self.station_file)
            if folder:
                os.makedirs(folder, exist_ok=True)
            with open(self.station_file, 'w', encoding='utf-8') as station_fp:
                json.dump({'stations': self._stations}, station_fp, indent=2)
        except Exception as exc:
            self.get_logger().error(f'Failed to save stations to {self.station_file}: {exc}')

    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = {
                'frame_id': msg.header.frame_id,
                'resolution': float(msg.info.resolution),
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'origin': pose_to_dict(msg.info.origin),
                'data': list(msg.data),
            }

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._last_pose_ts = time.time()
            self._robot_pose = pose_to_dict(msg.pose.pose)

    def _on_path(self, msg: Path):
        points = self._path_msg_to_points(msg)
        with self._lock:
            self._last_path_ts = time.time()
            self._path = points
            self._path_status = 'topic'

    def _on_scan(self, msg: LaserScan):
        source_frame = msg.header.frame_id or self.scan_topic
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                source_frame,
                Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException as exc:
            with self._lock:
                self._laser_frame = source_frame
                self._laser_error = str(exc)
                self._last_scan_ts = time.time()
            return

        total_count = len(msg.ranges)
        step = max(1, total_count // max(1, self.max_laser_points))
        t = transform.transform.translation
        q = transform.transform.rotation
        points = []

        for index in range(0, total_count, step):
            distance = float(msg.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + (msg.angle_increment * index)
            local = (
                math.cos(angle) * distance,
                math.sin(angle) * distance,
                0.0,
            )
            rotated = rotate_vector_by_quaternion(local, q)
            points.append({
                'x': float(t.x + rotated['x']),
                'y': float(t.y + rotated['y']),
            })

        with self._lock:
            self._laser_points = points
            self._laser_frame = source_frame
            self._laser_error = ''
            self._last_scan_ts = time.time()

    def _on_battery_voltage(self, msg: Float32):
        voltage = float(msg.data)
        span = max(0.1, self.battery_full_voltage - self.battery_empty_voltage)
        percent = max(0.0, min(100.0, ((voltage - self.battery_empty_voltage) / span) * 100.0))
        with self._lock:
            self._battery_voltage = voltage
            self._battery_percent = percent
            self._last_battery_ts = time.time()

    def _on_action_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        status_text = status_to_text(latest.status)
        with self._lock:
            if self._goal_status not in ('idle', 'motor_disabled', 'emergency_stop'):
                self._goal_status = status_text

    def _path_msg_to_points(self, msg: Path):
        points = []
        step = max(1, len(msg.poses) // 1200)
        for pose_stamped in msg.poses[::step]:
            p = pose_stamped.pose.position
            points.append({'x': float(p.x), 'y': float(p.y)})
        return points

    def _make_pose_stamped(self, pose: PoseRequest) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        msg.pose.position.z = 0.0
        quat = yaw_to_quaternion(float(pose.yaw))
        msg.pose.orientation.x = quat['x']
        msg.pose.orientation.y = quat['y']
        msg.pose.orientation.z = quat['z']
        msg.pose.orientation.w = quat['w']
        return msg

    def _publish_zero_velocity(self, count=8):
        stop = Twist()
        for _ in range(count):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.02)

    def _current_station_id_locked(self):
        if self._robot_pose is None:
            return None

        best_id = None
        best_dist = float('inf')
        for station in self._stations:
            dist = math.hypot(
                self._robot_pose['x'] - station['x'],
                self._robot_pose['y'] - station['y'],
            )
            yaw_ok = abs(angle_diff(self._robot_pose['yaw'], station['yaw'])) <= self.station_yaw_tolerance
            if dist <= self.station_tolerance_m and yaw_ok and dist < best_dist:
                best_dist = dist
                best_id = station['id']
        return best_id

    def add_station(self, station: StationRequest):
        name = station.name.strip()
        if not name:
            return {'ok': False, 'message': 'name_required'}
        if len(name) > 32:
            name = name[:32]

        with self._lock:
            if any(item['name'].lower() == name.lower() for item in self._stations):
                return {'ok': False, 'message': 'name_exists'}

            item = {
                'id': uuid.uuid4().hex[:8],
                'name': name,
                'x': float(station.x),
                'y': float(station.y),
                'yaw': float(station.yaw),
            }
            self._stations.append(item)
            self._save_stations_locked()
            return {'ok': True, 'station': item, 'stations': list(self._stations)}

    def delete_station(self, station_id: str):
        with self._lock:
            before = len(self._stations)
            self._stations = [item for item in self._stations if item['id'] != station_id]
            if len(self._stations) == before:
                return {'ok': False, 'message': 'not_found'}
            self._save_stations_locked()
            return {'ok': True, 'stations': list(self._stations)}

    def set_initial_pose(self, pose: PoseRequest):
        self.cancel_goal()
        self._publish_zero_velocity()

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self._make_pose_stamped(pose).pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)

        with self._lock:
            self._initial_pose = {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw}
            self._path = []
            self._path_status = 'idle'
            self._goal_status = 'idle'
        return {'ok': True, 'initial_pose': self._initial_pose}

    def send_goal(self, pose: PoseRequest):
        with self._lock:
            if not self._motor_enabled:
                self._goal_status = 'motor_disabled'
                return {
                    'ok': False,
                    'status': 'motor_disabled',
                    'message': 'Enable motor before sending a navigation goal',
                }

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            with self._lock:
                self._goal_status = 'action_unavailable'
            return {'ok': False, 'status': 'action_unavailable'}

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose_stamped(pose)

        with self._lock:
            self._goal_pose = {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw}
            self._goal_status = 'sending'
            self._path = []
            self._path_status = 'requesting'
            self._feedback = {
                'distance_remaining': 0.0,
                'navigation_time': 0.0,
                'estimated_time_remaining': 0.0,
            }

        self.goal_pose_pub.publish(goal_msg.pose)
        self._request_plan(pose)
        if self.goal_send_mode == 'topic':
            with self._lock:
                self._goal_status = 'sent_topic'
                self._goal_handle = None
            return {'ok': True, 'status': 'sent_topic', 'goal_pose': self._goal_pose}

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return {'ok': True, 'status': 'sending', 'goal_pose': self._goal_pose}

    def send_station_goal(self, station_id: str):
        with self._lock:
            station = next((item for item in self._stations if item['id'] == station_id), None)
        if station is None:
            return {'ok': False, 'message': 'not_found'}
        return self.send_goal(PoseRequest(x=station['x'], y=station['y'], yaw=station['yaw']))

    def _request_plan(self, pose: PoseRequest):
        if not self.planner_client.wait_for_server(timeout_sec=0.2):
            with self._lock:
                self._path_status = 'planner_unavailable'
            return

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = self._make_pose_stamped(pose)
        plan_goal.use_start = False
        plan_goal.planner_id = ''

        future = self.planner_client.send_goal_async(plan_goal)
        future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self._lock:
                self._path_status = f'planner_error: {exc}'
            return

        if not goal_handle.accepted:
            with self._lock:
                self._path_status = 'planner_rejected'
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        try:
            result = future.result()
            points = self._path_msg_to_points(result.result.path)
            status = status_to_text(result.status)
        except Exception as exc:
            points = []
            status = f'planner_error: {exc}'

        with self._lock:
            self._path = points
            self._last_path_ts = time.time()
            self._path_status = status

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self._lock:
                self._goal_status = f'goal_error: {exc}'
            return

        with self._lock:
            self._goal_handle = goal_handle
            if not goal_handle.accepted:
                self._goal_status = 'rejected'
                return
            self._goal_status = 'accepted'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_feedback(self, msg):
        feedback = msg.feedback
        with self._lock:
            self._goal_status = 'executing'
            self._feedback = {
                'distance_remaining': float(feedback.distance_remaining),
                'navigation_time': float(
                    feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9
                ),
                'estimated_time_remaining': float(
                    feedback.estimated_time_remaining.sec
                    + feedback.estimated_time_remaining.nanosec * 1e-9
                ),
            }

    def _on_goal_result(self, future):
        try:
            result = future.result()
            status = status_to_text(result.status)
        except Exception as exc:
            status = f'goal_error: {exc}'

        with self._lock:
            self._goal_status = status
            self._goal_handle = None

    def cancel_goal(self):
        self._publish_zero_velocity()
        with self._lock:
            goal_handle = self._goal_handle
            if goal_handle is None or self.goal_send_mode == 'topic':
                if not self.cancel_goal_client.wait_for_service(timeout_sec=0.5):
                    self._goal_status = 'cancel_unavailable'
                    return {'ok': False, 'status': 'cancel_unavailable'}
                self._goal_status = 'canceling'
                future = self.cancel_goal_client.call_async(CancelGoal.Request())
                future.add_done_callback(self._on_cancel_service_done)
                return {'ok': True, 'status': 'canceling'}
            self._goal_status = 'canceling'

        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)
        return {'ok': True, 'status': 'canceling'}

    def _on_cancel_service_done(self, future):
        try:
            result = future.result()
            status = 'canceled' if result.return_code == CancelGoal.Response.ERROR_NONE else (
                f'cancel_error_{result.return_code}'
            )
        except Exception as exc:
            status = f'cancel_error: {exc}'

        with self._lock:
            if self._goal_status != 'emergency_stop':
                self._goal_status = status
            self._goal_handle = None
            self._path_status = 'idle'

    def _on_cancel_done(self, future):
        with self._lock:
            if self._goal_status != 'emergency_stop':
                self._goal_status = 'canceled'
            self._goal_handle = None
            self._path_status = 'idle'

    def _set_motor_enabled_direct(self, enabled: bool):
        if not self.motor_client.wait_for_service(timeout_sec=0.5):
            with self._lock:
                self._motor_message = 'service_unavailable'
            return {'ok': False, 'enabled': self._motor_enabled, 'message': 'service_unavailable'}

        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.motor_client.call_async(req)
        deadline = time.time() + 3.0

        while time.time() < deadline:
            if future.done():
                resp = future.result()
                with self._lock:
                    if resp.success:
                        self._motor_enabled = bool(enabled)
                    self._motor_message = resp.message
                return {
                    'ok': bool(resp.success),
                    'enabled': self._motor_enabled,
                    'message': resp.message,
                }
            time.sleep(0.02)

        with self._lock:
            self._motor_message = 'timeout'
        return {'ok': False, 'enabled': self._motor_enabled, 'message': 'timeout'}

    def set_motor_enabled(self, enabled: bool):
        if not enabled:
            self.cancel_goal()
            self._publish_zero_velocity()

        return self._set_motor_enabled_direct(enabled)

    def clear_costmaps(self):
        results = []
        for label, client in (
            ('local', self.clear_local_client),
            ('global', self.clear_global_client),
        ):
            if not client.wait_for_service(timeout_sec=0.5):
                results.append({'costmap': label, 'ok': False, 'message': 'service_unavailable'})
                continue

            future = client.call_async(ClearEntireCostmap.Request())
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if future.done():
                    future.result()
                    results.append({'costmap': label, 'ok': True, 'message': 'cleared'})
                    break
                time.sleep(0.02)
            else:
                results.append({'costmap': label, 'ok': False, 'message': 'timeout'})

        ok = all(item['ok'] for item in results)
        with self._lock:
            self._costmap_message = 'cleared' if ok else 'partial'
            if ok:
                self._path = []
                self._path_status = 'cleared'
        return {'ok': ok, 'results': results}

    def emergency_stop(self):
        cancel_result = self.cancel_goal()
        self._publish_zero_velocity(count=15)
        motor_result = self._set_motor_enabled_direct(False)
        self._publish_zero_velocity(count=8)
        with self._lock:
            self._goal_status = 'emergency_stop'
            self._path_status = 'idle'
        return {
            'ok': bool(motor_result.get('ok', False)),
            'status': 'emergency_stop',
            'cancel': cancel_result,
            'motor': motor_result,
        }


    def get_map(self):
        with self._lock:
            return self._map or {'data': [], 'width': 0, 'height': 0}

    def get_state(self):
        now = time.time()
        with self._lock:
            current_station_id = self._current_station_id_locked()
            return {
                'map_ready': self._map is not None,
                'robot_connected': (now - self._last_pose_ts) < self.pose_timeout,
                'robot_pose': self._robot_pose,
                'initial_pose': self._initial_pose,
                'goal_pose': self._goal_pose,
                'path': list(self._path),
                'path_status': self._path_status,
                'path_age': now - self._last_path_ts if self._last_path_ts else None,
                'laser_connected': (now - self._last_scan_ts) < 1.5 and not self._laser_error,
                'laser_points': list(self._laser_points),
                'laser_frame': self._laser_frame,
                'laser_error': self._laser_error,
                'goal_status': self._goal_status,
                'motor_enabled': self._motor_enabled,
                'motor_message': self._motor_message,
                'costmap_message': self._costmap_message,
                'feedback': dict(self._feedback),
                'stations': list(self._stations),
                'current_station_id': current_station_id,
                'battery': {
                    'connected': (now - self._last_battery_ts) < 5.0,
                    'voltage': self._battery_voltage,
                    'percent': self._battery_percent,
                    'age': now - self._last_battery_ts if self._last_battery_ts else None,
                },
            }


operator_node: Optional[OperatorWebNode] = None

app = FastAPI(title='AMR Operator Interface')
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/api/operator/map')
def get_operator_map():
    return operator_node.get_map()


@app.get('/api/operator/state')
def get_operator_state():
    return operator_node.get_state()


@app.post('/api/operator/initial_pose')
def post_initial_pose(pose: PoseRequest):
    return operator_node.set_initial_pose(pose)


@app.post('/api/operator/goal')
def post_goal(pose: PoseRequest):
    return operator_node.send_goal(pose)


@app.post('/api/operator/stations')
def post_station(station: StationRequest):
    return operator_node.add_station(station)


@app.delete('/api/operator/stations/{station_id}')
def delete_station(station_id: str):
    return operator_node.delete_station(station_id)


@app.post('/api/operator/stations/{station_id}/goal')
def post_station_goal(station_id: str):
    return operator_node.send_station_goal(station_id)


@app.post('/api/operator/motor')
def post_motor(req: MotorRequest):
    return operator_node.set_motor_enabled(req.enabled)


@app.post('/api/operator/clear_costmaps')
def post_clear_costmaps():
    return operator_node.clear_costmaps()


@app.post('/api/operator/emergency_stop')
def post_emergency_stop():
    return operator_node.emergency_stop()


STATIC_DIR = os.path.join(
    get_package_share_directory('amr_operator_interface'),
    'static',
)
app.mount('/', StaticFiles(directory=STATIC_DIR, html=True), name='static')


def main(args=None):
    global operator_node

    import uvicorn

    rclpy.init(args=args)
    operator_node = OperatorWebNode()

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        operator_node.get_logger().info(
            f'AMR operator interface: http://{operator_node.web_host}:{operator_node.web_port}'
        )
        uvicorn.run(app, host=operator_node.web_host, port=operator_node.web_port)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        operator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
