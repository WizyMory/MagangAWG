#!/usr/bin/env python3

import math
import os
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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


class MotorRequest(BaseModel):
    enabled: bool


class NavWebNode(Node):
    def __init__(self):
        super().__init__('amr_nav_web_node')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('pose_topic', 'amcl_pose')
        self.declare_parameter('path_topic', 'plan')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('compute_path_action', 'compute_path_to_pose')
        self.declare_parameter('goal_send_mode', 'topic')
        self.declare_parameter('cancel_goal_service', '/navigate_to_pose/_action/cancel_goal')
        self.declare_parameter('motor_enable_service', '/amr_motor/enable')
        self.declare_parameter('clear_local_costmap_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('clear_global_costmap_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('goal_pose_topic', 'web_goal_pose')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8020)
        self.declare_parameter('max_laser_points', 900)
        self.declare_parameter('pose_timeout', 10.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.compute_path_action = self.get_parameter('compute_path_action').value
        self.goal_send_mode = self.get_parameter('goal_send_mode').value
        self.cancel_goal_service = self.get_parameter('cancel_goal_service').value
        self.motor_enable_service = self.get_parameter('motor_enable_service').value
        self.clear_local_costmap_service = self.get_parameter('clear_local_costmap_service').value
        self.clear_global_costmap_service = self.get_parameter('clear_global_costmap_service').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.web_host = self.get_parameter('web_host').value
        self.web_port = self.get_parameter('web_port').value
        self.max_laser_points = self.get_parameter('max_laser_points').value
        self.pose_timeout = self.get_parameter('pose_timeout').value

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
        self._last_pose_ts = 0.0
        self._last_path_ts = 0.0
        self._last_scan_ts = 0.0
        self._goal_status = 'idle'
        self._motor_enabled = False
        self._motor_message = 'unknown'
        self._costmap_message = 'unknown'
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
            f'Nav web bridge active: map={self.map_topic}, pose={self.pose_topic}, '
            f'path={self.path_topic}, scan={self.scan_topic}, action={self.navigate_action}'
        )

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
        step = max(1, total_count // max(1, int(self.max_laser_points)))
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

    def _on_action_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        status_text = status_to_text(latest.status)
        with self._lock:
            if self._goal_status not in ('idle', 'motor_disabled'):
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

    def set_initial_pose(self, pose: PoseRequest):
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._lock:
                self._path_status = 'planner_rejected'
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        result = future.result()
        points = self._path_msg_to_points(result.result.path)
        with self._lock:
            self._path = points
            self._last_path_ts = time.time()
            self._path_status = status_to_text(result.status)

    def _on_goal_response(self, future):
        goal_handle = future.result()
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
        result = future.result()
        with self._lock:
            self._goal_status = status_to_text(result.status)
            self._goal_handle = None

    def cancel_goal(self):
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
        result = future.result()
        with self._lock:
            if result.return_code == CancelGoal.Response.ERROR_NONE:
                self._goal_status = 'canceled'
            else:
                self._goal_status = f'cancel_error_{result.return_code}'
            self._goal_handle = None

    def _on_cancel_done(self, future):
        with self._lock:
            self._goal_status = 'canceled'
            self._goal_handle = None

    def get_map(self):
        with self._lock:
            return self._map or {'data': [], 'width': 0, 'height': 0}

    def set_motor_enabled(self, enabled: bool):
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

    def get_state(self):
        now = time.time()
        with self._lock:
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
            }


nav_node: Optional[NavWebNode] = None

app = FastAPI(title='AMR Navigation Web Interface')
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.get('/api/nav/map')
def get_nav_map():
    return nav_node.get_map()


@app.get('/api/nav/state')
def get_nav_state():
    return nav_node.get_state()


@app.post('/api/nav/initial_pose')
def post_initial_pose(pose: PoseRequest):
    return nav_node.set_initial_pose(pose)


@app.post('/api/nav/goal')
def post_goal(pose: PoseRequest):
    return nav_node.send_goal(pose)


@app.post('/api/nav/cancel')
def post_cancel():
    return nav_node.cancel_goal()


@app.post('/api/motor/enable')
def post_motor_enable(req: MotorRequest):
    return nav_node.set_motor_enabled(req.enabled)


@app.post('/api/nav/clear_costmaps')
def post_clear_costmaps():
    return nav_node.clear_costmaps()


STATIC_DIR = os.path.join(
    get_package_share_directory('amr_nav_interface'),
    'static',
)
app.mount('/', StaticFiles(directory=STATIC_DIR, html=True), name='static')


def main(args=None):
    global nav_node

    import uvicorn

    rclpy.init(args=args)
    nav_node = NavWebNode()

    executor = MultiThreadedExecutor()
    executor.add_node(nav_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        nav_node.get_logger().info(
            f'Navigation web interface: http://{nav_node.web_host}:{nav_node.web_port}'
        )
        uvicorn.run(app, host=nav_node.web_host, port=nav_node.web_port)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        nav_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
