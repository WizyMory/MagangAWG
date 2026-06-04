#!/usr/bin/env python3

import math
import os
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


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


class NavWebNode(Node):
    def __init__(self):
        super().__init__('amr_nav_web_node')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('pose_topic', 'amcl_pose')
        self.declare_parameter('path_topic', 'plan')
        self.declare_parameter('initial_pose_topic', 'initialpose')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8020)

        self.map_topic = self.get_parameter('map_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.web_host = self.get_parameter('web_host').value
        self.web_port = self.get_parameter('web_port').value

        self._lock = threading.Lock()
        self._map = None
        self._robot_pose = None
        self._initial_pose = None
        self._goal_pose = None
        self._path = []
        self._last_pose_ts = 0.0
        self._last_path_ts = 0.0
        self._goal_status = 'idle'
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

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

        self.get_logger().info(
            f'Nav web bridge active: map={self.map_topic}, pose={self.pose_topic}, '
            f'path={self.path_topic}, action={self.navigate_action}'
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
        points = []
        step = max(1, len(msg.poses) // 1200)
        for pose_stamped in msg.poses[::step]:
            p = pose_stamped.pose.position
            points.append({'x': float(p.x), 'y': float(p.y)})
        with self._lock:
            self._last_path_ts = time.time()
            self._path = points

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
        return {'ok': True, 'initial_pose': self._initial_pose}

    def send_goal(self, pose: PoseRequest):
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
            self._feedback = {
                'distance_remaining': 0.0,
                'navigation_time': 0.0,
                'estimated_time_remaining': 0.0,
            }

        self.goal_pose_pub.publish(goal_msg.pose)
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return {'ok': True, 'status': 'sending', 'goal_pose': self._goal_pose}

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
            if goal_handle is None:
                self._goal_status = 'idle'
                return {'ok': False, 'status': 'no_active_goal'}
            self._goal_status = 'canceling'

        future = goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)
        return {'ok': True, 'status': 'canceling'}

    def _on_cancel_done(self, future):
        with self._lock:
            self._goal_status = 'canceled'
            self._goal_handle = None

    def get_map(self):
        with self._lock:
            return self._map or {'data': [], 'width': 0, 'height': 0}

    def get_state(self):
        now = time.time()
        with self._lock:
            return {
                'map_ready': self._map is not None,
                'robot_connected': (now - self._last_pose_ts) < 3.0,
                'robot_pose': self._robot_pose,
                'initial_pose': self._initial_pose,
                'goal_pose': self._goal_pose,
                'path': list(self._path),
                'path_age': now - self._last_path_ts if self._last_path_ts else None,
                'goal_status': self._goal_status,
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
