#!/usr/bin/env python3
"""Visual servoing controller node for the 2-DOF twolink robot."""

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from twolink_msgs.msg import CMD, XYpos
from visualization_msgs.msg import Marker


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a scalar value to [low, high]."""
    return max(low, min(high, value))


def wrap_angle(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def nearest_angle(target: float, reference: float) -> float:
    """Return the angle equivalent to target closest to reference (avoids 360-deg jumps)."""
    return reference + wrap_angle(target - reference)


def forward_kinematics(
    q1: float, q2: float, l1: float, l2: float
) -> Tuple[float, float, float]:
    """Compute 2-link planar forward kinematics (x, y, theta)."""
    theta = q1 + q2
    x = l1 * math.cos(q1) + l2 * math.cos(theta)
    y = l1 * math.sin(q1) + l2 * math.sin(theta)
    return x, y, theta


def inverse_kinematics(
    x: float,
    y: float,
    l1: float,
    l2: float,
    elbow_sign: float = 1.0,
) -> Tuple[float, float]:
    """
    Compute 2-link planar analytical inverse kinematics with workspace clamping.

    Keeps the target inside the reachable annulus [|L1 - L2|, L1 + L2] with a
    5% safety margin so acos never hits singular boundaries.
    """
    r = math.hypot(x, y)
    r_min = abs(l1 - l2) + 0.015
    r_max = (l1 + l2) * 0.96

    if r < 1e-6:
        x, y, r = r_min, 0.0, r_min
    elif r < r_min:
        scale = r_min / r
        x, y, r = x * scale, y * scale, r_min
    elif r > r_max:
        scale = r_max / r
        x, y, r = x * scale, y * scale, r_max

    cos_q2 = clamp((r * r - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0)
    sign = 1.0 if elbow_sign >= 0.0 else -1.0
    sin_q2 = sign * math.sqrt(max(0.0, 1.0 - cos_q2 * cos_q2))
    q2 = math.atan2(sin_q2, cos_q2)

    k1 = l1 + l2 * cos_q2
    k2 = l2 * sin_q2
    q1 = math.atan2(y, x) - math.atan2(k2, k1)
    return wrap_angle(q1), wrap_angle(q2)


def marker_yaw_error(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Compute horizontal yaw misalignment between marker normal and camera optical axis.

    In solvePnP camera optical frame (+Z forward, +X right, +Y down), the marker
    +Z axis points out of the marker face. When marker faces the camera head-on,
    marker +Z points along -Z_cam (towards the camera).
    """
    # Rotation matrix column 2 (marker Z-axis in camera frame)
    zx = 2.0 * (qx * qz + qw * qy)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    # Ideal facing direction has zx = 0, zz = -1
    return math.atan2(zx, -zz)


class VisualServoingNode(Node):
    """PBVS + 2-link analytical IK controller for ArUco marker tracking."""

    def __init__(self) -> None:
        super().__init__('visual_servoing_node')

        # Robot kinematic parameters (meters / radians)
        self.l1 = float(self.declare_parameter('l1', 0.143).value)
        self.l2 = float(self.declare_parameter('l2', 0.120).value)
        self.camera_yaw_offset = float(
            self.declare_parameter('camera_yaw_offset', 0.0).value
        )

        # Visual servoing control parameters (tuned for responsive, smooth tracking)
        self.target_distance = float(
            self.declare_parameter('target_distance', 0.35).value
        )
        self.kp_dist = float(self.declare_parameter('kp_dist', 1.0).value)
        self.kp_lat = float(self.declare_parameter('kp_lat', 1.1).value)
        self.kp_yaw = float(self.declare_parameter('kp_yaw', 0.0).value)
        self.deadzone_dist = float(
            self.declare_parameter('deadzone_dist', 0.012).value
        )
        self.deadzone_lat = float(
            self.declare_parameter('deadzone_lat', 0.010).value
        )
        self.max_joint_step = float(
            self.declare_parameter('max_joint_step', 0.07).value
        )
        self.max_lead = float(self.declare_parameter('max_lead', 0.18).value)
        self.lpf_alpha = float(self.declare_parameter('lpf_alpha', 0.65).value)
        self.marker_timeout = float(
            self.declare_parameter('marker_timeout', 0.5).value
        )

        # Joint safety limits (radians) - wide enough for free tracking while avoiding physical stops
        self.q1_min = float(self.declare_parameter('q1_min', -2.4).value)
        self.q1_max = float(self.declare_parameter('q1_max', 2.4).value)
        self.q2_min = float(self.declare_parameter('q2_min', 0.10).value)
        self.q2_max = float(self.declare_parameter('q2_max', 2.75).value)
        self.max_q1_offset = float(self.declare_parameter('max_q1_offset', 1.65).value)
        self.max_q2_offset = float(self.declare_parameter('max_q2_offset', 1.45).value)

        # State variables
        self.q1_init: Optional[float] = None
        self.q2_init: Optional[float] = None
        self.q1_curr: Optional[float] = None
        self.q2_curr: Optional[float] = None
        self.q1_des: Optional[float] = None
        self.q2_des: Optional[float] = None

        self.filtered_x_cam: Optional[float] = None
        self.filtered_z_cam: Optional[float] = None
        self.filtered_yaw_err: float = 0.0
        self.last_marker_time: Optional[float] = None

        # ROS 2 interfaces
        self.cmd_pub = self.create_publisher(CMD, 'des_value', 10)
        self.viz_pub = self.create_publisher(Marker, 'visual_servoing/marker_viz', 10)
        self.state_sub = self.create_subscription(
            XYpos, 'robot_state', self.robot_state_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Pose, '/aruco/pose', self.aruco_pose_callback, 10
        )

        # 20 Hz control loop
        self.control_timer = self.create_timer(0.05, self.control_step)
        self.get_logger().info(
            f'VisualServoingNode started (L1={self.l1:.3f}m, L2={self.l2:.3f}m, '
            f'd_des={self.target_distance:.3f}m)'
        )

    def robot_state_callback(self, msg: XYpos) -> None:
        """Update current joint feedback and initialize desired angles once."""
        q1 = float(msg.qpos1)
        q2 = float(msg.qpos2)
        if not (math.isfinite(q1) and math.isfinite(q2)):
            return
        if abs(q1) > 20.0 or abs(q2) > 20.0:
            self.get_logger().warning(
                f'Ignoring corrupted robot_state: q1={q1}, q2={q2}',
                throttle_duration_sec=2.0,
            )
            return

        self.q1_curr = q1
        self.q2_curr = q2
        if self.q1_des is None or self.q2_des is None:
            self.q1_init = self.q1_curr
            self.q2_init = self.q2_curr
            self.q1_des = self.q1_curr
            self.q2_des = self.q2_curr
            # Dynamically bound safe range around initial home pose
            self.q1_min = max(self.q1_min, self.q1_init - self.max_q1_offset)
            self.q1_max = min(self.q1_max, self.q1_init + self.max_q1_offset)
            if self.q2_init < 0.0:
                # If robot starts with negative elbow angle, keep symmetric negative bounds
                self.q2_min = max(-2.55, self.q2_init - self.max_q2_offset)
                self.q2_max = min(-0.20, self.q2_init + self.max_q2_offset)
            else:
                self.q2_min = max(self.q2_min, self.q2_init - self.max_q2_offset)
                self.q2_max = min(self.q2_max, self.q2_init + self.max_q2_offset)
            self.get_logger().info(
                f'Initialized from robot_state: q1={self.q1_curr:.3f} '
                f'[{self.q1_min:.2f}, {self.q1_max:.2f}], '
                f'q2={self.q2_curr:.3f} [{self.q2_min:.2f}, {self.q2_max:.2f}]'
            )

    def aruco_pose_callback(self, msg: Pose) -> None:
        """Filter incoming ArUco marker pose in camera optical frame."""
        x_cam = float(msg.position.x)
        z_cam = float(msg.position.z)
        if not (math.isfinite(x_cam) and math.isfinite(z_cam)) or z_cam <= 0.02:
            return

        yaw_err = marker_yaw_error(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        alpha = self.lpf_alpha
        if self.filtered_x_cam is None or self.filtered_z_cam is None:
            self.filtered_x_cam = x_cam
            self.filtered_z_cam = z_cam
            self.filtered_yaw_err = yaw_err
        else:
            self.filtered_x_cam = alpha * x_cam + (1.0 - alpha) * self.filtered_x_cam
            self.filtered_z_cam = alpha * z_cam + (1.0 - alpha) * self.filtered_z_cam
            self.filtered_yaw_err = (
                alpha * yaw_err + (1.0 - alpha) * self.filtered_yaw_err
            )

        self.last_marker_time = time.monotonic()

    def compute_target_joints(self) -> Optional[Tuple[float, float]]:
        """Compute bounded desired joint angles (q1_raw, q2_raw) from marker pose."""
        if (
            self.q1_curr is None
            or self.q2_curr is None
            or self.filtered_x_cam is None
            or self.filtered_z_cam is None
            or self.last_marker_time is None
        ):
            return None

        if time.monotonic() - self.last_marker_time > self.marker_timeout:
            return None

        err_dist = self.filtered_z_cam - self.target_distance
        err_lat = self.filtered_x_cam

        if abs(err_dist) < self.deadzone_dist:
            err_dist = 0.0
        if abs(err_lat) < self.deadzone_lat:
            err_lat = 0.0

        # End-effector frame displacement (+X forward along camera axis, +Y left)
        dx_ee = self.kp_dist * err_dist
        dy_ee = -self.kp_lat * err_lat

        x_ee, y_ee, theta_ee = forward_kinematics(
            self.q1_curr, self.q2_curr, self.l1, self.l2
        )
        theta_cam = theta_ee + self.camera_yaw_offset

        dx_base = math.cos(theta_cam) * dx_ee - math.sin(theta_cam) * dy_ee
        dy_base = math.sin(theta_cam) * dx_ee + math.cos(theta_cam) * dy_ee

        x_target = x_ee + dx_base
        y_target = y_ee + dy_base

        elbow_sign = 1.0 if self.q2_curr >= 0.0 else -1.0
        q1_ik, q2_ik = inverse_kinematics(
            x_target, y_target, self.l1, self.l2, elbow_sign=elbow_sign
        )

        # Unwrap IK solution to closest continuous angle relative to current joint angle
        q1_ik = nearest_angle(q1_ik, self.q1_curr)
        q2_ik = nearest_angle(q2_ik, self.q2_curr)

        # Optional gentle yaw alignment coupling on joint1
        q1_ik = clamp(
            q1_ik - self.kp_yaw * self.filtered_yaw_err * 0.1,
            self.q1_min,
            self.q1_max,
        )
        q2_ik = clamp(q2_ik, self.q2_min, self.q2_max)
        return q1_ik, q2_ik

    def publish_marker_viz(self, x_target: float, y_target: float) -> None:
        """Publish target marker visualization in base_link frame for RViz."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'visual_servoing'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x_target
        marker.pose.position.y = y_target
        marker.pose.position.z = 0.18
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        self.viz_pub.publish(marker)

    def control_step(self) -> None:
        """Periodic control loop publishing smoothed joint targets."""
        if self.q1_des is None or self.q2_des is None:
            return

        target = self.compute_target_joints()
        if target is None:
            return

        q1_ik, q2_ik = target
        dq1 = clamp(q1_ik - self.q1_des, -self.max_joint_step, self.max_joint_step)
        dq2 = clamp(q2_ik - self.q2_des, -self.max_joint_step, self.max_joint_step)

        q1_next = self.q1_des + dq1
        q2_next = self.q2_des + dq2

        # Prevent desired target from running too far ahead of actual robot feedback
        if self.q1_curr is not None and self.q2_curr is not None:
            q1_next = clamp(
                q1_next, self.q1_curr - self.max_lead, self.q1_curr + self.max_lead
            )
            q2_next = clamp(
                q2_next, self.q2_curr - self.max_lead, self.q2_curr + self.max_lead
            )

        self.q1_des = clamp(q1_next, self.q1_min, self.q1_max)
        self.q2_des = clamp(q2_next, self.q2_min, self.q2_max)

        x_des, y_des, _ = forward_kinematics(
            self.q1_des, self.q2_des, self.l1, self.l2
        )

        cmd_msg = CMD()
        cmd_msg.xdes = x_des
        cmd_msg.ydes = y_des
        cmd_msg.q1des = self.q1_des
        cmd_msg.q2des = self.q2_des
        self.cmd_pub.publish(cmd_msg)

        self.publish_marker_viz(x_des, y_des)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
