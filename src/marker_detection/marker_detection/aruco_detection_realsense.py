import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

class ArucoDetectorNode_Realsense(Node):
    def __init__(self):
        super().__init__('aruco_detector_node_realsense')
        self.bridge = CvBridge()
        self.dashboard = None
        self.tracking_status = 'WAITING FOR CAMERA INFO'
        self.last_image_time = None
        self.marker_id = None
        self.latest_pose = None

        # 카메라 파라미터 저장
        self.camera_matrix = None
        self.dist_coeffs = None

        # 아루코 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.06 # m

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )

        # Publisher
        self.image_pub = self.create_publisher(Image, '/aruco/image', 10)
        self.pose_pub = self.create_publisher(Pose, '/aruco/pose', 10)

        self.dashboard_timer = self.create_timer(0.1, self.refresh_dashboard)

    def render_dashboard(self):
        status = self.tracking_status
        if self.last_image_time is not None and time.monotonic() - self.last_image_time > 2.0:
            status = 'CAMERA TIMEOUT'
        tracking = status == 'TRACKING'
        table = Table.grid(padding=(0, 2))
        table.add_column(min_width=16)
        table.add_column(justify='right', min_width=14)
        table.add_row('STATUS', status)
        table.add_row('MARKER ID', f'{self.marker_id:02d}' if tracking else '--')
        pose = self.latest_pose if tracking else None
        distance = (
            np.linalg.norm([pose.position.x, pose.position.y, pose.position.z])
            if pose is not None else None
        )
        table.add_row('DISTANCE', f'{distance:.3f} m' if distance is not None else '--')
        table.add_row('', '')
        table.add_row('[bold]POSITION (m)[/bold]', '')
        for axis in ('x', 'y', 'z'):
            value = f'{getattr(pose.position, axis):+.4f}' if pose is not None else '--'
            table.add_row(axis.upper(), value)
        table.add_row('', '')
        table.add_row('[bold]QUATERNION[/bold]', '')
        for axis in ('x', 'y', 'z', 'w'):
            value = f'{getattr(pose.orientation, axis):+.4f}' if pose is not None else '--'
            table.add_row(f'q{axis}', value)
        return Panel(
            table, title='ARUCO MARKER TRACKER',
            subtitle='Camera frame | Ctrl+C to exit',
            border_style='green' if tracking else 'yellow',
            expand=False,
        )

    def refresh_dashboard(self):
        if self.dashboard is not None:
            self.dashboard.update(self.render_dashboard(), refresh=True)

    def camera_info_callback(self, msg): # CameraInfo를 받으면 실행되는 callback 함수
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            D = np.array(msg.d, dtype=np.float64)
            self.camera_matrix = K
            self.dist_coeffs = D
            self.tracking_status = 'WAITING FOR IMAGE'

    def image_callback(self, msg): # Image를 받는면 실행되는 callback 함수
        self.last_image_time = time.monotonic()
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        # ROS Image -> OpenCV 이미지
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 아루코 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )
        self.latest_pose = None
        self.marker_id = None
        self.tracking_status = 'NO MARKER'

        estimate = self.my_estimatePoseSingleMarkers(corners, ids) if ids is not None else None
        if ids is not None and estimate is None:
            self.tracking_status = 'POSE ESTIMATION FAILED'
        if estimate is not None: # 마커 검출 및 자세 추정에 성공했다면?
            # 마커 3차원 위치 추정
            rvec, tvec, quaternion, marker_id = estimate

             # 마커 그리기
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 좌표축 그리기 (x: 빨강, y: 초록, z: 파랑)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

            tvec = tvec.flatten()       

            # 결과 Publish
            pose_msg = Pose()
            pose_msg.position.x = tvec[0]
            pose_msg.position.y = tvec[1]
            pose_msg.position.z = tvec[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            self.pose_pub.publish(pose_msg)

            self.latest_pose = pose_msg
            self.marker_id = marker_id
            self.tracking_status = 'TRACKING'

        # OpenCV -> ROS Image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        # 결과 이미지 Publish
        self.image_pub.publish(out_msg)

    def my_estimatePoseSingleMarkers(self, corners, ids):
        # 마커 좌표
        object_points = np.array([
            [-self.marker_length / 2, self.marker_length / 2, 0],   # 좌상단 좌표
            [self.marker_length / 2, self.marker_length / 2, 0],    # 우상단 좌표
            [self.marker_length / 2, -self.marker_length / 2, 0],   # 우하단 좌표
            [-self.marker_length / 2, -self.marker_length / 2, 0]   # 좌하단 좌표
        ], dtype=np.float32)

        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i][0]

                # solvePnP를 사용하여 3차원 위치 추정
                success, rvec, tvec = cv2.solvePnP(object_points, corner, self.camera_matrix , self.dist_coeffs)

                if success:
                    R = Rotation.from_rotvec(rvec.flatten())
                    quaternion = R.as_quat() # xyzw
                    return rvec, tvec, quaternion, int(ids[i][0])

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode_Realsense()
    try:
        with Live(node.render_dashboard(), screen=True, auto_refresh=False) as dashboard:
            node.dashboard = dashboard
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dashboard = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

