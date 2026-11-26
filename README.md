# Two-Link Robot Visual Servoing (ROS 2 Humble)

2-DOF Two-Link 로봇과 RealSense D435 카메라를 이용한 ROS 2 ArUco 마커 추종(Visual Servoing) 및 Gazebo 시각화 프로젝트입니다.

## 패키지 구성 (`src/`)

- `marker_detection`: RealSense 컬러 영상을 이용해 ArUco 마커를 인식하고 `/aruco/pose`, `/aruco/image` 발행
- `twolink_msgs`: 로봇 제어 명령(`CMD`) 및 상태 피드백(`XYpos`) 메시지 정의
- `twolink_control`: 시리얼 통신(`serial_node`), 키보드 조작(`command_node`), Visual Servoing 역기구학 제어기(`visual_servoing_node`) 및 원클릭 런치(`visual_servoing.launch.py`)
- `twolink_gazebo`: URDF/Xacro 로봇 모델, Gazebo 및 RViz 시각화 연동(`joint_publisher.py`)

---

## 팀원 초기 세팅 방법

### 1. 저장소 클론
```bash
git clone <공통_GitHub_저장소_URL> ~/ros2_ws
cd ~/ros2_ws
```

### 2. 시스템 의존성 설치
```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rqt-image-view \
  ros-humble-realsense2-camera \
  python3-serial

# 시리얼 포트(/dev/ttyACM0) 접근 권한 부여 (최초 1회, 재로그인 필요)
sudo usermod -aG dialout $USER
```

### 3. 빌드 및 환경 적용
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 실행 방법

### 1. Visual Servoing 원클릭 실행 (카메라 + 마커 인식 + 로봇 제어 + 카메라 뷰)
```bash
ros2 launch twolink_control visual_servoing.launch.py
```

- **Gazebo 시각화까지 동시에 실행**:
  ```bash
  ros2 launch twolink_control visual_servoing.launch.py use_gazebo:=true
  ```
- **파라미터 조정 예시** (목표 거리 0.35m, 카메라 장착 오프셋 각도):
  ```bash
  ros2 launch twolink_control visual_servoing.launch.py target_distance:=0.35 camera_yaw_offset:=0.0
  ```

### 2. 로봇 없이 Gazebo / RViz만 테스트
```bash
# 터미널 1: Gazebo & RViz 실행
ros2 launch twolink_gazebo gazebo.launch.py
# 터미널 2 & 3: 더미 데이터로 움직임 확인
ros2 run twolink_gazebo joint_publisher.py
ros2 run twolink_gazebo dummy_data_publisher.py
```

---

## Git 협업 규칙 (팀 작업 시)
1. 작업 전 항상 최신 코드 받기: `git pull origin main`
2. 작업 후 커밋 & 푸시:
   ```bash
   git add <수정한_파일>
   git commit -m "작업 내용 요약"
   git push origin main
   ```
3. `build/`, `install/`, `log/` 폴더는 `.gitignore`로 제외되어 있으므로 커밋되지 않습니다.
