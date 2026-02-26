# 🏎️ ROS2 A* & Pure Pursuit Navigation Project

**A* 알고리즘**을 통한 전역 경로 계획 / **Pure Pursuit**을 이용한 지역 경로 추종을 학습
LiDAR 기반의 동적 회피와 YOLOv8 기반의 객체 인식을 포함한 주행 

## 📂 저장소 구조 (Repository Structure)

| 폴더/파일 | 기능 및 역할 | 핵심 기술 |
|:---:|:---|:---|
| **astar_pkg/** | 자율주행 핵심 알고리즘 소스 코드 폴더 | Python, rclpy |
| └─ `astar_pure.py` | LiDAR 기반 실시간 장애물 회피 주행 | A*, Pure Pursuit, LiDAR |
| └─ `astar_pure_yolo.py` | 주행 중 YOLOv8 기반 객체(휴대폰) 감지 및 정지 | YOLOv8 Integration |
| └─ `basic.py` | 자율주행 시스템의 기본 데이터 처리 프레임워크 | Map, Pose, Goal |
| **resource/** | 패키지 정보 및 인덱스 리소스 | - |
| **yolov8n.pt** | 객체 인식을 위한 YOLOv8 Nano 모델 가중치 파일 | AI Model |

---

## 🚀 주요 기능 (Key Features)

### 1. 전역 및 지역 경로 계획 (Integrated Navigation)
* **A* Planner**: `OccupancyGrid` 지도를 2차원 배열로 변환하여 최단 경로를 탐색합니다.
* **Pure Pursuit**: 전방 주시 거리(`lookahead_dist`)를 기반으로 선속도와 각속도를 계산하여 부드러운 주행을 구현합니다.
* **LiDAR Avoidance**: 라이다 센서 데이터를 3구역으로 분할하여 맵에 없는 장애물 발견 시 즉각 회피합니다.


### 2. 시각 지능형 안전 시스템 (Vision Safety)
* **YOLOv8 Detection**: 주행 중 카메라를 통해 휴대폰(Class ID: 67)이 감지되면 즉시 로봇을 정지시킵니다.
---

## 🛠️ 실행 방법 (Usage)

### 시뮬레이션 환경 실행 (Gazebo + RViz + Localization)
```bash
$ ros2 launch astar_pkg main.launch.py
```
