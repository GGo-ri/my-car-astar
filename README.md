# my-car-astar

TurtleBot3가 **A\*로 경로를 계획하고 Pure Pursuit로 주행**하는 ROS 2 Humble 패키지입니다.
주행 중 카메라(YOLOv8n)에 지정한 물체가 가까이 보이면 **비상정지**하고, 사라지면 다시 출발합니다.
Gazebo 시뮬레이션과 실제 TurtleBot3(미로)에서 모두 주행을 확인했습니다.

## 구성

| 파일 | 내용 |
|---|---|
| `src/my_bot/my_bot/integrated_navigation.py` | 주행 노드 (A\* + Pure Pursuit + 비상정지) |
| `src/my_bot/launch/tb3_slam_real.launch.py` | 지도 만들기 (SLAM Toolbox + RViz) |
| `src/my_bot/launch/tb3_localization_real.launch.py` | 위치 추정 (map_server + AMCL + RViz) |
| `src/my_bot/model/yolov8n.pt` | 비상정지용 YOLOv8n 모델 |
| `maps/maze_map.pgm`, `.yaml` | 실제 로봇으로 만든 미로 지도 |

## 실행

로봇에서 `turtlebot3_bringup`과 카메라(`v4l2_camera`)를 띄운 뒤 PC에서 실행합니다.

```bash
ros2 launch my_bot tb3_localization_real.launch.py map:=<경로>/maps/maze_map.yaml
ros2 run my_bot integrated_navigation --ros-args -r /camera/image_raw:=/image_raw
```

RViz에서 `2D Pose Estimate`로 초기 위치를 잡고 `2D Goal Pose`로 목적지를 찍으면 출발합니다.
YOLOv8은 `pip3 install ultralytics`로 설치합니다.

## 비상정지

- 0.3초마다 카메라 영상에 YOLOv8n을 실행합니다.
- 대상 물체(`stop_classes`, 현재 `bottle`)가 신뢰도 0.6 이상이고 **박스 높이가 화면의 35% 이상(가까움)**이면 정지합니다.
- 1초 동안 안 보이면 원래 경로로 이어서 주행합니다.

## 강의 코드에서 바꾼 점

- `/map`, `/amcl_pose` 구독 QoS를 `TRANSIENT_LOCAL`로 변경: 노드를 나중에 실행해도 지도와 위치를 받음
- 목표가 60° 이상 틀어져 있으면 제자리 회전부터: 목표가 뒤에 있을 때 sin(α) ≈ 0이라 반대로 직진하던 문제 해결
- YOLO의 CPU 사용 제한(`OMP_NUM_THREADS=2` 등): CPU를 다 써서 AMCL이 밀리던 문제 해결
- 실제 미로에 맞춰 속도 `linear_vel`을 0.2에서 0.1로 낮춤
- 목적지는 RViz `Nav2 Goal`이 아니라 `2D Goal Pose`(`/goal_pose`)로 찍어야 함
