import os
# YOLO(PyTorch)와 OpenCV가 CPU 코어를 전부 붙잡고 대기하지 않도록, torch/cv2를 불러오기 전에 설정
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OMP_WAIT_POLICY', 'PASSIVE')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from ultralytics import YOLO
from math import pow, atan2, sqrt, sin, cos, pi
import heapq
import time
import cv2
import numpy as np
import torch

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        # 여기가 바로 당신들이 수정할 파라미터
        self.lookahead_dist = 0.5  # 전방 주시 거리 : 멀리 보냐, 가까이 보냐에 따른 차이 발생함
        self.linear_vel = 0.1      # 이동 속도 : 느리냐, 빠르냐에 따른 차이 발생함
        self.stop_tolerance = 0.15 # 도착 인정 거리 : 가까우면 멈추냐, 멀어도 멈추냐에 따른 차이 발생함

        # 장애물에 부딪히지 않기 위한 안전거리 (x칸)
        self.safety_margin = 4 # 맵을 격자로 구분했을 때의 칸수

        # 비상정지: 카메라에 이 객체가 보이면 즉시 정지, 안 보인 지 release_time초가 지나면 주행 재개
        self.stop_classes = ['bottle'] # 비상정지할 객체 (YOLOv8 COCO 클래스 이름)
        self.stop_conf = 0.6           # 이 신뢰도 이상일 때만 검출로 인정
        self.release_time = 1.0        # 객체가 사라진 뒤 다시 출발하기까지 기다리는 시간 (초)
        self.detect_interval = 0.3     # YOLO 검출 주기 (초): 매 프레임 돌리면 주행 제어가 밀림
        self.detect_imgsz = 320        # YOLO 입력 크기: 가까운 객체만 잡으면 되므로 작게 (640보다 계산량 약 1/4)
        self.min_box_ratio = 0.35      # 박스 높이가 화면 높이의 이 비율 이상일 때만 정지 (= 가까이 있을 때만)

        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0

        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = [] # 계획된 경로
        self.path_index = 0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        # /map과 /amcl_pose는 발행 측이 마지막 메시지를 저장해두는(TRANSIENT_LOCAL) 토픽이라
        # 같은 설정으로 구독해야 노드를 나중에 실행해도 맵과 현재 위치를 바로 받을 수 있음
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, latched_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 비상정지용 YOLOv8 모델과 카메라 구독 (가장 최신 프레임 1장만 받음)
        model_path = os.path.join(get_package_share_directory('my_bot'), 'model', 'yolov8n.pt')
        self.model = YOLO(model_path)
        # PyTorch는 기본으로 CPU 코어를 전부 쓰기 때문에, Gazebo/AMCL/주행 제어가 밀리지 않도록 제한
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.emergency_stop = False
        self.last_seen = 0.0      # 마지막으로 비상정지 객체를 본 시각
        self.last_detect = 0.0    # 마지막으로 YOLO를 돌린 시각
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, image_qos)

        # 이 주기를 바꾸는 것도 주행 퍼포먼스에 영향을 줄 수 있음
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Let's Run!")

    # 카메라 영상에서 비상정지 객체를 찾고, 결과를 창에 표시
    def image_callback(self, msg):
        now = time.time()
        if now - self.last_detect < self.detect_interval:
            return
        self.last_detect = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model(frame, conf=self.stop_conf, imgsz=self.detect_imgsz, verbose=False)[0]

        found = False
        for box in result.boxes:
            name = self.model.names[int(box.cls[0])]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            close = (y2 - y1) / frame.shape[0] >= self.min_box_ratio
            hit = name in self.stop_classes and close
            found = found or hit
            color = (0, 0, 255) if hit else (127, 0, 127)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{name} {float(box.conf[0]):.2f}", (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if found:
            self.last_seen = now
            if not self.emergency_stop:
                self.emergency_stop = True
                self.stop_robot()
                self.get_logger().warn(f"EMERGENCY STOP! {self.stop_classes} detected")
        elif self.emergency_stop and now - self.last_seen > self.release_time:
            self.emergency_stop = False
            self.get_logger().info("Clear. Resume driving.")

        if self.emergency_stop:
            cv2.putText(frame, "EMERGENCY STOP", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        cv2.imshow('Emergency Stop Camera', frame)
        cv2.waitKey(1)

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    # goal_pose 토픽 발행 시
    # 1. 맵을 그리드 형태로 변환한 다음 최단 경로를 구한다.
    # 2. 구한 경로를 다시 원래의 형태로 변환한 다음 발행한다.
    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None: return

        goal_pose = [msg.pose.position.x, msg.pose.position.y]

        # x,y 좌표 값을 그리드 상에서의 행,열 로 변환하는 작업
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)

        self.get_logger().info("Calculating Path...")
        path_grid = self.run_astar(start_grid, goal_grid)

        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info("Path Found! Go!")
        else:
            self.get_logger().warn("No Path Found.")

    # 안전거리(마진)를 유지하는지 체크.... 안전거리보다 좁혀지지 않게!
    def check_safety(self, y, x) :
        margin = self.safety_margin

        for r in range(y - margin, y + margin + 1):
            for c in range(x - margin, x + margin + 1):
                if 0 <= r < self.map_height and 0 <= c < self.map_width:
                    if self.map_data[r][c] != 0: # 0이 아닌 게 장애물
                        return False
        return True

    def run_astar(self, start, end):
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] != 0: continue

                if not self.check_safety(ny, nx) : continue

                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + 1
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    # 경로는 에이스타가 구해줬다는 가정 하에, 나는 이동만 하면 돼!
    def control_loop(self):
        if not self.global_path: return

        # 비상정지 중에는 경로를 유지한 채 멈춰서 기다림
        if self.emergency_stop:
            self.stop_robot()
            return

        final_goal = self.global_path[-1] # 목적지

        # 목적지까지 가기 위해 계산해 본 거리
        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        if dist_to_final < self.stop_tolerance:
            self.global_path = []
            self.stop_robot()
            return

        # 도착 지점의 좌표
        target_x, target_y = final_goal

        # 경로가 점 10개짜리 경로라고 쳐봐....
        # 그러면 10번의 인덱싱을 할 수 있는거야!
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)
            # 내가 주시하는 거리보다 조금 먼 정도의 목적지를 설정
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py # 적절한 타겟으로 갱신!
                self.path_index = i
                break

        # 새로 갱신한 현실적인 목적지까지의 거리 기반으로 계산
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        alpha = atan2(dy, dx) - self.current_yaw

        if alpha > pi: alpha -= 2*pi
        elif alpha < -pi: alpha += 2*pi

        # 목표가 옆이나 뒤에 있으면(60도 이상) 제자리에서 먼저 회전
        # (뒤에 있을 때 sin(alpha)가 0에 가까워져서 돌지 않고 반대로 직진하는 문제 방지)
        if abs(alpha) > pi / 3:
            cmd = Twist()
            cmd.angular.z = 0.6 if alpha > 0 else -0.6
            self.pub_cmd.publish(cmd)
            return

        angular_velocity = self.linear_vel * (2.0 * sin(alpha)) / self.lookahead_dist

        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = angular_velocity

        if cmd.angular.z > 1.0: cmd.angular.z = 1.0
        if cmd.angular.z < -1.0: cmd.angular.z = -1.0

        self.pub_cmd.publish(cmd)

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution), int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0], (grid[0]*self.map_resolution)+self.map_origin[1]]

    # 계획된 경로 발행
    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
