import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from math import atan2
import numpy as np

class Map(Node):
    def __init__(self):
        super().__init__('step2_pose_goal')
        
        self.current_pose = [0.0, 0.0] # 현재 위치
        self.current_yaw = 0.0 # 현재 각도
        self.goal_pose = [0.0, 0.0] # 목적지 

        self.map_data = None
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def pose_callback(self, msg):
        # x, y 위치
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        q = msg.pose.pose.orientation # 쿼터니언
        
        # 쿼터니언은 회전을 나타내는 4개 값의 묶음으로 로봇의 방향 정보
        # 다음은 쿼터니언을 회전 각도를 나타내는 yaw(angular.z) 값으로 변환하는 공식
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
        
        self.get_logger().info(f"Current: {self.current_pose}, Yaw: {self.current_yaw:.2f}")

    def goal_callback(self, msg):
        self.goal_pose = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info(f"New Goal: {self.goal_pose}")
    
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

        # 맵 데이터는 1차원 리스트 형태이므로, 다루기 쉬운 2차원 형태로 reshape 하기
        # 이렇게 하면 특정 좌표가 벽인지 길인지 판단 가능. 예를 들면... self.map_data[y][x]의 값을 확인하는 식으로
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        self.get_logger().info(f"Map Received: {self.map_width} x {self.map_height}")

def main(args=None):
    rclpy.init(args=args)
    node = Map()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()