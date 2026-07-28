import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class SpiderDancer(Node):
    def __init__(self):
        super().__init__('spider_dancer')
        # Tạo luồng gửi dữ liệu liên tục
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        # Cứ mỗi 0.1 giây sẽ gọi hàm tính toán 1 lần
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time_step = 0.0

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'joint_coxa_R1', 'joint_femur_R1', 'joint_tibia_R1',
            'joint_coxa_R2', 'joint_femur_R2', 'joint_tibia_R2',
            'joint_coxa_R3', 'joint_femur_R3', 'joint_tibia_R3',
            'joint_coxa_L1', 'joint_femur_L1', 'joint_tibia_L1',
            'joint_coxa_L2', 'joint_femur_L2', 'joint_tibia_L2',
            'joint_coxa_L3', 'joint_femur_L3', 'joint_tibia_L3'
        ]
        
        point = JointTrajectoryPoint()
        
        # Dùng hàm Sin để tạo dao động lên xuống mượt mà từ -0.2 đến 0.2
        offset = math.sin(self.time_step) * 0.2
        
        # Áp dụng dao động vào đùi và cẳng chân
        point.positions = [
            0.0, -0.5 + offset, 0.5 - offset,   # R1
            0.0, -0.5 + offset, 0.5 - offset,   # R2
            0.0, -0.5 + offset, 0.5 - offset,   # R3
            0.0, 0.5 - offset, -0.5 + offset,   # L1
            0.0, 0.5 - offset, -0.5 + offset,   # L2
            0.0, 0.5 - offset, -0.5 + offset    # L3
        ]
        
        # Cập nhật thời gian thực thi cực ngắn (0.1 giây) để tạo độ mượt
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000 
        msg.points = [point]
        
        self.pub.publish(msg)
        self.time_step += 0.2 # Tăng nhịp thời gian

def main():
    rclpy.init()
    node = SpiderDancer()
    print("Mở nhạc lên! Con nhện bắt đầu nhún nhảy tự động...")
    try:
        rclpy.spin(node) # Lệnh này giữ cho node chạy mãi mãi
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

