import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

def main():
    rclpy.init()
    node = Node('spider_stand_node')
    
    # Tạo cầu nối bắn lệnh vào hệ thống thần kinh của nhện
    pub = node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
    
    msg = JointTrajectory()
    # Khai báo đủ 18 khớp (Hông, Đùi, Cẳng chân) của 6 chân
    msg.joint_names = [
        'joint_coxa_R1', 'joint_femur_R1', 'joint_tibia_R1',
        'joint_coxa_R2', 'joint_femur_R2', 'joint_tibia_R2',
        'joint_coxa_R3', 'joint_femur_R3', 'joint_tibia_R3',
        'joint_coxa_L1', 'joint_femur_L1', 'joint_tibia_L1',
        'joint_coxa_L2', 'joint_femur_L2', 'joint_tibia_L2',
        'joint_coxa_L3', 'joint_femur_L3', 'joint_tibia_L3'
    ]
    
    point = JointTrajectoryPoint()
    
    # Thiết lập góc quay (radian) cho các khớp. 
    # Hông (0.0), Đùi gập xuống (-0.5), Cẳng chân gập lên (0.5)
    point.positions = [
        0.0, -0.5, 0.5,   # R1
        0.0, -0.5, 0.5,   # R2
        0.0, -0.5, 0.5,   # R3
        0.0, 0.5, -0.5,   # L1 (Bên trái ngược trục nên đảo dấu)
        0.0, 0.5, -0.5,   # L2
        0.0, 0.5, -0.5    # L3
    ]
    
    # Thời gian hoàn thành chuyển động: 2 giây
    point.time_from_start.sec = 2
    point.time_from_start.nanosec = 0
    msg.points = [point]
    
    # Chờ 1 giây để ROS 2 kịp kết nối đường truyền
    node.get_logger().info('Đang đồng bộ tín hiệu...')
    time.sleep(1.0)
    
    # Kích hoạt!
    node.get_logger().info('Gửi lệnh: ĐỨNG DẬY!')
    pub.publish(msg)
    time.sleep(1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
