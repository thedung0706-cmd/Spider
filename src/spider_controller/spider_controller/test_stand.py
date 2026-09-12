import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from inverse_kinematics import HexapodRTMG
import numpy as np

class StandTestNode(Node):
    def __init__(self):
        super().__init__('test_stand_node')
        
        # Mở cổng giao tiếp nã thẳng lực mô-men xoắn xuống tủy sống Gazebo
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/forward_command_controller/commands', 
            10
        )
        self.rtmg = HexapodRTMG()
        
        # Tọa độ 6 chân bám đất (Mô phỏng tư thế đứng thẳng với gầm cao Z = -0.05m)
        target_positions = [
            [ 0.18, -0.08, -0.05], [ 0.0, -0.12, -0.05], [-0.18, -0.08, -0.05], 
            [ 0.18,  0.08, -0.05], [ 0.0,  0.12, -0.05], [-0.18,  0.08, -0.05]
        ]
        
        # Lấy 18 góc quay tham chiếu từ lõi toán học RTMG của bạn
        self.q_ref = self.rtmg.get_18_joint_references(target_positions)
        
        # Nhịp tim bơm lực liên tục 50Hz
        self.timer = self.create_timer(0.02, self.pump_torque)
        self.Kp = 50.0  # Hệ số kéo của bệ đỡ vật lý DecAP

    def pump_torque(self):
        # Giả lập trạng thái ban đầu robot đang thả lỏng (q = 0)
        q_current = np.zeros(18) 
        
        # Thuật toán DecAP: Biến góc lệch thành Lực định hướng
        torque_bias = self.Kp * (self.q_ref - q_current)
        
        # Đóng gói tín hiệu và nã xuống cơ bắp ảo
        msg = Float64MultiArray()
        msg.data = torque_bias.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info('Đang bơm luồng mô-men xoắn 50Hz duy trì tư thế đứng...', once=True)

def main():
    rclpy.init()
    node = StandTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()