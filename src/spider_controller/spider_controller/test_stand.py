import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from inverse_kinematics import HexapodRTMG
import numpy as np

class StandTestNode(Node):
    def __init__(self):
        super().__init__('test_stand_node')
        
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_command_controller/commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.rtmg = HexapodRTMG()
        
        
        self.current_z = -0.05     
        self.target_z = -0.08       
        self.stand_speed = 0.04     
        
        self.Kp = 2.0
        self.Kd = 0.1
        
        self.q_current = np.zeros(18)
        self.q_dot_current = np.zeros(18)
        self.received_states = False
        
        self.joint_order = [
            'joint_coxa_R1', 'joint_femur_R1', 'joint_tibia_R1',
            'joint_coxa_R2', 'joint_femur_R2', 'joint_tibia_R2',
            'joint_coxa_R3', 'joint_femur_R3', 'joint_tibia_R3',
            'joint_coxa_L1', 'joint_femur_L1', 'joint_tibia_L1',
            'joint_coxa_L2', 'joint_femur_L2', 'joint_tibia_L2',
            'joint_coxa_L3', 'joint_femur_L3', 'joint_tibia_L3'
        ]

        # Đã thêm: Mảng bù trừ sai số cơ khí (Calibration Offsets) cho nền tảng phần cứng thật
        self.calibration_offsets = np.zeros(18)
        
        self.timer = self.create_timer(0.01, self.pump_torque)

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_order):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q_current[i] = msg.position[idx]
                self.q_dot_current[i] = msg.velocity[idx]
        self.received_states = True

    def pump_torque(self):
        if not self.received_states:
            return 
            
        # 1. HỆ THỐNG PHANH CAO ĐỘ (Bắt buộc giữ lại)
        if self.current_z > self.target_z:
            self.current_z -= self.stand_speed * 0.01  
            self.current_z = max(self.current_z, self.target_z) # Khóa tọa độ Z
            
        # 2. TẠM TẮT ĐỘNG HỌC NGƯỢC (IK)
        # q_ref_theoretical = self.rtmg.get_18_joint_references(target_positions)
        
        # 3. BƠM TRỰC TIẾP TƯ THẾ ĐỨNG CHUẨN (Khớp hông: 0, Đùi: -0.5, Cẳng: 1.0)
        safe_standing_pose = np.array([
            0.0, -0.5, 1.0,  # Chân R1
            0.0, -0.5, 1.0,  # Chân R2
            0.0, -0.5, 1.0,  # Chân R3
            0.0, -0.5, 1.0,  # Chân L1
            0.0, -0.5, 1.0,  # Chân L2
            0.0, -0.5, 1.0   # Chân L3
        ])
        
        # 4. TÍCH HỢP BÙ TRỪ CƠ KHÍ
        q_ref_calibrated = safe_standing_pose + self.calibration_offsets
        q_dot_ref = np.zeros(18) 
        
        # 5. TÍNH TOÁN LỰC (PD)
        torque_bias = self.Kp * (q_ref_calibrated - self.q_current) + self.Kd * (q_dot_ref - self.q_dot_current)
        
        msg = Float64MultiArray()
        msg.data = torque_bias.tolist()
        self.publisher_.publish(msg)
        if self.current_z <= self.target_z:
            self.get_logger().info('Cỗ máy đã đứng vững và khóa chặt khớp!', once=True)

def main():
    rclpy.init()
    node = StandTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()