import numpy as np
import math

class HexapodRTMG:
    def __init__(self):
        # Kích thước cơ khí IK đã có từ trước...
        self.l_bs = 0.04
        self.l_se = 0.08
        self.l_ef = 0.12
        
        # THÔNG SỐ QUỸ ĐẠO
        self.swing_height = 0.07  # Khoảng sáng gầm khi nhấc chân là 7cm

    def compute_stance_trajectory(self, p_start, v_lin, omega_z, t_stance):
        """
        Toán học Pha Chống (Bám đất)
        t_stance: Thời gian chuẩn hóa chạy từ -0.5 đến 0.5 để căn giữa chu kỳ bước.
        """
        x_i, y_i = p_start[0], p_start[1]
        
        # 1. Tính vận tốc dịch chuyển do góc xoay yaw sinh ra
        delta_p_rot = np.array([-y_i * omega_z, x_i * omega_z, 0.0])
        v_lin_3d = np.array([v_lin[0], v_lin[1], 0.0])
        
        # 2. Phương trình dịch chuyển chân lùi về sau để đẩy thân tới trước
        p_stance = p_start + (-v_lin_3d + delta_p_rot) * t_stance
        return p_stance

    def compute_swing_trajectory(self, P0, P3, t_swing):
        """
        Toán học Pha Vung (Nhấc chân)
        t_swing: Thời gian chuẩn hóa chạy từ 0 đến 1.
        """
        # 1. Thiết lập 2 điểm điều khiển trên không, cộng thêm 0.07m chiều cao[cite: 1]
        P1 = P0 + np.array([0.0, 0.0, self.swing_height]) 
        P2 = P3 + np.array([0.0, 0.0, self.swing_height]) 
        
        # 2. Áp dụng phương trình đường cong Bézier bậc 3 tạo độ mượt[cite: 1]
        term0 = (1 - t_swing)**3 * P0
        term1 = 3 * (1 - t_swing)**2 * t_swing * P1
        term2 = 3 * (1 - t_swing) * t_swing**2 * P2
        term3 = t_swing**3 * P3
        
        return term0 + term1 + term2 + term3

    def transform_body_to_leg(self, p_body, t_vec, R_matrix):
        """
        Chuyển đổi hệ tọa độ từ tâm robot ra gốc của từng chiếc chân[cite: 1]
        """
        # Áp dụng công thức p_leg = R^T * (p_body - t)[cite: 1]
        p_leg = np.dot(R_matrix.T, (p_body - t_vec))
        return p_leg
    
    def compute_single_leg_ik(self, target_pos):
        x, y, z = target_pos
        
        # 1. Tính góc Hông (Coxa)
        theta_1 = math.atan2(y, x)
        
        # 2. Tính toán mặt phẳng 2D của Đùi và Cẳng chân
        L = math.sqrt(x**2 + y**2)
        
        # CHỐT AN TOÀN: Ngăn robot cố vươn chân dài hơn thực tế gây lỗi NaN
        max_reach = self.l_bs + self.l_se + self.l_ef - 0.001
        if L > max_reach:
            L = max_reach 
            
        HF = math.sqrt((L - self.l_bs)**2 + z**2)
        
        # CHỐT AN TOÀN 2: Khóa biến số trước khi đưa vào hàm arccos
        # Ngăn chặn lỗi chia cho 0 hoặc giá trị ngoài khoảng [-1, 1]
        val_a2 = (self.l_ef**2 - self.l_se**2 - HF**2) / (-2 * self.l_se * HF)
        val_a2 = np.clip(val_a2, -1.0, 1.0)
        
        val_theta3 = (HF**2 - self.l_ef**2 - self.l_se**2) / (-2 * self.l_se * self.l_ef)
        val_theta3 = np.clip(val_theta3, -1.0, 1.0)

        # 3. Giải phương trình góc Đùi và Cẳng chân
        A1 = math.atan2(L - self.l_bs, abs(z)) # abs(z) vì hệ Gazebo quy định z âm đâm xuống đất
        A2 = math.acos(val_a2)
        
        theta_2 = (math.pi / 2) - (A1 + A2)
        theta_3 = (math.pi / 2) - math.acos(val_theta3)
        
        return [theta_1, theta_2, theta_3]

    def get_18_joint_references(self, target_positions_list):
        """
        Nhận mảng 6 tọa độ [x, y, z] của 6 chân, trả về mảng 18 góc xoay liên tục
        Thứ tự: R1, R2, R3, L1, L2, L3
        """
        q_ref = []
        for pos in target_positions_list:
            angles = self.compute_single_leg_ik(pos)
            q_ref.extend(angles)
        return np.array(q_ref)