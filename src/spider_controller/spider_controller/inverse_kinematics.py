import math
import numpy as np

class HexapodRTMG:
    def __init__(self):
        # Thông số cấu hình cơ khí của robot
        self.l_coxa = 0.05
        self.l_femur = 0.08
        self.l_tibia = 0.12
        self.clearance_h = 0.07  # Khoảng sáng gầm an toàn khi nhấc chân

    def calculate_ik(self, x, y, z):
        # Lõi Động học ngược (IK) cũ được giữ nguyên vẹn
        theta_coxa = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - self.l_coxa
        s = z 

        d = math.sqrt(r**2 + s**2)
        if d > (self.l_femur + self.l_tibia):
            d = self.l_femur + self.l_tibia 

        cos_tibia = (self.l_femur**2 + self.l_tibia**2 - d**2) / (2 * self.l_femur * self.l_tibia)
        cos_tibia = max(-1.0, min(1.0, cos_tibia)) 
        theta_tibia = math.acos(cos_tibia) - math.pi 

        alpha = math.atan2(s, r)
        cos_beta = (self.l_femur**2 + d**2 - self.l_tibia**2) / (2 * self.l_femur * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        theta_femur = alpha + beta

        return theta_coxa, theta_femur, theta_tibia

    def bezier_swing_trajectory(self, t, P0, P3):
        # Quỹ đạo Pha vung (Swing Phase) bằng đường cong Bézier bậc 3
        P1 = np.array([P0[0], P0[1], P0[2] + self.clearance_h])
        P2 = np.array([P3[0], P3[1], P3[2] + self.clearance_h])
        
        # Phương trình nội suy Bézier
        pos = ((1 - t)**3) * P0 + 3 * ((1 - t)**2) * t * P1 + 3 * (1 - t) * (t**2) * P2 + (t**3) * P3
        return pos

    def stance_trajectory(self, t, P_start, v_lin, w_z, leg_pos):
        # Quỹ đạo Pha chống (Stance Phase) ngược hướng tịnh tiến
        delta_p_rot = np.array([-leg_pos[1] * w_z, leg_pos[0] * w_z, 0.0])
        pos = P_start + (-np.array([v_lin[0], v_lin[1], 0]) + delta_p_rot) * t
        return pos

    def get_18_joint_references(self, target_positions):
        # Hàm cấp 18 góc quay tham chiếu cho DecAP và mạng PPO
        joint_refs = []
        for pos in target_positions:
            c, f, t_angle = self.calculate_ik(pos[0], pos[1], pos[2])
            joint_refs.extend([c, f, t_angle])
        return np.array(joint_refs, dtype=np.float32)