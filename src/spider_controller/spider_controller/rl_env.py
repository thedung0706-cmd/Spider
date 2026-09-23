import numpy as np
import torch
import torch.nn as nn

class HexapodRLEnv:
    def __init__(self):
        # Không gian hành động 18 chiều (Torque commands)[cite: 3]
        self.num_actions = 18
        # Không gian quan sát 65 chiều[cite: 3]
        self.num_observations = 65

    def get_observation(self, state_data):
        """
        Tổng hợp dữ liệu cảm biến thực tế tại mỗi bước thời gian.
        """
        obs = np.concatenate([
            state_data['base_angular_velocity'],  # 3D: Vận tốc góc thân (từ IMU)[cite: 3]
            state_data['projected_gravity'],      # 3D: Vector trọng lực chiếu (từ quaternion)[cite: 3]
            state_data['user_commands'],          # 3D: Lệnh điều hướng (v_x, v_y, omega_z)[cite: 3]
            state_data['joint_positions'],        # 18D: Góc quay hiện tại của 18 khớp[cite: 3]
            state_data['joint_velocities'],       # 18D: Vận tốc góc của 18 khớp[cite: 3]
            state_data['previous_actions']        # 18D: Lệnh mô-men xoắn từ bước trước đó[cite: 3]
        ])
        return obs

class PPOActorCritic(nn.Module):
    def __init__(self):
        super(PPOActorCritic, self).__init__()
        
        # MẠNG CHÍNH SÁCH (ACTOR): Quyết định hành động mô-men xoắn[cite: 3]
        self.actor = nn.Sequential(
            nn.Linear(65, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 18) # Đầu ra 18 chiều
        )
        
        # MẠNG GIÁ TRỊ (CRITIC): Đánh giá trạng thái[cite: 3]
        self.critic = nn.Sequential(
            nn.Linear(65, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1)  # Đầu ra 1 chiều (Scalar value estimate)[cite: 3]
        )

    def forward(self, observation):
        action = self.actor(observation)
        value = self.critic(observation)
        return action, value