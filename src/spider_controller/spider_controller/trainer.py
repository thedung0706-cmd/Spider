import torch
import numpy as np
# Import 3 module bạn vừa viết
# from rtmg import HexapodRTMG
# from curriculum import HexapodCurriculumLearning
# from rl_env import HexapodRLEnv, PPOActorCritic

def stage_1_training_loop(env, ppo_agent, curriculum, rtmg, max_steps=2000):
    """
    Vòng lặp huấn luyện Giai đoạn 1 trên mặt phẳng (Flat Terrain) với 4096 agents[cite: 2].
    """
    cycle_length = 100  # Chu kỳ của 1 bước đi hoàn chỉnh
    
    for step in range(max_steps):
        # 1. BỘ ĐẾM NHỊP (GAIT TIMER)
        # Chia 6 chân làm 2 nhóm luân phiên Chống/Vung sau mỗi nửa chu kỳ
        phase = (step % cycle_length) / cycle_length
        if phase < 0.5:
            t_swing = phase * 2.0
            t_stance = (phase * 2.0) - 0.5
        else:
            t_swing = (phase - 0.5) * 2.0
            t_stance = ((phase - 0.5) * 2.0) - 0.5

        # 2. RTMG TẠO QUỸ ĐẠO MẪU
        user_cmd = env.get_user_commands()
        q_ref, q_dot_ref = rtmg.get_18_joint_references(user_cmd, t_swing, t_stance)
        
        # 3. AI QUAN SÁT VÀ SUY LUẬN
        # Đọc 65 thông số giác quan (IMU, góc khớp, hành động cũ)[cite: 2]
        obs_65d = env.get_observation()
        obs_tensor = torch.FloatTensor(obs_65d)
        
        # Mạng MLP xuất ra 18 lệnh mô-men xoắn (Torque)[cite: 2]
        agent_action = ppo_agent(obs_tensor).detach().numpy()
        
        # 4. TRỌNG TÀI BƠM LỰC DECAP
        # Bộ PD mượn quỹ đạo của RTMG tạo lực đỡ, giảm dần theo thời gian[cite: 2]
        final_torque = curriculum.compute_decap_torque(
            q_ref, q_dot_ref, env.current_q, env.current_q_dot, step, agent_action
        )
        
        # 5. THỰC THI VÀ CHẤM ĐIỂM
        # Truyền lực final_torque xuống 18 động cơ trong NVIDIA Isaac Gym
        env.step(final_torque)
        
        # Tính điểm thưởng bắt chước tư thế, chiều cao và vị trí[cite: 2]
        reward = curriculum.compute_stage_1_reward(...)
        
        # Cập nhật tham số mạng PPO
        ppo_agent.update(obs_tensor, agent_action, reward)