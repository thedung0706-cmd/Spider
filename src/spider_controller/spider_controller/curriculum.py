import numpy as np

class HexapodCurriculumLearning:
    def __init__(self):
        self.Kp = 50.0
        self.Kd = 0.5
        self.gamma = 0.99      
        self.k_decay = 100.0  
        self.w_lin_vel = 1.0     
        self.w_ang_vel = 1.0     
        self.w_imit_ori = 1.5    
        self.w_imit_height = 1.5 
        self.w_imit_pos = 1.5    

    def compute_decap_torque(self, q_ref, q_dot_ref, q_current, q_dot_current, step_t, agent_action):
        beta_t = self.Kp * (q_ref - q_current) + self.Kd * (q_dot_ref - q_dot_current)
        
        decay_factor = self.gamma ** (step_t / self.k_decay)
        
        tau_t = agent_action + (decay_factor * beta_t)
        
        return tau_t

    def compute_stage_1_reward(self, v_actual, v_cmd, omega_actual, omega_cmd, 
                               theta_actual, theta_ref, z_actual, z_ref, p_actual, p_ref):
        
        
        r_lin_vel = np.exp(-np.sum(np.square(v_actual - v_cmd)) / 0.02)
        r_ang_vel = np.exp(-np.sum(np.square(omega_actual - omega_cmd)) / 0.04)
        
        
        r_imit_ori = np.exp(-np.sum(np.square(theta_actual - theta_ref)) / 0.1)
        r_imit_height = np.exp(-np.square(z_actual - z_ref) / 0.025)
        r_imit_pos = np.exp(-np.sum(np.square(p_actual - p_ref)) / 0.025)
        
        
        total_reward = (self.w_lin_vel * r_lin_vel + 
                        self.w_ang_vel * r_ang_vel + 
                        self.w_imit_ori * r_imit_ori + 
                        self.w_imit_height * r_imit_height + 
                        self.w_imit_pos * r_imit_pos)
        
        return total_reward