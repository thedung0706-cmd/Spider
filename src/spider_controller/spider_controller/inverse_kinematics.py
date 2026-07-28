import rclpy
from rclpy.node import Node
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HexapodIKNode(Node):
    def __init__(self):
        super().__init__('hexapod_ik_node')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.time_counter = 0.0
        self.get_logger().info("Hexapod Inverse Kinematics Node has been started. Sending commands to Gazebo!")

    def calculate_ik(self, x, y, z):
        l_coxa = 0.05
        l_femur = 0.08
        l_tibia = 0.12

        theta_coxa = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2) - l_coxa
        s = z 

        d = math.sqrt(r**2 + s**2)
        if d > (l_femur + l_tibia):
            d = l_femur + l_tibia 

        cos_tibia = (l_femur**2 + l_tibia**2 - d**2) / (2 * l_femur * l_tibia)
        cos_tibia = max(-1.0, min(1.0, cos_tibia)) 
        theta_tibia = math.acos(cos_tibia) - math.pi 

        alpha = math.atan2(s, r)
        cos_beta = (l_femur**2 + d**2 - l_tibia**2) / (2 * l_femur * d)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        theta_femur = alpha + beta

        return theta_coxa, theta_femur, theta_tibia

    def timer_callback(self):
        self.time_counter += 0.05
        
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=0, nanosec=50000000) 
        
        legs = ['R1', 'R2', 'R3', 'L1', 'L2', 'L3']
        
        joint_names = []
        joint_positions = []

        for i, leg in enumerate(legs):
            phase = math.pi if i % 2 == 0 else 0.0
            
            x_target = 0.18 + 0.02 * math.sin(self.time_counter * 3.0 + phase)
            y_target = -0.08 if 'R' in leg else 0.08
            z_target = -0.05 + 0.02 * max(0, math.cos(self.time_counter * 3.0 + phase)) 

            c_angle, f_angle, t_angle = self.calculate_ik(x_target, y_target, z_target)
            if 'L' in leg:
                f_angle = -f_angle
                t_angle = -t_angle

            joint_names.extend([f'joint_coxa_{leg}', f'joint_femur_{leg}', f'joint_tibia_{leg}'])
            joint_positions.extend([c_angle, f_angle, t_angle])

        msg.joint_names = joint_names
        point.positions = joint_positions
        msg.points = [point]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()