import rclpy
from rclpy.node import Node

class SpiderCore(Node):
    def __init__(self):
        super().__init__('spider_core_node')
        self.get_logger().info('Xin chao! He thong Dong hoc Robot Nhen da san sang nhan lenh!')

def main(args=None):
    rclpy.init(args=args)
    node = SpiderCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()