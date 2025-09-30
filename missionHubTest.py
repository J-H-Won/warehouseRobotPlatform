# hub_cmd.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main():
    rclpy.init(); n = Hub(); rclpy.spin(n); rclpy.shutdown()
if __name__ == '__main__': main()
