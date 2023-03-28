#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


from nav_msgs.msg import Odometry

class OdomSubscriber(Node): # węzeł odbierający wiadomości (topic: odom)

    def __init__(self):
        self.x = None
        self.y = None
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(Odometry, 'odom', self.listener_callback, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info((f'x pos: {self.x}'))
        self.get_logger().info((f'y pos: {self.y}'))

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)

    node = OdomSubscriber()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()