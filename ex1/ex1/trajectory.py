import rclpy
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math as m
from nav_msgs.msg import Odometry


class ControlSys(Node):

    def __init__(self): #ruch po trajektorii
        self.start = 0
        self.x = 0
        self.y = 0
        self.distThresh = 0.25  # dokładność
        self.v = 0  # predkosc liniowa
        self.w = 0  # predkosc katowa
        self.d_theta = 1  # kat do celu
        self.angles = None  # kąty
        self.trajectory = [[-10, 10], [2, 2], [10, 10], [0, 0]]  # trajektoria
        self.point_idx = 0
        self.goal = self.trajectory[self.point_idx]
        self.total_distance = 0
        self.in_point = 1
        self.distance = 0  # odleglosc do celu
        super().__init__('from_a_to_b')
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # obliczanie dystansu
        self.distance = m.sqrt(
            (self.x-self.goal[0])**2 + (self.y-self.goal[1])**2)
        if (self.start == 0):  # nie porusza się lioniowo (obrót)
            self.total_distance = self.distance
            if (self.in_point == 0):  # dobór kolejnych pkt
                self.point_idx += 1
                if (self.point_idx > len(self.trajectory)-1):  # koniec trajektorii
                    self.get_logger().info("Koniec Trasy")
                    self.start = 3
                    self.v = 0
                    self.w = 0
                else:
                    # wyliczenie kątów obrotu
                    self.goal = self.trajectory[self.point_idx]
                    desireYaw = m.atan2(
                        self.goal[1]-self.y, self.goal[0]-self.x)
                    self.d_theta = desireYaw-self.angles[2]
                    if self.d_theta > m.pi:
                        self.d_theta -= 2 * m.pi
                    elif self.d_theta < - m.pi:
                        self.d_theta += 2 * m.pi
            self.in_point = 1
        self.w = min(0.5, max(-0.5, self.d_theta))  # nadanie prędkości kątowej

        if (abs(self.d_theta) < 0.01):  # warunek rozoczęcia ruchu do przodu
            self.start = 1
            self.in_point = 0
        if (self.start == 1):  # ruch do przodu
            if (self.distance > self.distThresh):
                # zadanie p©edkości liniowej
                self.v = min(4, self.distance / 3,
                             (self.total_distance - self.distance) + 0.2)
            else:
                self.v = 0
                self.start = 0
        else:
            self.v = 0
        self.publish()  # wysłanie wiadomości

    def listener_callback(self, msg):  # odbiór danych i przeliczenie kątów
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.angles = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])
        desireYaw = m.atan2(self.goal[1]-self.y, self.goal[0]-self.x)
        self.d_theta = desireYaw-self.angles[2]
        if self.d_theta > m.pi:
            self.d_theta -= 2 * m.pi
        elif self.d_theta < - m.pi:
            self.d_theta += 2 * m.pi

    def publish(self):  #wysłanie prędkości
        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ControlSys()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
