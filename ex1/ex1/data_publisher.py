# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import random

class DataPublisher(Node):  #węzeł publikujący losowe wiadomości (topic: cmd_vel)

    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(random.randint(-5,5))/5
        msg.angular.z = float(random.randint(-5, 5))/5     
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    data_publisher = DataPublisher()

    rclpy.spin(data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
