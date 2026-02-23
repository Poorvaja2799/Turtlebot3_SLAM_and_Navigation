"""
Lab 3 - Chase Object
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from dataclasses import dataclass
import math


class RotateRobot(Node):

    def __init__(self):

        # initialization things
        super().__init__('minimal_publisher')
        self.declare_parameter('publish_frequency', 40) # in Hz
        self.declare_parameter('distance_setpoint', 1.0) # in meters?
        # rotational PID constants
        self.declare_parameter('kp_r', hmmm)
        self.declare_parameter('ki_r', uhhhh)
        self.declare_parameter('kd_r', sdkfj)
        # translational PID constants
        self.declare_parameter('kp_t', djfj)
        self.declare_parameter('ki_t', fjf)
        self.declare_parameter('kd_t', fjfj)

        # store parameters for speed
        kp_r = self.get_parameter('kp_r').value
        ki_r = self.get_parameter('ki_r').value
        kd_r = self.get_parameter('kd_r').value
        kp_t = self.get_parameter('kp_t').value
        ki_t = self.get_parameter('ki_t').value
        kd_t = self.get_parameter('kd_t').value
        publish_period = 1/self.get_parameter('publish_frequency').value
        self.setpoint_t = self.get_parameter('distance_setpoint').value

        # set up PID controllers
        self.rotational_controller = PID_controller(kp_r, ki_r, kd_r, publish_period)
        self.translational_controller = PID_controller(kp_t, ki_t, kd_t, publish_period)

        # set up QOS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # set up publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.timer = self.create_timer(publish_period, self.publisher_callback)
        
        # set up subscriber
        self.x = None
        self.y = None
        self.subscription_ = self.create_subscription(Vector3, '/chase_object/object_vector', self.subscription_callback, qos_profile)

    def publisher_callback(self):
        msg = Twist()
        
        if self.x is None or self.y is None:
            msg.angular.z = 0.0
            msg.linear.x = 0.0
        else:
            angle_err = math.atan2(self.y, self.x) # bounded from -pi to pi
            dist = math.sqrt(pow(self.x, 2) + pow(self.y, 2))
            dist_err = self.setpoint_t - dist
            msg.linear.x = self.translational_controller.get_effort(dist_err)
            msg.angular.z = self.rotational_controller.get_effort(angle_err)
            
        self.publisher_.publish(msg)

    def subscription_callback(self, msg):
        if msg.z != 0:
            self.x, self.y = None
        else:
            self.x = msg.x
            self.y = msg.y

@dataclass
class PID_controller:
    kp: float
    ki: float
    kd: float
    period: float

    def __post_init__(self):
        self.error_sum = 0.0
        self.prev_error = 0.0

    def get_effort(self, error):
        self.error_sum += error
        output = self.kp*error + self.ki*self.error_sum*self.period + self.kd*(error-self.prev_error)/self.period
        self.prev_error = error
        return output

def main(args=None):
    rclpy.init(args=args)

    node = RotateRobot()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
