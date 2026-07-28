"""
Lab 4 - Galactic Navigate to Goal
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class GetObjectRange(Node):

    def __init__(self):
        super().__init__('get_object_range')
        self.declare_parameter('publish_frequency', 20)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # set up subscriber
        self.scan_data = None
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_subscription_callback, qos_profile)

        # set up publisher
        self.publisher_ = self.create_publisher(Vector3, '/obj_pos', qos_profile)
        self.timer = self.create_timer(1/self.get_parameter('publish_frequency').value, self.publisher_callback)

    def publisher_callback(self):
        msg = Vector3()
        
        if self.scan_data is None:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
        else:
            ranges = self.scan_data['ranges']
            finite_indices = np.where(np.isfinite(ranges))[0]
            if finite_indices.size == 0:
                self.publisher_.publish(msg)
                return

            outlier_ratio_threshold = 0.3
            candidate_ranges = np.copy(ranges)
            selected_index = None

            while np.isfinite(np.min(candidate_ranges)):
                nearest_index = int(np.argmin(candidate_ranges))
                nearest_distance = float(candidate_ranges[nearest_index])

                left_distance = candidate_ranges[nearest_index - 1] if nearest_index - 1 >= 0 else np.inf
                right_distance = candidate_ranges[nearest_index + 1] if nearest_index + 1 < len(candidate_ranges) else np.inf

                left_close = np.isfinite(left_distance) and (
                    abs(nearest_distance - left_distance) / max(left_distance, 1e-6) <= outlier_ratio_threshold
                )
                right_close = np.isfinite(right_distance) and (
                    abs(nearest_distance - right_distance) / max(right_distance, 1e-6) <= outlier_ratio_threshold
                )

                if left_close or right_close:
                    selected_index = nearest_index
                    break

                candidate_ranges[nearest_index] = np.inf

            if selected_index is None:
                self.publisher_.publish(msg)
                return

            selected_distance = float(ranges[selected_index])

            if np.isfinite(selected_distance):
                nearest_angle = self.scan_data['angle_min'] + (
                    selected_index * self.scan_data['angle_increment']
                )
                msg.x = float(selected_distance * np.cos(nearest_angle))
                msg.y = float(selected_distance * np.sin(nearest_angle))
                msg.z = 0.0
            else:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
            
        self.publisher_.publish(msg)

    def scan_subscription_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[~np.isfinite(ranges)] = np.inf
        ranges[ranges > msg.range_max] = np.inf
        ranges[ranges < msg.range_min] = np.inf
        
        self.scan_data = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment
        }



def main(args=None):
    rclpy.init(args=args)

    node = GetObjectRange()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
