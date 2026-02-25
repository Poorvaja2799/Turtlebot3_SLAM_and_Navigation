"""
Lab 3 - Galactic Chase Object
Author: Griffin Martin & Poorvaja Veera Balaji Kumar
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class GetObjectRange(Node):

    def __init__(self):

        # initialization things
        super().__init__('minimal_publisher')
        self.declare_parameter('publish_frequency', 20)
        self.declare_parameter('pixel_width', 320)
        self.declare_parameter('camera_fov', 62.2)  # Turtlebot3 camera FOV in degrees

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        # set up subscriber
        self.point_msg = None
        self.scan_data = None
        self.coords_subscription = self.create_subscription(Point, '/chase_object/object_coords', self.coords_subscription_callback, qos_profile)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_subscription_callback, qos_profile)

        # set up publisher
        self.publisher_ = self.create_publisher(Vector3, '/chase_object/object_vector', qos_profile)
        self.timer = self.create_timer(1/self.get_parameter('publish_frequency').value, self.publisher_callback)

    def publisher_callback(self):
        msg = Vector3()
        
        if self.point_msg is None or self.scan_data is None:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0
        else:
            # Get pixel x-coordinate and calculate angle from center
            pixel_x = self.point_msg.x
            pixel_width = self.get_parameter('pixel_width').value
            camera_fov = self.get_parameter('camera_fov').value
            
            # Angle in degrees from center of camera
            pixel_offset = pixel_x - (pixel_width / 2)
            angle_offset = (pixel_offset / (pixel_width / 2)) * (camera_fov / 2)
            
            # Convert to radians and adjust to LIDAR frame
            angle_rad = -np.radians(angle_offset)
            
            # Map angle to LIDAR array index
            num_readings = len(self.scan_data['ranges'])
            angle_min = self.scan_data['angle_min']
            angle_increment = self.scan_data['angle_increment']
            
            # Find the index corresponding to our angle
            lidar_index = int((angle_rad - angle_min) / angle_increment)
            lidar_index = np.clip(lidar_index, 0, num_readings - 1)
            
            # Get distance from LIDAR at that angle
            window_size = 5
            start_idx = max(0, lidar_index - window_size // 2)
            end_idx = min(num_readings, lidar_index + window_size // 2 + 1)
            distance = np.mean(self.scan_data['ranges'][start_idx:end_idx])
            
            msg.x = float(distance * np.cos(angle_rad))  # x position in meters
            msg.y = float(distance * np.sin(angle_rad))  # y position in meters
            msg.z = 0.0
            
            # polar coordinate format:
            # msg.x = float(angle_rad)  # Angular position in radians
            # msg.y = float(distance)   # Distance in meters
            # msg.z = 0.0
            
        self.publisher_.publish(msg)

    def coords_subscription_callback(self, msg):
        if msg.z != 0:
            self.point_msg = None
        else:
            self.point_msg = msg

    def scan_subscription_callback(self, msg):
        # Store filtered LIDAR data
        ranges = np.array(msg.ranges)
        # Filter out invalid readings
        ranges[ranges > msg.range_max] = msg.range_max
        ranges[ranges < msg.range_min] = msg.range_min
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        
        self.scan_data = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }



def main(args=None):
    rclpy.init(args=args)

    node = GetObjectRange()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
