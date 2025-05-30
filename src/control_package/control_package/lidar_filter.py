#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String
from robot_interfaces.msg import Position
from std_msgs.msg import Bool
import numpy as np
import math
import json
import time


class LidarFilter(Node):
    def __init__(self, min_distance, max_distance, emergency_distance):
        """
        Initialize the LidarFilter node.
        
        Args:
            min_distance: Minimum valid distance to consider
            max_distance: Maximum valid distance to consider
            emergency_distance: Threshold for emergency stop
        """
        super().__init__("lidar_filter")
        
        # Robot's current position and orientation
        self.x_ = 0.0  # Current robot x position
        self.y_ = 0.0  # Current robot y position
        self.r_ = 0.0  # Current robot orientation angle
        
        # Store distance thresholds
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.emergency_distance = emergency_distance
        
        # Create unit conversion factor (assuming config uses mm and code uses m)
        self.unit_factor = 0.001  # Convert mm to m if needed
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            10
        )
        
        self.position_subscriber = self.create_subscription(
            Position,
            "robot_position",
            self.robot_position_callback,
            10
        )
        
        # Publishers
        self.filter_scan_publisher = self.create_publisher(
            LaserScan, 
            "filter_scan_topic", 
            10
        )
        
        self.emergency_stop_publisher = self.create_publisher(
            Bool, 
            "emergency_stop_topic", 
            10
        )
        
        self.get_logger().info('🚀 LidarFilter node has been started')
        self.get_logger().info(f'Parameters: min_distance={min_distance}m, '
                              f'max_distance={max_distance}m, '
                              f'emergency_distance={emergency_distance}m')

    def robot_position_callback(self, msg):
        """Update robot position data when new position is received"""
        self.x_ = msg.x
        self.y_ = msg.y
        self.r_ = msg.r

    def scan_callback(self, msg):
        """Process incoming LiDAR scan data"""
        ranges_list = msg.ranges

        # Filter ranges based on min/max distances
        filtered_ranges = [
            max(min(r, self.max_distance), self.min_distance) 
            for r in ranges_list
        ]

        # Group and average by angle
        angle_dict = self.process_ranges_by_angle(filtered_ranges)
        
        # Create angle-based ranges list
        angle_ranges = self.create_angle_ranges(angle_dict, len(ranges_list))
        
        # Check for obstacles requiring emergency stop
        self.check_emergency_stop(angle_ranges)
        
        # Publish filtered scan
        #self.publish_filtered_scan(msg, angle_ranges)

    def process_ranges_by_angle(self, filtered_ranges):
        """Group and average range readings by angle"""
        angle_dict = {}
        
        for index, distance in enumerate(filtered_ranges):
            if self.min_distance < distance < self.max_distance:
                index_offset = (index + 900) % 1800  # 900 is the offset
                angle = int(360 - index_offset / 5)  # 1800tic/360°=5
                
                if angle not in angle_dict:
                    angle_dict[angle] = {'count': 1, 'total_distance': distance}
                else:
                    angle_dict[angle]['count'] += 1
                    angle_dict[angle]['total_distance'] += distance
                
                # Calculate the average distance for the current angle
                angle_dict[angle]['average_distance'] = round(
                    angle_dict[angle]['total_distance'] / angle_dict[angle]['count'], 
                    2
                )
                
        return angle_dict

    def create_angle_ranges(self, angle_dict, length):
        """Create range list with average distances from angle dictionary"""
        angle_ranges = [0] * length
        
        for index in range(length):
            index_offset = (index + 900) % 1800
            angle = int(360 - index_offset / 5)
            
            if angle in angle_dict:
                angle_ranges[index] = angle_dict[angle]['average_distance']
                
        return angle_ranges

    def check_emergency_stop(self, angle_ranges):
        """Check if emergency stop is required based on detected obstacles"""
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = False
        
        for index, distance in enumerate(angle_ranges):            
            if self.max_distance > distance > self.min_distance:
                index_offset = (index + 900) % 1800
                angle = int(2*360 - index_offset / 5)
                if angle > 180:
                    angle = -(360 - angle)

                # Convert polar coordinates to Cartesian coordinates
                angle_rad = np.radians(angle)
                dist_x = distance * np.sin(angle_rad)  # in m
                dist_y = distance * np.cos(angle_rad)  # in m
                
                x_obstacle, y_obstacle = self.calculate_obstacle_position(distance, angle)
                
                # Check if obstacle is in emergency zone
                # Assuming x_obstacle and y_obstacle are in mm
                
                #self.get_logger().info(f"👮👮👮 Obstacle ! self.x_:={self.x_}m, self.y_:={self.y_}m, angle:={angle}, real_angle:={self.r_ - angle}, dist_x:={round(dist_x,2)}m, dist_y={round(dist_y,2)}m; Ostacle Position ({round(x_obstacle)}, {round(y_obstacle)})")
                if (self.min_distance < dist_y < self.emergency_distance and
                        -0.35 < dist_x < 0.35 and
                        -1300 < x_obstacle < 1300 and
                        200 < y_obstacle < 1800):
                    
                    self.print_robot_infos()
                    self.get_logger().info(
                        f"👮 Obstacle! dist_x={round(dist_x,2)}m, "
                        f"dist_y={round(dist_y,2)}m; "
                        f"Obstacle Position ({round(x_obstacle)}, {round(y_obstacle)})"
                    )
                
                    emergency_stop_msg.data = True
                    self.emergency_stop_publisher.publish(emergency_stop_msg)
                    return  # Exit after first emergency detection to avoid flooding
        
        # No emergency detected
        self.emergency_stop_publisher.publish(emergency_stop_msg)
    
    def print_robot_infos(self):
        """Print current robot position and orientation"""
        self.get_logger().info(
            f"\033[95m[Robot Infos] x:{self.x_}, y:{self.y_}, r:{self.r_}\033[0m")

    def calculate_obstacle_position(self, distance, angle):
        """
        Calculate absolute position of obstacle in world coordinates
        
        Args:
            distance: Distance to obstacle in meters
            angle: Angle to obstacle in degrees
            
        Returns:
            Tuple of (x, y) coordinates in mm
        """
        # Convert observer's orientation from degrees to radians
        ar_rad = math.radians(self.r_)
        
        # Calculate the angle from the observer to the obstacle in radians
        angle_rad = math.radians(angle)
        total_angle = ar_rad - angle_rad
        
        # Calculate the Cartesian coordinates of the obstacle (convert distance to mm)
        distance_mm = distance * 1000  # Convert m to mm
        ox = self.x_ - distance_mm * math.sin(total_angle)
        oy = self.y_ + distance_mm * math.cos(total_angle)
        
        return round(ox), round(oy)
    
    def publish_filtered_scan(self, original_msg, angle_ranges):
        """Publish the filtered scan message"""
        filtered_scan = LaserScan(
            header=original_msg.header,
            angle_min=original_msg.angle_min,
            angle_max=original_msg.angle_max,
            angle_increment=original_msg.angle_increment,
            time_increment=original_msg.time_increment,
            scan_time=original_msg.scan_time,
            range_min=self.min_distance,
            range_max=self.max_distance,
            ranges=angle_ranges,
            intensities=original_msg.intensities
        )
        self.filter_scan_publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        with open('/home/edog/ros2_ws/src/control_package/resource/robot_config.json') as file:
            config = json.load(file)
            
        # Create the node with correct parameter order
        node = LidarFilter(
            min_distance=config['robot']['min_distance'],
            max_distance=config['robot']['max_distance'],
            emergency_distance=config['robot']['emergency_distance']
        )
        
        rclpy.spin(node)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()