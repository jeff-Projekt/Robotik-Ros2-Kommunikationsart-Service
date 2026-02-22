#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from math import radians

class LaserSubscriberNode(Node):
    def __init__(self):
        super().__init__('laser_data_subscriber')
        self.laser_data=[]
        self.laser_data_subscriber=self.create_subscription(LaserScan,"scan",self.callback_laser_data,10)
        self.get_logger().info("Node laser_data_subscriber gestarted.")

    def callback_laser_data(self,msg:LaserScan):
        self.laser_data=msg.ranges
        min_values=min(self.laser_data)
        if min_values < 1.0:
                self.get_logger().warn(f"Hindernis erkannin weniger als 1m Entfernung! Abstand = {min_values:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LaserSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
   main()