#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import random

def lidar_publisher():
    rospy.init_node('lidar_publisher', anonymous=True)
    lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Generate mock LiDAR data
        ranges = [random.uniform(0.1, 5.0) for _ in range(360)]
        intensities = [random.uniform(0.0, 1.0) for _ in range(360)]

        lidar_msg = LaserScan()
        lidar_msg.header.stamp = rospy.Time.now()
        lidar_msg.header.frame_id = "lidar_frame"
        lidar_msg.angle_min = -3.14159274101
        lidar_msg.angle_max = 3.14159274101
        lidar_msg.angle_increment = 0.0174532923847
        lidar_msg.time_increment = 0.0
        lidar_msg.range_min = 0.0
        lidar_msg.range_max = 100.0
        lidar_msg.ranges = ranges
        lidar_msg.intensities = intensities

        lidar_pub.publish(lidar_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        lidar_publisher()
    except rospy.ROSInterruptException:
        pass
