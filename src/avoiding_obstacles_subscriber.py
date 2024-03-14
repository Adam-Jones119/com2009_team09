#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

class ObstacleSubscriber:
    def __init__(self):
        rospy.init_node('tb3_obstacle_subscriber', anonymous=True)
        self.publisher = rospy.Publisher('/obstacle_detected', Bool, queue_size=10)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.threshold_distance = 0.5  # Adjust as needed

    def laserscan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        min_distance = front_arc.min()

        # Check if there is an obstacle within the threshold distance
        obstacle_detected = min_distance > 0 and min_distance < self.threshold_distance

        rospy.loginfo("Minimum distance to obstacle: {}".format(min_distance))
        rospy.loginfo("Obstacle detected: {}".format(obstacle_detected))

        self.publisher.publish(Bool(obstacle_detected))  # Publish obstacle detection result as Bool message

if __name__ == '__main__':
    try:
        obstacle_sub = ObstacleSubscriber()
        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        pass
