#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

class ObstacleSubscriber:
    def __init__(self):
        rospy.init_node('tb3_obstacle_subscriber', anonymous=True)
        self.publisher = rospy.Publisher('/obstacle_detected', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.threshold_distance = 0.5  # Adjust as needed
        self.front_arc = None
        
    def laserscan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        # Filter out zero values
        front_arc = front_arc[front_arc > 0]

        if len(front_arc) == 0:
            # No valid distances found
            min_distance = float('inf')  # Set min_distance to infinity
        else:
            # Calculate the minimum distance
            min_distance = front_arc.min()

        # Check if there is an obstacle within the threshold distance
        obstacle_detected = "None"
        on_left = np.array(left_arc).min() < self.threshold_distance
        on_right = np.array(right_arc).min() < self.threshold_distance

        if on_left:
            obstacle_detected = "Left"
            if on_right:
                obstacle_detected = "Both"
        elif on_right:
            obstacle_detected = "Right"
        else:
            obstacle_detected = "None"


        rospy.loginfo("Minimum distance to obstacle: {}".format(min_distance))
        rospy.loginfo("Obstacle detected: {}".format(obstacle_detected))
        rospy.loginfo("Obstacle on left: {}".format(on_left))
        rospy.loginfo("Obstacle on right: {}".format(on_right))

        self.publisher.publish(obstacle_detected)  # Publish obstacle detection result as Bool message
        self.front_arc = front_arc
        
if __name__ == '__main__':
    try:
        obstacle_sub = ObstacleSubscriber()
        rospy.spin()  # Keep the node running

    except rospy.ROSInterruptException:
        pass
