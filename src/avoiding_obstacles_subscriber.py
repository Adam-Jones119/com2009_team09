#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
import random  # Import random module for generating random numbers

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
        
        # Filter out zero values
        front_arc = front_arc[front_arc > 0]

        if len(front_arc) == 0:
            # No valid distances found
            min_distance = float('inf')  # Set min_distance to infinity
        else:
            # Calculate the minimum distance
            min_distance = front_arc.min()

        # Check if there is an obstacle within the threshold distance
        obstacle_detected = min_distance < self.threshold_distance

        rospy.loginfo("Minimum distance to obstacle: {}".format(min_distance))
        rospy.loginfo("Obstacle detected: {}".format(obstacle_detected))

        self.publisher.publish(Bool(obstacle_detected))  # Publish obstacle detection result as Bool message

class MovementPublisher:
    def __init__(self):
        rospy.init_node('tb3_movement_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_subscriber = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.linear_speed = 0.2  # Adjust as needed
        self.angular_speed = 0.5  # Adjust as needed
        self.obstacle_detected = False

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def move_robot(self):
        twist = Twist()
        if not self.obstacle_detected:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0
        else:
            twist.linear.x = 0  # Stop forward motion
            twist.angular.z = random.choice([-self.angular_speed, self.angular_speed])  # Randomly choose left or right turn
        self.publisher.publish(twist)

if __name__ == '__main__':
    try:
        obstacle_sub = ObstacleSubscriber()
        movement_pub = MovementPublisher()

        while not rospy.is_shutdown():
            # Control robot movement based on obstacle detection
            movement_pub.move_robot()

            rospy.spin()  # Keep the node running

            movement_pub.rate.sleep()

    except rospy.ROSInterruptException:
        pass
