#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

# Import the ObstacleSubscriber class
from avoiding_obstacles_subscriber import ObstacleSubscriber

class MovementPublisher:
    def __init__(self):
        rospy.init_node('tb3_movement_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize the ObstacleSubscriber instance
        self.obstacle_subscriber = ObstacleSubscriber()
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.linear_speed = 0.2  # Adjust as needed
        self.angular_speed = 0.5  # Adjust as needed
        self.obstacle_detected = False

    def move_robot(self):
        front_arc = self.obstacle_subscriber.get_front_arc()
        if front_arc is None:
            return  # Return if front arc is not available yet

        twist = Twist()
        if not self.obstacle_detected:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0
        else:
            # Calculate the index corresponding to the direction with the furthest distance
            furthest_idx = np.argmax(front_arc)
            # Calculate the angle corresponding to the furthest direction
            angle = (furthest_idx - len(front_arc) // 2) * (self.obstacle_subscriber.subscriber.angle_increment * 180 / np.pi)
            # Calculate the angular speed to turn towards the furthest direction
            twist.angular.z = np.sign(angle) * self.angular_speed

            twist.linear.x = 0  # Stop forward motion
        self.publisher.publish(twist)

if __name__ == '__main__':
    try:
        movement_pub = MovementPublisher()

        while not rospy.is_shutdown():
            # Control robot movement based on obstacle detection
            movement_pub.move_robot()

            movement_pub.rate.sleep()

    except rospy.ROSInterruptException:
        pass
