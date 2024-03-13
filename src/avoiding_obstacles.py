#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

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
            twist.angular.z = self.angular_speed  # Rotate until the obstacle is no longer in front
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
