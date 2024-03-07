#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CmdVelPublisher():
    def __init__(self):
        self.node_name = "cmd_vel_publisher"
        rospy.init_node(self.node_name, anonymous=True)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.loginfo(f"The '{self.node_name}' node is active...")
        self.counter = 0
        self.rate = rospy.Rate(1)  # 1Hz

    def main(self):
        while not rospy.is_shutdown():
            # Stop the robot after 60 seconds
            if self.counter >= 60:
                cmd_vel_msg = Twist()
                self.pub.publish(cmd_vel_msg)
                break

            cmd_vel_msg = Twist()
            if self.counter <= 30:
                # Loop 1: Move anti-clockwise
                cmd_vel_msg.linear.x = 0.11  # Adjust velocity as needed
                cmd_vel_msg.angular.z = 0.22  # Rotate anti-clockwise
            else:
                # Loop 2: Move clockwise
                cmd_vel_msg.linear.x = 0.11  # Adjust velocity as needed
                cmd_vel_msg.angular.z = -0.22  # Rotate clockwise

            self.pub.publish(cmd_vel_msg)
            self.counter += 1
            self.rate.sleep()

if __name__ == '__main__':
    node = CmdVelPublisher()
    node.main()

