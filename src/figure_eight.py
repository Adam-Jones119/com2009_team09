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
        self.rate = rospy.Rate(10)  

    def main(self):
        while not rospy.is_shutdown():
            
            if self.counter >= 610:
                cmd_vel_msg = Twist()
                self.pub.publish(cmd_vel_msg)
                break

            cmd_vel_msg = Twist()
            if self.counter <= 310:
                # Loop 1: Move anti-clockwise
                cmd_vel_msg.linear.x = 0.115  
                cmd_vel_msg.angular.z = 0.23  
            else:
                # Loop 2: Move clockwise
                cmd_vel_msg.linear.x = 0.115  
                cmd_vel_msg.angular.z = -0.23  

            self.pub.publish(cmd_vel_msg)
            self.counter += 1
            self.rate.sleep()

if __name__ == '__main__':
    node = CmdVelPublisher()
    node.main()

