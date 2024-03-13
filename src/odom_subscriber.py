#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class OdomSubscriber():
    def __init__(self):
        self.node_name = "odom_subscriber"
        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"The '{self.node_name}' node is active...")
        self.counter = 0
        self.rate = rospy.Rate(1)  # 1Hz

    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation 
        pos_x = position.x
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        # Print current pose at 1Hz
        if self.counter % 20 == 0:
            rospy.loginfo(f"x={pos_x:.2f} [m], y={pos_y:.2f} [m], yaw={math.degrees(yaw):.1f} [degrees]")
        
        self.counter += 1

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = OdomSubscriber()
    node.main()


