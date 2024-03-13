#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class MovementController:
    def __init__(self):
        rospy.init_node('movement_subscriber', anonymous=True)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.movement_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.obstacle_detected = False
        self.obstacle_angle = None

    def movement_callback(self, data):
        # Check if the robot is currently moving
        # For simplicity, let's just print the status
        rospy.loginfo("Received movement status: %s", data.status_list)

    def lidar_callback(self, data):
        # Check LiDAR data for obstacles in front of the robot
        angle_increment = data.angle_increment
        angle_min = data.angle_min
        angle_max = data.angle_max

        # Define the range of angles to consider in front of the robot
        angle_range = 30  # Consider ±30 degrees from the forward direction
        min_index = int((0 - angle_min) / angle_increment) - angle_range
        max_index = int((0 - angle_min) / angle_increment) + angle_range

        # Ensure that the indices are within the valid range
        min_index = max(0, min_index)
        max_index = min(len(data.ranges) - 1, max_index)

        # Check for obstacles within the defined angle range
        distances = data.ranges[min_index:max_index]

        if distances:
            min_distance = min(distances)
            rospy.loginfo("Minimum distance detected in front of the robot: %.2f meters", min_distance)

            if min_distance < 0.2:  # If an obstacle is detected within 0.2 meters
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            rospy.logwarn("No LiDAR readings within the defined angle range.")



    def avoid_obstacle(self):
        # Perform obstacle avoidance maneuvers
        if self.obstacle_detected:
            # Rotate the robot away from the obstacle
            move_cmd = Twist()
            move_cmd.angular.z = 0.3  # Rotate left
            self.cmd_vel_pub.publish(move_cmd)
            rospy.sleep(0.5)  # Rotate for 0.5 seconds

            # Continue rotating until no obstacle in front
            while self.obstacle_detected:
                rospy.sleep(0.1)  # Wait for a short duration
                if not self.obstacle_detected:
                    break  # Exit the loop if obstacle is no longer detected

                move_cmd.angular.z = 0.3  # Keep rotating left
                self.cmd_vel_pub.publish(move_cmd)

            # Stop rotating once obstacle is avoided
            move_cmd.angular.z = 0  # Stop rotation
            self.cmd_vel_pub.publish(move_cmd)

        else:
            # If no obstacle is detected, continue exploring
            self.explore()

    def explore(self):
        # Move forward
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            if not self.obstacle_detected:
                self.explore()  # Continue exploring if no obstacle detected
            else:
                self.avoid_obstacle()  # Avoid obstacle while continuing to move

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = MovementController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
