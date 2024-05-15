#!/usr/bin/env python3
"""
A wall-following algorithm for maze navigation using the Turtlebot3 Waffle
"""

import rospy
from waffle import Pose, Lidar, Motion

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.pose = Pose()
        self.lidar = Lidar()
        self.motion = Motion()
        
        self.MAX_SPEED_LINEAR = 0.26  # Maximum speed of the robot (linear)
        self.MAX_SPEED_ANGULAR = 0.5 #Maximum speed of robot (angular)
        self.REDUCED_SPEED = 0.18  # Reduced speed while turning
        self.WALL_THRESHOLD = 0.6  # Threshold distance for detecting walls
        self.BUFFER_DISTANCE = 0.3  # Buffer distance to maintain from the walls
        self.FRONT_THRESHOLD = 0.45  # Threshold distance for detecting a wall in front
        self.TURN_THRESHOLD = 0.3  # Threshold distance for turning while moving
        
    def wall_follow(self):
        # Default linear and angular velocities
        linear = self.MAX_SPEED_LINEAR
        angular = 0.0
        
        # Check if there is a wall in front
        front_distance = min(self.lidar.subsets.l1, self.lidar.subsets.front, self.lidar.subsets.r1)
        if front_distance < self.FRONT_THRESHOLD:
            # Wall detected in front, stop the robot
            rospy.loginfo("Wall detected in front. Stopping the robot.")
            linear = 0.0
            
            # Determine the direction to turn based on the left and right distances
            left_distance = self.lidar.subsets.l2
            right_distance = self.lidar.subsets.r2
            
            if left_distance > right_distance:
                # More space on the left, turn left
                rospy.loginfo("Turning left.")
                angular = self.MAX_SPEED_ANGULAR
            else:
                # More space on the right, turn right
                rospy.loginfo("Turning right.")
                angular = -self.MAX_SPEED_ANGULAR
        else:
            # No wall detected in front, continue wall following
            rospy.loginfo("Following the right wall.")
            # Calculate the average distance from the right wall
            right_distance = (self.lidar.subsets.r1 + self.lidar.subsets.r2 + self.lidar.subsets.r3 + self.lidar.subsets.r4) / 4
            
            # Check if the robot is too close to the right wall
            if right_distance < self.TURN_THRESHOLD:
                # Too close to the right wall, steer to the left while moving
                rospy.loginfo("Too close to the right wall. Steering left.")
                linear = self.REDUCED_SPEED
                angular = self.MAX_SPEED_ANGULAR / 1.5
            elif right_distance > self.WALL_THRESHOLD:
                # Too far from the right wall, steer to the right while moving
                rospy.loginfo("Too far from the right wall. Steering right.")
                linear = self.REDUCED_SPEED
                angular = -self.MAX_SPEED_ANGULAR / 1.5
        
        return linear, angular
    
    def run(self):
        while not rospy.is_shutdown():
            linear, angular = self.wall_follow()
            
            self.motion.set_velocity(linear=linear, angular=angular)
            self.motion.publish_velocity()
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = WallFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass