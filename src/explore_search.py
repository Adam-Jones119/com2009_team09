#!/usr/bin/env python3

import rospy
from waffle import Motion, Pose, Lidar
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import subprocess
import random
import math

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node')
        
        #self.slam_process = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=gmapping"])
        
        # Initialize motion, pose and lidar objects
        self.motion = Motion(debug=True)
        self.pose = Pose(debug=True)
        self.lidar = Lidar(debug=True)
        
        # Initialize map and beacon detection
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.beacon_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.beacon_callback)
        self.depth_image_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)
        self.cv_bridge = CvBridge()
        self.beacon_found = False
        
        # Parameters for exploration
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.collision_distance = 0.45
        
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        
    
    #def __del__(self):
        # Terminate the SLAM/gmapping process when the node is shutdown
        #self.slam_process.terminate()
        
    def depth_image_callback(self, msg):
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.process_depth_image(depth_image)
        except CvBridgeError as e:
            rospy.logerr("Failed to convert depth image: %s", str(e))
            
    def process_depth_image(self, depth_image):
        # Get the depth values at specific regions of interest (ROI)
        height, width = depth_image.shape
        center_roi = depth_image[height//2-20:height//2+20, width//2-20:width//2+20]
        left_roi = depth_image[height//2-20:height//2+20, :width//4]
        right_roi = depth_image[height//2-20:height//2+20, width*3//4:]
        
        # Calculate the average depth values for each ROI
        center_depth = np.mean(center_roi)
        left_depth = np.mean(left_roi)
        right_depth = np.mean(right_roi)
        
        # Update the collision avoidance logic based on the depth values
        self.update_collision_avoidance(center_depth, left_depth, right_depth)
        
    def map_callback(self, msg):
        rospy.loginfo("Received map data")
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        rospy.loginfo(f"Map shape: {self.map_data.shape}")
        rospy.loginfo(f"Map data: {self.map_data}")
        
    def beacon_callback(self, msg):
        rospy.loginfo("Received camera image")
        # Convert image to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # TODO: Implement color detection to identify beacon
        # If beacon of specified color is found, set self.beacon_found to True
        
    def avoid_collision(self, center_depth, left_depth, right_depth):
        if center_depth < self.collision_distance:
            rospy.logwarn("Obstacle detected! Initiating collision avoidance.")
            self.motion.stop()
            
            if left_depth > right_depth:
                rospy.loginfo("Rotating to the left (more clearance)")
                target_angle = 30  # Rotate left by 30 degrees
            else:
                rospy.loginfo("Rotating to the right (more clearance)")
                target_angle = -30  # Rotate right by 30 degrees
            

            
            # Set the angular velocity (adjust as needed)
            angular_velocity = 0.5  # Angular velocity in radians per second
            
            # Calculate the rotation duration based on the target angle and angular velocity
            rotation_duration = abs(target_angle) * (math.pi / 180) / angular_velocity
            
            # Set the angular velocity and publish it for the calculated duration
            self.motion.set_velocity(angular=angular_velocity)
            self.motion.publish_velocity()
            rospy.sleep(rotation_duration)
            
            self.motion.stop()
            rospy.loginfo("Obstacle cleared. Resuming exploration.")
            return True
    
        return False
    
    
    #def find_frontier(self):
        # Find unexplored regions in the map
        unexplored = (self.map_data == -1)
        
        # Find frontier regions (unexplored cells adjacent to explored cells)
        kernel = np.ones((3, 3), dtype=np.uint8)
        eroded = cv2.erode(unexplored.astype(np.uint8), kernel, iterations=5)
        frontier = np.logical_and(unexplored, np.logical_not(eroded))
        
        # Find contours of frontier regions
        contours, _ = cv2.findContours(frontier.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest frontier region
        if len(contours) > 0:
            largest_frontier = max(contours, key=cv2.contourArea)
            
            # Find the centroid of the largest frontier region
            moments = cv2.moments(largest_frontier)
            if moments["m00"] != 0:
                centroid_x = int(moments["m10"] / moments["m00"])
                centroid_y = int(moments["m01"] / moments["m00"])
                return centroid_x, centroid_y
        
        return None, None
        


    def explore(self):
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < 180:
            
            # Check for collision and perform avoidance maneuver
            center_depth = self.center_depth
            left_depth = self.left_depth
            right_depth = self.right_depth
            
            if self.avoid_collision(center_depth, left_depth, right_depth):
                continue
            
            # Randomly choose between wall following and random exploration
            if random.random() < 0.7:  # 70% chance of wall following
                # Wall following behavior
                if self.lidar.subsets.l1 < self.lidar.subsets.r1:
                    rospy.loginfo("Following left wall...")
                    self.motion.set_velocity(linear=self.linear_speed, angular=self.angular_speed)
                else:
                    rospy.loginfo("Following right wall...")
                    self.motion.set_velocity(linear=self.linear_speed, angular=-self.angular_speed)
            else:
                # Random exploration behavior
                if random.random() < 0.5:  # 50% chance of moving forward
                    rospy.loginfo("Exploring forward...")
                    self.motion.set_velocity(linear=self.linear_speed)
                else:
                    # Randomly choose a rotation direction
                    if random.random() < 0.5:  # 50% chance of rotating left
                        rospy.loginfo("Rotating left...")
                        self.motion.set_velocity(angular=self.angular_speed)
                    else:
                        rospy.loginfo("Rotating right...")
                        self.motion.set_velocity(angular=-self.angular_speed)
            
            self.motion.publish_velocity()
            rospy.sleep(0.1)
            
        self.motion.stop()
        rospy.loginfo("Exploration completed.")
if __name__ == '__main__':
    node = ExplorationNode()
    node.explore()