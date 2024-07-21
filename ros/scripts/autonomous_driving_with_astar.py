# File: /Omega-Code/ros/scripts/autonomous_driving_with_astar.py

"""
Autonomous Driving with A* Path Planning and Obstacle Avoidance

This script handles autonomous driving by integrating A* path planning and real-time obstacle avoidance.
It subscribes to sensor topics, processes sensor data, and controls the robot to navigate towards the goal.

Functions:
- image_callback: Processes incoming camera images.
- control_robot: Sends commands to control the robot based on sensor data and path planning.
- obstacle_avoidance: Adjusts the path or stops the robot if an obstacle is detected.
- main: Initializes the ROS node and starts the necessary subscribers.

Dependencies:
- ROS: rospy, geometry_msgs, sensor_msgs, cv_bridge, std_msgs
- TensorFlow: tf.keras
- Path Planning: a_star

Usage:
- Run this script to start autonomous driving with path planning and obstacle avoidance.
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np
import cv2
import heapq

# Load the pre-trained model
model = tf.keras.models.load_model('/path/to/robot_navigation_model.h5')
bridge = CvBridge()

# Define A* Node and Pathfinding functions
class Node:
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def a_star_search(start, goal, grid):
    open_list = []
    heapq.heappush(open_list, Node(start[0], start[1], 0, None))
    closed_list = set()
    came_from = {}

    while open_list:
        current_node = heapq.heappop(open_list)
        current = (current_node.x, current_node.y)
        
        if current in closed_list:
            continue

        if current == goal:
            return reconstruct_path(came_from, current)
        
        closed_list.add(current)
        for neighbor in get_neighbors(current, grid):
            if neighbor in closed_list:
                continue
            tentative_cost = current_node.cost + 1  # Assuming uniform cost
            heapq.heappush(open_list, Node(neighbor[0], neighbor[1], tentative_cost, current))
            came_from[neighbor] = current
    
    return None

def get_neighbors(position, grid):
    neighbors = []
    x, y = position
    if x > 0: neighbors.append((x - 1, y))
    if x < len(grid) - 1: neighbors.append((x + 1, y))
    if y > 0: neighbors.append((x, y - 1))
    if y < len(grid[0]) - 1: neighbors.append((x, y + 1))
    return neighbors

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path

# Robot control and sensor data processing
class AutonomousDriving:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        rospy.Subscriber('/line_tracking/data', Int32MultiArray, self.line_tracking_callback)
        
        self.latest_image = None
        self.latest_ultrasonic = None
        self.latest_line_tracking = None
        self.path = []
        self.goal = (4, 4)  # Example goal
        self.grid = [[0, 1, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 1, 0],
                     [1, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0]]

    def image_callback(self, data):
        self.latest_image = bridge.imgmsg_to_cv2(data, "bgr8")
        self.control_robot()

    def ultrasonic_callback(self, data):
        self.latest_ultrasonic = data.range
        self.control_robot()

    def line_tracking_callback(self, data):
        self.latest_line_tracking = data.data
        self.control_robot()

    def control_robot(self):
        if self.latest_image is not None and self.latest_ultrasonic is not None and self.latest_line_tracking is not None:
            if not self.path:
                self.path = a_star_search((0, 0), self.goal, self.grid)  # Starting at (0, 0)

            if self.path:
                next_position = self.path.pop(0)
                # Move towards the next position based on path planning
                self.move_to_position(next_position)
            
            self.obstacle_avoidance()

    def move_to_position(self, position):
        cmd = Twist()
        current_position = self.get_current_position()
        if position[0] > current_position[0]:
            cmd.linear.x = 0.5  # Move forward
        elif position[0] < current_position[0]:
            cmd.linear.x = -0.5  # Move backward
        elif position[1] > current_position[1]:
            cmd.angular.z = 0.5  # Turn right
        elif position[1] < current_position[1]:
            cmd.angular.z = -0.5  # Turn left
        self.pub.publish(cmd)

    def obstacle_avoidance(self):
        if self.latest_ultrasonic < 0.5:  # Threshold distance to stop
            cmd = Twist()
            cmd.linear.x = 0  # Stop the robot
            self.pub.publish(cmd)

    def get_current_position(self):
        # For simplicity, we return a dummy position (0, 0)
        return (0, 0)

def main():
    rospy.init_node('autonomous_driving_with_astar')
    AutonomousDriving()
    rospy.spin()

if __name__ == '__main__':
    main()
