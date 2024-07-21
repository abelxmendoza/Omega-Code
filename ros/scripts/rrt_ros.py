# File: /Omega-Code/ros/scripts/rrt_ros.py

"""
ROS Node for RRT Path Planning

This script implements a ROS node that uses the RRT (Rapidly-exploring Random Trees) algorithm for path planning.

Classes:
- Node: Represents a node in the tree.
- RRT: Implements the RRT algorithm.

Functions:
- distance: Calculates the Euclidean distance between two nodes.
- main: Initializes the ROS node and starts the path planning.

Dependencies:
- ROS: rospy, geometry_msgs, nav_msgs
- Python: random, math

Usage:
- Run this script to start the RRT path planning ROS node.
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

class RRT:
    def __init__(self, start, goal, grid, max_iter=500):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.grid = grid
        self.max_iter = max_iter
        self.tree = [self.start]

    def get_random_node(self):
        x = random.randint(0, len(self.grid) - 1)
        y = random.randint(0, len(self.grid[0]) - 1)
        return Node(x, y)

    def get_nearest_node(self, random_node):
        return min(self.tree, key=lambda node: distance(node, random_node))

    def is_collision_free(self, node):
        return self.grid[node.x][node.y] == 0

    def build_tree(self):
        for _ in range(self.max_iter):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(random_node)

            new_node = Node(random_node.x, random_node.y)
            new_node.parent = nearest_node

            if self.is_collision_free(new_node):
                self.tree.append(new_node)
                if distance(new_node, self.goal) < 1:
                    self.goal.parent = new_node
                    self.tree.append(self.goal)
                    return self.reconstruct_path()
        return None

    def reconstruct_path(self):
        path = []
        current = self.goal
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path

def grid_callback(msg):
    grid = [[0 if cell == 0 else 1 for cell in row] for row in msg.data]
    return grid

def main():
    rospy.init_node('rrt')
    grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
    grid = grid_callback(grid_msg)

    start = (0, 0)
    goal = (len(grid)-1, len(grid[0])-1)

    rrt = RRT(start, goal, grid)
    path = rrt.build_tree()

    path_pub = rospy.Publisher('/rrt_path', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)

    for node in path:
        pose = PoseStamped()
        pose.pose.position.x = node[0]
        pose.pose.position.y = node[1]
        path_pub.publish(pose)
        rospy.loginfo(f"Publishing node: ({node[0]}, {node[1]})")
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()

