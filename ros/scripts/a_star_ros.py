# File: /Omega-Code/ros/scripts/a_star_ros.py

"""
ROS Node for A* Path Planning

This script implements a ROS node that uses the A* algorithm for path planning.

Classes:
- Node: Represents a node in the grid.
- AStar: Implements the A* algorithm.

Functions:
- get_neighbors: Gets the neighboring nodes.
- heuristic: Calculates the heuristic cost.
- reconstruct_path: Reconstructs the path from start to goal.
- grid_callback: Callback for receiving the map data.
- main: Initializes the ROS node and starts the path planning.

Dependencies:
- ROS: rospy, geometry_msgs, nav_msgs
- Python: heapq, math

Usage:
- Run this script to start the A* path planning ROS node.
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import heapq
import math

class Node:
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = None

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic

def get_neighbors(node, grid):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
            neighbors.append(Node(x, y, node.cost + 1, 0))
    return neighbors

def heuristic(node, goal):
    return abs(node.x - goal.x) + abs(node.y - goal.y)

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def grid_callback(msg):
    width, height = msg.info.width, msg.info.height
    grid = [[0 if msg.data[i + j * width] == 0 else 1 for i in range(width)] for j in range(height)]
    return grid

def main():
    rospy.init_node('a_star')
    grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
    grid = grid_callback(grid_msg)

    start = (0, 0)
    goal = (len(grid) - 1, len(grid[0]) - 1)

    open_list = []
    start_node = Node(start[0], start[1], 0, heuristic(start_node, goal_node))
    goal_node = Node(goal[0], goal[1], 0, 0)
    heapq.heappush(open_list, start_node)
    closed_list = set()
    came_from = {}

    while open_list:
        current_node = heapq.heappop(open_list)
        if (current_node.x, current_node.y) in closed_list:
            continue
        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            path = reconstruct_path(current_node)
            break
        closed_list.add((current_node.x, current_node.y))
        for neighbor in get_neighbors(current_node, grid):
            if (neighbor.x, neighbor.y) not in closed_list:
                neighbor.parent = current_node
                neighbor.heuristic = heuristic(neighbor, goal_node)
                heapq.heappush(open_list, neighbor)
                came_from[(neighbor.x, neighbor.y)] = (current_node.x, current_node.y)

    path_pub = rospy.Publisher('/a_star_path', PoseStamped, queue_size=10)
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

