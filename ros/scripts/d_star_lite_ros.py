# File: /Omega-Code/ros/scripts/d_star_lite_ros.py

"""
ROS Node for D* Lite Path Planning

This script implements a ROS node that uses the D* Lite algorithm for dynamic path planning.

Functions:
- main: Initializes the ROS node and starts the path planning.

Dependencies:
- ROS: rospy, geometry_msgs, nav_msgs
- Python: heapq, math

Usage:
- Run this script to start the D* Lite path planning ROS node.
"""

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import heapq
import math

class Node:
    def __init__(self, x, y, g=float('inf'), rhs=float('inf')):
        self.x = x
        self.y = y
        self.g = g
        self.rhs = rhs
        self.neighbors = []

    def __lt__(self, other):
        return self.g < other.g

def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

class DStarLite:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.open_list = []
        self.k_m = 0
        self.rhs[self.goal] = 0
        heapq.heappush(self.open_list, (self.calculate_key(self.goal), self.goal))
        self.came_from = {}

    def calculate_key(self, node):
        return (min(node.g, node.rhs) + heuristic(self.start, node) + self.k_m, min(node.g, node.rhs))

    def update_vertex(self, node):
        if node != self.goal:
            node.rhs = min([self.grid[neighbor[0]][neighbor[1]].g + 1 for neighbor in node.neighbors])
        if node in self.open_list:
            self.open_list.remove((self.calculate_key(node), node))
            heapq.heapify(self.open_list)
        if node.g != node.rhs:
            heapq.heappush(self.open_list, (self.calculate_key(node), node))

    def compute_shortest_path(self):
        while self.open_list and (self.open_list[0][0] < self.calculate_key(self.start) or self.start.rhs != self.start.g):
            k_old, u = heapq.heappop(self.open_list)
            if k_old < self.calculate_key(u):
                heapq.heappush(self.open_list, (self.calculate_key(u), u))
            elif u.g > u.rhs:
                u.g = u.rhs
                for neighbor in u.neighbors:
                    self.update_vertex(neighbor)
            else:
                u.g = float('inf')
                self.update_vertex(u)
                for neighbor in u.neighbors:
                    self.update_vertex(neighbor)
        return self.reconstruct_path()

    def reconstruct_path(self):
        path = []
        current = self.start
        while current != self.goal:
            path.append(current)
            current = min(current.neighbors, key=lambda n: self.grid[n[0]][n[1]].g)
        path.append(self.goal)
        return path

def main():
    rospy.init_node('d_star_lite')
    start = Node(0, 0)
    goal = Node(4, 4)
    grid = [[0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 1, 0],
            [1, 1, 0, 1, 0],
            [0, 0, 0, 0, 0]]

    nodes = [[Node(x, y) for y in range(len(grid[0]))] for x in range(len(grid))]
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x][y] == 0:
                neighbors = []
                if x > 0 and grid[x-1][y] == 0: neighbors.append((x-1, y))
                if x < len(grid)-1 and grid[x+1][y] == 0: neighbors.append((x+1, y))
                if y > 0 and grid[x][y-1] == 0: neighbors.append((x, y-1))
                if y < len(grid[0])-1: neighbors.append((x, y+1))
                nodes[x][y].neighbors = neighbors

    dstar = DStarLite(start, goal, nodes)
    path = dstar.compute_shortest_path()
    for node in path:
        rospy.loginfo(f"Path node: ({node.x}, {node.y})")

    rospy.spin()

if __name__ == "__main__":
    main()


