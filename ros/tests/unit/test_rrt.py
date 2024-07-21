# File: /Omega-Code/ros/tests/unit/test_rrt.py

import unittest
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

class TestRRT(unittest.TestCase):

    def test_distance(self):
        node1 = Node(0, 0)
        node2 = Node(3, 4)
        self.assertEqual(distance(node1, node2), 5.0)

    def test_rrt(self):
        grid = [[0, 1, 0, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 0, 0, 1, 0],
                [1, 1, 0, 1, 0],
                [0, 0, 0, 0, 0]]
        start = (0, 0)
        goal = (4, 4)
        rrt = RRT(start, goal, grid)
        path = rrt.build_tree()
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

if __name__ == '__main__':
    unittest.main()

