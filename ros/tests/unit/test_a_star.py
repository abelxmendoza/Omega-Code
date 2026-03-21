# File: /Omega-Code/ros/tests/unit/test_a_star.py

import unittest
from scripts.a_star import a_star_search

class TestAStarSearch(unittest.TestCase):
    def setUp(self):
        self.grid = [[0, 1, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 1, 0],
                     [1, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0]]
        self.start = (0, 0)
        self.goal = (4, 4)

    def test_path_found(self):
        path = a_star_search(self.start, self.goal, self.grid)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.start)
        self.assertEqual(path[-1], self.goal)

    def test_no_path(self):
        # Row 1 is a wall with a single gap at (1,1); blocking it seals all routes
        grid = [[0, 0, 0],
                [1, 0, 1],
                [0, 0, 0]]
        grid[1][1] = 1  # Close the only gap — no path exists
        path = a_star_search((0, 0), (2, 2), grid)
        self.assertIsNone(path)

    def test_start_equals_goal(self):
        path = a_star_search(self.start, self.start, self.grid)
        self.assertEqual(path, [self.start])

if __name__ == '__main__':
    unittest.main()

