# File: /Omega-Code/ros/tests/unit/test_d_star_lite.py

import unittest
from d_star_lite import Node, DStarLite, heuristic

class TestDStarLite(unittest.TestCase):

    def setUp(self):
        self.grid = [[0, 1, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 1, 0],
                     [1, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0]]
        self.start = Node(0, 0)
        self.goal = Node(4, 4)
        self.nodes = [[Node(x, y) for y in range(len(self.grid[0]))] for x in range(len(self.grid))]
        for x in range(len(self.grid)):
            for y in range(len(self.grid[0])):
                if self.grid[x][y] == 0:
                    neighbors = []
                    if x > 0 and self.grid[x-1][y] == 0: neighbors.append((x-1, y))
                    if x < len(self.grid)-1 and self.grid[x+1][y] == 0: neighbors.append((x+1, y))
                    if y > 0 and self.grid[x][y-1] == 0: neighbors.append((x, y-1))
                    if y < len(self.grid[0])-1 and self.grid[x][y+1] == 0: neighbors.append((x, y+1))
                    self.nodes[x][y].neighbors = neighbors

    def test_heuristic(self):
        self.assertEqual(heuristic(self.start, self.goal), 8)

    def test_dstar_lite(self):
        dstar = DStarLite(self.start, self.goal, self.nodes)
        path = dstar.compute_shortest_path()
        self.assertTrue(path)

if __name__ == '__main__':
    unittest.main()


