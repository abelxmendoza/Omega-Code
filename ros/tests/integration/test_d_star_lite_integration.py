# File: /Omega-Code/ros/tests/integration/test_d_star_lite_integration.py

import unittest
from d_star_lite_ros import Node as ROSNode, DStarLite as ROSDStarLite
from d_star_lite import Node as PyNode, DStarLite as PyDStarLite

class TestDStarLiteIntegration(unittest.TestCase):

    def setUp(self):
        self.grid = [[0, 1, 0, 0, 0],
                     [0, 1, 0, 1, 0],
                     [0, 0, 0, 1, 0],
                     [1, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0]]
        self.start_ros = ROSNode(0, 0)
        self.goal_ros = ROSNode(4, 4)
        self.start_py = PyNode(0, 0)
        self.goal_py = PyNode(4, 4)
        self.nodes_ros = [[ROSNode(x, y) for y in range(len(self.grid[0]))] for x in range(len(self.grid))]
        self.nodes_py = [[PyNode(x, y) for y in range(len(self.grid[0]))] for x in range(len(self.grid))]
        for x in range(len(self.grid)):
            for y in range(len(self.grid[0])):
                if self.grid[x][y] == 0:
                    neighbors = []
                    if x > 0 and self.grid[x-1][y] == 0: neighbors.append((x-1, y))
                    if x < len(self.grid)-1 and self.grid[x+1][y] == 0: neighbors.append((x+1, y))
                    if y > 0 and self.grid[x][y-1] == 0: neighbors.append((x, y-1))
                    if y < len(self.grid[0])-1 and self.grid[x][y+1] == 0: neighbors.append((x, y+1))
                    self.nodes_ros[x][y].neighbors = neighbors
                    self.nodes_py[x][y].neighbors = neighbors

    def test_dstar_lite_integration(self):
        dstar_ros = ROSDStarLite(self.start_ros, self.goal_ros, self.nodes_ros)
        path_ros = dstar_ros.compute_shortest_path()
        self.assertTrue(path_ros)

        dstar_py = PyDStarLite(self.start_py, self.goal_py, self.nodes_py)
        path_py = dstar_py.compute_shortest_path()
        self.assertTrue(path_py)

        self.assertEqual(len(path_ros), len(path_py))

if __name__ == '__main__':
    unittest.main()

