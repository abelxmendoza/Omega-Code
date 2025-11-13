#!/usr/bin/env python3
"""
Path Planner Node

ROS2 node for path planning using A*, D*, or RRT algorithms.
Subscribes to goal poses and publishes planned paths.

Topics:
- Subscribes: /goal_pose (geometry_msgs/PoseStamped)
- Publishes: /plan (nav_msgs/Path)
- Subscribes: /map (nav_msgs/OccupancyGrid) - optional
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import math
import numpy as np


class PathPlanner(Node):
    """Path planning node using A* algorithm."""

    def __init__(self):
        super().__init__('path_planner')
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # State
        self.current_pose = None
        self.map_data = None
        self.map_resolution = 0.05  # meters per pixel
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # Algorithm selection
        self.algorithm = self.declare_parameter('algorithm', 'astar').value  # astar, dstar, rrt
        
        self.get_logger().info(f'Path planner started with algorithm: {self.algorithm}')

    def goal_callback(self, msg: PoseStamped):
        """Handle goal pose and plan path."""
        self.get_logger().info(f'Received goal: ({msg.pose.position.x}, {msg.pose.position.y})')
        
        # Get current pose (from odometry or default)
        if self.current_pose is None:
            # Use origin as start
            start = (0.0, 0.0)
        else:
            start = (self.current_pose.position.x, self.current_pose.position.y)
        
        goal = (msg.pose.position.x, msg.pose.position.y)
        
        # Plan path
        if self.algorithm == 'astar':
            path = self.plan_astar(start, goal)
        elif self.algorithm == 'dstar':
            path = self.plan_dstar(start, goal)
        elif self.algorithm == 'rrt':
            path = self.plan_rrt(start, goal)
        else:
            path = self.plan_astar(start, goal)
        
        if path:
            # Publish path
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'odom'
            
            for point in path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'Published path with {len(path)} waypoints')
        else:
            self.get_logger().warn('Path planning failed')

    def map_callback(self, msg: OccupancyGrid):
        """Update map data."""
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.get_logger().debug('Map updated')

    def plan_astar(self, start, goal):
        """A* path planning algorithm."""
        # Simplified A* implementation
        # In production, would use proper grid-based A* with obstacles
        
        # Simple straight-line path (for now)
        # Would be replaced with proper A* on grid
        path = []
        
        # Interpolate between start and goal
        steps = int(math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2) / 0.1)
        steps = max(2, steps)
        
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + (goal[0] - start[0]) * t
            y = start[1] + (goal[1] - start[1]) * t
            path.append((x, y))
        
        return path

    def plan_dstar(self, start, goal):
        """D* path planning algorithm."""
        # Placeholder - would implement D* Lite or similar
        # For now, use A*
        return self.plan_astar(start, goal)

    def plan_rrt(self, start, goal):
        """RRT path planning algorithm."""
        # Placeholder - would implement RRT or RRT*
        # For now, use A*
        return self.plan_astar(start, goal)

    def check_collision(self, x, y):
        """Check if position collides with obstacles."""
        if self.map_data is None:
            return False  # No map, assume no obstacles
        
        # Convert world coordinates to map coordinates
        # This is simplified - would need proper map indexing
        return False  # Placeholder


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

