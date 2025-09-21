"""
AI Navigation System
Implements autonomous navigation with path planning, obstacle avoidance, and learning
"""

import asyncio
import logging
import numpy as np
import json
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import math
import time
from collections import deque

logger = logging.getLogger(__name__)

class NavigationMode(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    LEARNING = "learning"
    EMERGENCY = "emergency"

class ObstacleType(Enum):
    STATIC = "static"
    DYNAMIC = "dynamic"
    UNKNOWN = "unknown"

@dataclass
class Point:
    """2D point representation"""
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate distance to another point"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Point') -> float:
        """Calculate angle to another point in radians"""
        return math.atan2(other.y - self.y, other.x - self.x)

@dataclass
class Obstacle:
    """Obstacle representation"""
    position: Point
    radius: float
    obstacle_type: ObstacleType
    confidence: float = 1.0
    timestamp: float = 0.0

@dataclass
class NavigationState:
    """Current navigation state"""
    position: Point
    heading: float  # radians
    velocity: float
    target: Optional[Point] = None
    path: List[Point] = None
    obstacles: List[Obstacle] = None
    mode: NavigationMode = NavigationMode.MANUAL

class AStarPlanner:
    """A* path planning algorithm"""
    
    def __init__(self, grid_size: float = 0.1, max_iterations: int = 10000):
        self.grid_size = grid_size
        self.max_iterations = max_iterations
        self.grid_cache = {}
    
    def plan_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
        """Plan path using A* algorithm"""
        try:
            # Convert to grid coordinates
            start_grid = self._point_to_grid(start)
            goal_grid = self._point_to_grid(goal)
            
            # Create obstacle grid
            obstacle_grid = self._create_obstacle_grid(obstacles)
            
            # A* algorithm
            open_set = [(0, start_grid)]
            came_from = {}
            g_score = {start_grid: 0}
            f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
            
            iterations = 0
            while open_set and iterations < self.max_iterations:
                iterations += 1
                
                # Get node with lowest f_score
                current = min(open_set, key=lambda x: x[0])[1]
                open_set = [(f, pos) for f, pos in open_set if pos != current]
                
                if current == goal_grid:
                    # Reconstruct path
                    path = self._reconstruct_path(came_from, current)
                    return [self._grid_to_point(p) for p in path]
                
                # Explore neighbors
                for neighbor in self._get_neighbors(current):
                    if self._is_obstacle(neighbor, obstacle_grid):
                        continue
                    
                    tentative_g_score = g_score[current] + self._distance(current, neighbor)
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_grid)
                        
                        if not any(pos == neighbor for _, pos in open_set):
                            open_set.append((f_score[neighbor], neighbor))
            
            logger.warning("A* path planning failed - no path found")
            return []
            
        except Exception as e:
            logger.error(f"Error in A* path planning: {e}")
            return []
    
    def _point_to_grid(self, point: Point) -> Tuple[int, int]:
        """Convert point to grid coordinates"""
        return (int(point.x / self.grid_size), int(point.y / self.grid_size))
    
    def _grid_to_point(self, grid: Tuple[int, int]) -> Point:
        """Convert grid coordinates to point"""
        return Point(grid[0] * self.grid_size, grid[1] * self.grid_size)
    
    def _create_obstacle_grid(self, obstacles: List[Obstacle]) -> Dict[Tuple[int, int], bool]:
        """Create obstacle grid from obstacle list"""
        grid = {}
        for obstacle in obstacles:
            center_grid = self._point_to_grid(obstacle.position)
            radius_grid = int(obstacle.radius / self.grid_size) + 1
            
            for dx in range(-radius_grid, radius_grid + 1):
                for dy in range(-radius_grid, radius_grid + 1):
                    grid_pos = (center_grid[0] + dx, center_grid[1] + dy)
                    if self._distance(center_grid, grid_pos) <= radius_grid:
                        grid[grid_pos] = True
        
        return grid
    
    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((pos[0] + dx, pos[1] + dy))
        return neighbors
    
    def _is_obstacle(self, pos: Tuple[int, int], obstacle_grid: Dict[Tuple[int, int], bool]) -> bool:
        """Check if position is obstacle"""
        return obstacle_grid.get(pos, False)
    
    def _distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate distance between grid positions"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Heuristic function for A* (Manhattan distance)"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

class ObstacleAvoidance:
    """Dynamic obstacle avoidance using potential fields"""
    
    def __init__(self, repulsive_force: float = 1.0, attractive_force: float = 0.5):
        self.repulsive_force = repulsive_force
        self.attractive_force = attractive_force
    
    def calculate_force(self, position: Point, target: Point, obstacles: List[Obstacle]) -> Point:
        """Calculate resultant force for obstacle avoidance"""
        try:
            # Attractive force towards target
            target_vector = Point(target.x - position.x, target.y - position.y)
            target_distance = math.sqrt(target_vector.x**2 + target_vector.y**2)
            
            if target_distance > 0:
                attractive_force = self.attractive_force / target_distance
                attractive_vector = Point(
                    target_vector.x * attractive_force / target_distance,
                    target_vector.y * attractive_force / target_distance
                )
            else:
                attractive_vector = Point(0, 0)
            
            # Repulsive forces from obstacles
            repulsive_vector = Point(0, 0)
            for obstacle in obstacles:
                obstacle_vector = Point(position.x - obstacle.position.x, position.y - obstacle.position.y)
                obstacle_distance = math.sqrt(obstacle_vector.x**2 + obstacle_vector.y**2)
                
                if obstacle_distance < obstacle.radius + 0.5:  # Safety margin
                    if obstacle_distance > 0:
                        repulsive_force = self.repulsive_force / (obstacle_distance - obstacle.radius)**2
                        repulsive_vector.x += obstacle_vector.x * repulsive_force / obstacle_distance
                        repulsive_vector.y += obstacle_vector.y * repulsive_force / obstacle_distance
            
            # Combine forces
            resultant_force = Point(
                attractive_vector.x + repulsive_vector.x,
                attractive_vector.y + repulsive_vector.y
            )
            
            return resultant_force
            
        except Exception as e:
            logger.error(f"Error calculating obstacle avoidance force: {e}")
            return Point(0, 0)

class LearningSystem:
    """Machine learning system for navigation improvement"""
    
    def __init__(self, learning_rate: float = 0.01, memory_size: int = 1000):
        self.learning_rate = learning_rate
        self.memory_size = memory_size
        self.experience_memory = deque(maxlen=memory_size)
        self.success_patterns = {}
        self.failure_patterns = {}
    
    def record_experience(self, state: NavigationState, action: str, reward: float, next_state: NavigationState):
        """Record navigation experience for learning"""
        try:
            experience = {
                'timestamp': time.time(),
                'state': {
                    'position': (state.position.x, state.position.y),
                    'heading': state.heading,
                    'velocity': state.velocity,
                    'target': (state.target.x, state.target.y) if state.target else None,
                    'obstacle_count': len(state.obstacles) if state.obstacles else 0
                },
                'action': action,
                'reward': reward,
                'next_state': {
                    'position': (next_state.position.x, next_state.position.y),
                    'heading': next_state.heading,
                    'velocity': next_state.velocity
                }
            }
            
            self.experience_memory.append(experience)
            
            # Update success/failure patterns
            if reward > 0:
                self._update_success_pattern(state, action)
            else:
                self._update_failure_pattern(state, action)
                
        except Exception as e:
            logger.error(f"Error recording experience: {e}")
    
    def get_recommended_action(self, state: NavigationState) -> Optional[str]:
        """Get recommended action based on learned patterns"""
        try:
            # Simple pattern matching for now
            state_key = self._get_state_key(state)
            
            if state_key in self.success_patterns:
                # Return most successful action for this state
                best_action = max(self.success_patterns[state_key].items(), key=lambda x: x[1])
                return best_action[0]
            
            return None
            
        except Exception as e:
            logger.error(f"Error getting recommended action: {e}")
            return None
    
    def _get_state_key(self, state: NavigationState) -> str:
        """Create state key for pattern matching"""
        obstacle_count = len(state.obstacles) if state.obstacles else 0
        return f"obs_{obstacle_count}_vel_{int(state.velocity)}"
    
    def _update_success_pattern(self, state: NavigationState, action: str):
        """Update success patterns"""
        state_key = self._get_state_key(state)
        if state_key not in self.success_patterns:
            self.success_patterns[state_key] = {}
        
        if action not in self.success_patterns[state_key]:
            self.success_patterns[state_key][action] = 0
        
        self.success_patterns[state_key][action] += 1
    
    def _update_failure_pattern(self, state: NavigationState, action: str):
        """Update failure patterns"""
        state_key = self._get_state_key(state)
        if state_key not in self.failure_patterns:
            self.failure_patterns[state_key] = {}
        
        if action not in self.failure_patterns[state_key]:
            self.failure_patterns[state_key][action] = 0
        
        self.failure_patterns[state_key][action] += 1

class AINavigationSystem:
    """Main AI navigation system"""
    
    def __init__(self):
        self.state = NavigationState(
            position=Point(0, 0),
            heading=0,
            velocity=0,
            obstacles=[],
            mode=NavigationMode.MANUAL
        )
        
        self.planner = AStarPlanner()
        self.obstacle_avoidance = ObstacleAvoidance()
        self.learning_system = LearningSystem()
        
        self.path_index = 0
        self.last_plan_time = 0
        self.plan_interval = 1.0  # seconds
        
        self.running = False
        self.navigation_task = None
    
    async def start(self):
        """Start AI navigation system"""
        self.running = True
        self.navigation_task = asyncio.create_task(self._navigation_loop())
        logger.info("AI Navigation system started")
    
    async def stop(self):
        """Stop AI navigation system"""
        self.running = False
        if self.navigation_task:
            self.navigation_task.cancel()
            try:
                await self.navigation_task
            except asyncio.CancelledError:
                pass
        logger.info("AI Navigation system stopped")
    
    async def set_target(self, target: Point):
        """Set navigation target"""
        self.state.target = target
        self.path_index = 0
        logger.info(f"Navigation target set to ({target.x}, {target.y})")
    
    async def update_position(self, position: Point, heading: float, velocity: float):
        """Update robot position and state"""
        self.state.position = position
        self.state.heading = heading
        self.state.velocity = velocity
    
    async def update_obstacles(self, obstacles: List[Obstacle]):
        """Update obstacle information"""
        self.state.obstacles = obstacles
    
    async def set_mode(self, mode: NavigationMode):
        """Set navigation mode"""
        self.state.mode = mode
        logger.info(f"Navigation mode set to {mode.value}")
    
    async def _navigation_loop(self):
        """Main navigation control loop"""
        while self.running:
            try:
                if self.state.mode == NavigationMode.AUTONOMOUS:
                    await self._autonomous_navigation()
                elif self.state.mode == NavigationMode.LEARNING:
                    await self._learning_navigation()
                
                await asyncio.sleep(0.1)  # 10Hz control loop
                
            except Exception as e:
                logger.error(f"Error in navigation loop: {e}")
                await asyncio.sleep(0.1)
    
    async def _autonomous_navigation(self):
        """Autonomous navigation control"""
        if not self.state.target:
            return
        
        # Check if we need to replan
        current_time = time.time()
        if current_time - self.last_plan_time > self.plan_interval:
            await self._plan_path()
            self.last_plan_time = current_time
        
        # Follow current path
        if self.state.path and self.path_index < len(self.state.path):
            await self._follow_path()
        else:
            # Use obstacle avoidance for local navigation
            await self._obstacle_avoidance_navigation()
    
    async def _learning_navigation(self):
        """Learning-based navigation"""
        # Get recommended action from learning system
        recommended_action = self.learning_system.get_recommended_action(self.state)
        
        if recommended_action:
            await self._execute_action(recommended_action)
        else:
            # Fall back to autonomous navigation
            await self._autonomous_navigation()
    
    async def _plan_path(self):
        """Plan path to target"""
        if not self.state.target:
            return
        
        try:
            path = self.planner.plan_path(
                self.state.position,
                self.state.target,
                self.state.obstacles
            )
            
            if path:
                self.state.path = path
                self.path_index = 0
                logger.info(f"Path planned with {len(path)} waypoints")
            else:
                logger.warning("No path found to target")
                
        except Exception as e:
            logger.error(f"Error planning path: {e}")
    
    async def _follow_path(self):
        """Follow planned path"""
        if not self.state.path or self.path_index >= len(self.state.path):
            return
        
        target_point = self.state.path[self.path_index]
        distance_to_target = self.state.position.distance_to(target_point)
        
        if distance_to_target < 0.2:  # Reached waypoint
            self.path_index += 1
            return
        
        # Calculate direction to next waypoint
        angle_to_target = self.state.position.angle_to(target_point)
        angle_diff = self._normalize_angle(angle_to_target - self.state.heading)
        
        # Execute movement
        if abs(angle_diff) > 0.1:  # Need to turn
            if angle_diff > 0:
                await self._execute_action("turn_right")
            else:
                await self._execute_action("turn_left")
        else:  # Move forward
            await self._execute_action("forward")
    
    async def _obstacle_avoidance_navigation(self):
        """Navigate using obstacle avoidance"""
        if not self.state.target:
            return
        
        # Calculate force for obstacle avoidance
        force = self.obstacle_avoidance.calculate_force(
            self.state.position,
            self.state.target,
            self.state.obstacles
        )
        
        # Convert force to movement command
        if abs(force.x) > abs(force.y):
            if force.x > 0:
                await self._execute_action("turn_right")
            else:
                await self._execute_action("turn_left")
        else:
            if force.y > 0:
                await self._execute_action("forward")
            else:
                await self._execute_action("backward")
    
    async def _execute_action(self, action: str):
        """Execute navigation action"""
        try:
            # Record experience for learning
            old_state = NavigationState(
                position=Point(self.state.position.x, self.state.position.y),
                heading=self.state.heading,
                velocity=self.state.velocity,
                target=self.state.target,
                obstacles=self.state.obstacles.copy() if self.state.obstacles else []
            )
            
            # Execute action (this would interface with motor system)
            logger.info(f"Executing navigation action: {action}")
            
            # Calculate reward based on progress towards target
            reward = self._calculate_reward(action)
            
            # Record experience
            self.learning_system.record_experience(old_state, action, reward, self.state)
            
        except Exception as e:
            logger.error(f"Error executing action {action}: {e}")
    
    def _calculate_reward(self, action: str) -> float:
        """Calculate reward for action"""
        if not self.state.target:
            return 0.0
        
        # Simple reward: progress towards target
        distance_to_target = self.state.position.distance_to(self.state.target)
        
        if action == "forward":
            return 1.0 if distance_to_target < 1.0 else 0.5
        elif action in ["turn_left", "turn_right"]:
            return 0.1  # Small reward for turning
        else:
            return -0.1  # Penalty for backward movement
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    async def get_navigation_status(self) -> Dict[str, Any]:
        """Get current navigation status"""
        return {
            'mode': self.state.mode.value,
            'position': {'x': self.state.position.x, 'y': self.state.position.y},
            'heading': self.state.heading,
            'velocity': self.state.velocity,
            'target': {'x': self.state.target.x, 'y': self.state.target.y} if self.state.target else None,
            'path_length': len(self.state.path) if self.state.path else 0,
            'path_index': self.path_index,
            'obstacle_count': len(self.state.obstacles) if self.state.obstacles else 0,
            'learning_patterns': len(self.learning_system.success_patterns)
        }

# Global AI navigation system instance
ai_navigation = AINavigationSystem()
