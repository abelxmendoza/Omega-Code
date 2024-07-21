import heapq

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

# Example usage
if __name__ == "__main__":
    grid = [[0, 1, 0, 0, 0],
            [0, 1, 0, 1, 0],
            [0, 0, 0, 1, 0],
            [1, 1, 0, 1, 0],
            [0, 0, 0, 0, 0]]
    
    start = (0, 0)
    goal = (4, 4)
    path = a_star_search(start, goal, grid)
    print("Path found by A* algorithm:", path)
