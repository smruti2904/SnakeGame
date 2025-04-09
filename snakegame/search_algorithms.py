import random
from collections import deque
import heapq

# Directions (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Random movement algorithm (limited moves)
def random_move(start, goal, obstacles, rows, cols):
    path = []
    current = start

    for _ in range(1000):  # Limit to 1000 moves
        direction = random.choice(DIRECTIONS)
        new_pos = (current[0] + direction[0], current[1] + direction[1])

        if (
            0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and
            new_pos not in obstacles
        ):
            path.append(direction)
            current = new_pos

        if current == goal:
            return path

    return []  # If it takes too long, return empty path


# Write your code below this only
# Breadth First Search (BFS) Algorithm
def bfs(start, goal, obstacles, rows, cols):
    queue = deque([(start, [])]) 
    visited = set([start])

    while queue:
        pos, path = queue.popleft()
        if pos == goal:
            return path  

        for x , y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                visited.add(new)
                queue.append((new, path + [(x, y)]))  
    return []  

# Depth First Search (DFS) Algorithm
def dfs(start, goal, obstacles, rows, cols):
    stack = [(start, [])]  
    visited = set([start])

    while stack:
        pos, path = stack.pop()
        if pos == goal:
            return path  

        for x, y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                visited.add(new)
                stack.append((new, path + [(x, y)]))  
    return []  

# Iterative Deepening Search (IDS Algorithm)
def ids(start, goal, obstacles, rows, cols):
    def dls(pos, depth, path, visited):
        if pos == goal:
            return path  
        if depth == 0:
            return None  

        visited.add(pos) 

        for x, y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                result = dls(new, depth - 1, path + [(x, y)], visited)
                if result is not None:
                    return result  
                
        return None  

    depth = 0
    while depth < (rows * cols):  
        visited = set()
        path = dls(start, depth, [], visited)
        if path is not None:
            return path  
        
        depth += 1

    return []

# Uniform Cost Search (UCS) Algorithm
def ucs(start, goal, obstacles, rows, cols):
    heap = [(0, start, [])]  
    visited = set()

    while heap:
        cost, pos, path = heapq.heappop(heap)

        if pos in visited:
            continue
        visited.add(pos)

        if pos == goal:
            return path  

        for x, y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                heapq.heappush(heap, (cost + 1, new, path + [(x, y)]))  

    return []

# Custom heuristic function (Manhattan Distance)
def heuristic(pos, goal):
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])  

# Greedy Best First Search Algorithm
def greedy_bfs(start, goal, obstacles, rows, cols):
    queue = [(heuristic(start, goal), start, [])]  
    visited = set()

    while queue:
        priority_value, pos, path = heapq.heappop(queue)

        if pos == goal:
            return path  

        if pos in visited:
            continue
        visited.add(pos)

        for x, y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                heapq.heappush(queue, (heuristic(new, goal), new, path + [(x, y)]))

    return[]
 
# A* Search Algorithm
def astar(start, goal, obstacles, rows, cols):
    heap = [(0, start, [])]  
    visited = set()
    actual_cost = {start: 0}  

    while heap:
        cost, pos, path = heapq.heappop(heap)

        if pos in visited:
            continue
        visited.add(pos)

        if pos == goal:
            return path  

        for x, y in DIRECTIONS:  
            new = (pos[0] + x, pos[1] + y)

            if (0 <= new[0] < rows and 0 <= new[1] < cols and new not in obstacles and new not in visited):
                gx = actual_cost[pos] + 1  
                hx = heuristic(new, goal)  
                fx = gx + hx  

                if new not in actual_cost or gx < actual_cost[new]:
                    actual_cost[new] = gx
                    heapq.heappush(heap, (fx, new, path + [(x, y)]))

    return []