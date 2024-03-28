import math

# Define the grid map and obstacles
grid_map = [[0 for x in range(10)] for y in range(10)]
obstacles = [(0, 8), (1, 8), (2, 8), (3, 8), (8, 8), (2, 7), (8, 7), (2, 6), (5, 6), (6, 6),
             (8, 6), (2, 5), (2, 4), (6, 4), (7, 4),
             (2, 3), (7, 3), (2, 2), (3, 2), (4, 2),
             (5, 2), (7, 2)]

for obstacle in obstacles:
    grid_map[obstacle[0]][obstacle[1]] = 1

# Define the heuristic function (Euclidean distance)
def heuristic(node, goal):
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

# Define the A* search algorithm
def astar(start, goal, gridmap):
    # Initialize the open and closed sets
    open_set = set([start])
    closed_set = set()

    # Define the g-score and f-score dictionaries
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    # Define the dictionary to keep track of the parents of each node
    parents = {}

    # Repeat until the goal node is reached or the open set is empty
    while open_set:
        # Select the node with the lowest f-score from the open set
        current = min(open_set, key=lambda x: f_score[x])

        # If the selected node is the goal node, then the path has been found
        if current == goal:
            path = []
            cost = g_score[current]
            while current in parents:
                path.append(current)
                current = parents[current]
            path.append(start)
            path.reverse()
            return path, cost

        # Remove the current node from the open set and add it to the closed set
        open_set.remove(current)
        closed_set.add(current)

        # Generate the neighboring nodes and calculate their g-scores and f-scores
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                x = current[0] + dx
                y = current[1]+ dy
                if x < 0 or x >= len(gridmap) or y < 0 or y >= len(gridmap[0]) or gridmap[x][y] == 1:
                    continue
                neighbor = (x, y)
                tentative_g_score = g_score[current] + ((dx == 0 or dy == 0) and 1 or math.sqrt(2))
                if neighbor in closed_set and tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                if tentative_g_score < g_score.get(neighbor, float('inf')) or neighbor not in open_set:
                    parents[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

    # If the open set is empty and the goal state has not been reached, then there is no path from the start state to the goal state
    return None, float('inf')

# Test the A* search algorithm
start = (0, 0)
goal = (6, 9)
path, cost = astar(start, goal, grid_map)
if path:
    print("Path:", path)
    print("Cost:", cost)
else:
    print("No path found")
    