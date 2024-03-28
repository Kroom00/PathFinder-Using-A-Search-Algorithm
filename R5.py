import random
import math
# Define the grid size
GRID_SIZE = 100

# Define the range of obstacle percentages to test
OBSTACLE_PERCENTAGES = range(10, 100, 10)

# Define the number of random tests to run for each obstacle percentage
NUM_TESTS = 10

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

        # If the selected node is the goal node,# then the path has been found
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

# Generate random grid maps with varying percentages of obstacles and run the A* search algorithm on them
for obstacle_percentage in OBSTACLE_PERCENTAGES:
    print("Testing with {}% obstacles:".format(obstacle_percentage))
    for test_num in range(NUM_TESTS):
        # Generate the grid map with the specified obstacle percentage
        grid_map = [[0 for x in range(GRID_SIZE)] for y in range(GRID_SIZE)]
        num_obstacles = int(GRID_SIZE * GRID_SIZE * obstacle_percentage / 100)
        for i in range(num_obstacles):
            while True:
                x = random.randint(0, GRID_SIZE - 1)
                y = random.randint(0, GRID_SIZE - 1)
                if grid_map[x][y] == 0:
                    grid_map[x][y] = 1
                    break

        # Run the A* search algorithm from a random start node to a random goal node
        start = (random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1))
        while grid_map[start[0]][start[1]] == 1:
            start = (random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1))
        goal = (random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1))
        while grid_map[goal[0]][goal[1]] == 1:
            goal = (random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1))

        path, cost = astar(start, goal, grid_map)

        # Print the result of the test
        if path:
            print("Test {}: Path found with cost {}".format(test_num+1, cost))
        else:
            print("Test {}: No path found".format(test_num+1))