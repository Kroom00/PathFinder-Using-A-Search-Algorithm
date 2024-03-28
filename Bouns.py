import math
import tkinter as tk
from queue import PriorityQueue

root = tk.Tk()
root.title("Grid")

# Create 10x10 grid
cells = {}
for row in range(10):
    for col in range(10):
        cell = tk.Frame(root, bg="white", highlightbackground="black", highlightthickness=1, width=50, height=50)

        cell.grid(row=row, column=col)
        cells[(row, col)] = cell

# Place start state at (0,0). PS we put (9.0) because the map is upside down
start = (9, 0)
start_label = tk.Label(root, text="S", bg="green",width=6,height=3)
start_label.grid(row=start[0], column=start[1])

# Place goal state at (6,9) PS we put (0,9) because the map is upside down
goal = (0, 6)
goal_label = tk.Label(root, text="G", bg="red",width=6,height=3)
goal_label.grid(row=goal[0], column=goal[1])

# Place obstacles. PS we changed the coordinate so the map display correctly
obstacles =  [(1, 0), (1, 1), (1, 2), (1, 3), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (7, 2),
             (7, 1), (7, 3), (7, 4), (7, 5), (7, 7),
             (6, 7), (5, 6), (5, 7), (3, 5), (3, 6),
             (3, 8), (2, 8),(1,8)]
for obstacle in obstacles:
    obs = tk.Label(root, bg="black",width=6,height=3)
    obs.grid(row=obstacle[0], column=obstacle[1])

# A* algorithm
def heuristic(node, goal):
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)


def a_star(start, goal, obstacles):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {start: None}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while not frontier.empty():
        current = frontier.get()[1]

        if current == goal:
            break

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                next = (current[0] + dx, current[1] + dy)
                if next[0] < 0 or next[0] > 9 or next[1] < 0 or next[1] > 9 or next in obstacles:
                    continue
                new_cost = g_score[current] + ((dx == 0 or dy == 0) and 1 or math.sqrt(2))
                if next not in g_score or new_cost < g_score[next]:
                    g_score[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current

    return came_from

# Find path from start to goal
came_from = a_star(start, goal, obstacles)
current = goal
while current != start:
    cell = cells[current]
    cell.configure(bg="blue")
    current = came_from[current]

root.mainloop()