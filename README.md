# PathFinder Using A* Search Algorithm

---

## Description

Path Planning involves finding an optimal path for a robot from a starting position to a destination while navigating around obstacles. The A* search algorithm is utilized to solve this problem by intelligently exploring the search space based on both the cost incurred so far and an estimated cost to reach the goal. The project tasks include:

- Formulating the search problem with the initial state, goal state, and successor function.
- Designing appropriate data structures to represent states and facilitate efficient search.
- Implementing the A* search algorithm in Python.
- Testing the algorithm on randomly generated test problems with varying obstacle configurations.
- Analyzing and interpreting the results to assess the algorithm's performance.

---

## Step 1: Problem Formulation

- **Initial State:** Starting position denoted by 'S'
- **Goal State:** Destination position denoted by 'G'
- **Successor Function:** Movement allowed in horizontal, vertical, and diagonal directions.
- **Cost:**
  - Horizontal and vertical movement cost: 1 per move.
  - Diagonal movement cost: √2 per move.

---

## Step 2: Data Structure

- Each state represented as a tuple (x, y) for coordinates.
- Open set and closed set implemented as sets of nodes.
- G-score and F-score dictionaries for node scores.
- Parents dictionary maps each node to its parent.

---

## Step 3: List of Fringes


| Node  | Cost from Start | Cost to Goal |
|-------|-----------------|--------------|
| (0,1) | 1               | 13.656       |
| (1,0) | 1               | 14.485       |
| (0,2) | 2               | 15.485       |
| (2,0) | 2               | 13.485       |
| (3,0) | 3               | 12.485       |
| (4,0) | 4               | 11.485       |
| (5,0) | 5               | 10.485       |
| (6,0) | 6               | 10.656       |
| (6,1) | 6.414           | 9.656        |
| (7,1) | 7.414           | 10.071       |
| (5,3) | 8.242           | 7.242        |
| (4,3) | 9.242           | 6.828        |
| (4,4) | 9.656           | 5.828        |
| (4,5) | 10.656          | 4.828        |
| (6,5) | 10.656          | 4.828        |
| (3,5) | 11.656          | 5.242        |
| (3,6) | 12.070          | 4.242        |
| (3,7) | 13.070          | 3.828        |
| (4,7) | 12.656          | 2.828        |
| (4,8) | 13.656          | 2.414        |
| (6,7) | 14.071          | 2            |
| (6,8) | 14.485          | 1            |
| (4,9) | 14.656          | 2            |
| (5,9) | 15.071          | 1            |

The nodes are sorted based on priority, with the highest priority node at the top. The algorithm dequeues nodes from the priority queue, expands them, and adds neighbors based on estimated cost-to-go and cost-so-far.

---

## Step 7: Time Complexity Analysis

- **Worst Case:** O(b^d), where b is branching factor and d is depth of optimal solution.
- **Best Case:** O(1), when start and goal nodes are close with few obstacles.
- **Average Case:** O(b^d), typically lower than worst case due to heuristics and optimizations.

### Table:

| Performance Measure | Big O Complexity |
|---------------------|------------------|
| Worst Case Time     | O(b^d)           |
| Best Case Time      | O(1)             |
| Average Case Time   | O(b^d)           |
| Space Complexity    | O(b^d)           |


---
