# Program-1-Search-Methods/Route Finder

This program finds a route between two cities using different search algorithms. It allows users to choose from multiple methods to find the shortest path in a small network of cities.

## How to Run

1. Clone the repository.
2. Place the `Adjacencies.txt` and `coordinates.csv` files in the same directory.
3. Compile the code using `javac RouteFinder.java`.
4. Run the program with `java RouteFinder`.

## Implemented Algorithms

- **Brute-Force**: BFS is used as a simple approach to finding a path.
- **BFS**: Finds the shortest path by exploring all nodes level by level.
- **DFS**: Explores paths depth-first to find a route.
- **ID-DFS**: Iterative deepening of DFS to avoid deep recursion.
- **Best-First Search**: Uses city coordinates (lat, lon) as a heuristic for pathfinding.
- **A* Search**: Combines BFS with a heuristic (distance) to find the optimal path.

## Input & Output

**Input**:
- Start and destination cities.
- Select a search method (Brute-Force, BFS, DFS, ID-DFS, Best-First, or A*).

**Output**:
- The route from the start city to the destination city.
- Total distance (in km) for the route found.
- Execution time of the search.

