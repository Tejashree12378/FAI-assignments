Classical search capstone (BFS, UCS, A*, Greedy)

What this is
  Small demo project for classical graph search on 2D grids:
  - 4-direction movement (up/down/left/right)
  - unit step costs (default = 1)
  - start, goal, and blocked obstacle cells

  Implemented algorithms:
  - Breadth-First Search (BFS)
  - Uniform-Cost Search (UCS)
  - A* Search
  - Greedy Best-First Search

  Heuristics:
  - Manhattan distance (admissible on this grid setup)
  - Zero heuristic (A* behaves like UCS)
  - Inadmissible heuristic = 3 * Manhattan (can produce suboptimal A*)

How to run
  From this folder:

    python3 run_experiments.py

  Requirements: Python 3 only (standard library). No pip install.

Files
  grid_environment.py
    GridWorld class, legal neighbors, and heuristic functions:
    Manhattan, zero, and inadmissible.

  search_algorithms.py
    BFS, UCS, A*, Greedy; shared path reconstruction; SearchResult object.
    Every algorithm reports:
    - found / not found
    - path
    - path cost
    - nodes expanded
    - runtime in milliseconds
    - peak memory (bytes) via tracemalloc

  run_experiments.py
    Defines multiple maps and prints comparison tables including:
    cost, expansions, runtime, and memory.

Maps and experiments
  1) Open corridor (5x14)
     Shows baseline behavior in an open map.

  2) Obstacle maze with corridors
     Shows behavior in structured passages.

  3) A* showcase: chamber + corridor (12x26)
     A* with Manhattan expands significantly fewer nodes than BFS/UCS,
     and fewer than Greedy on this map.

  4) Inadmissible-heuristic trap (6x8)
     Includes A* with inadmissible heuristic and demonstrates suboptimality:
     A*(inadmissible) can return a higher path cost than BFS/UCS/A*(Manhattan).

Output columns
  Algorithm  OK?  Cost  Expand  ms  Mem(KB)

  Notes:
  - Expand: number of states removed from frontier and processed.
  - ms: wall-clock runtime in milliseconds.
  - Mem(KB): peak traced memory for that search call (tracemalloc).

Optional
  In run_experiments.py, set PRINT_PATHS = True to print each path as (row, col)
  coordinates and call print_grid_with_path (from grid_environment.py) to show:
    S = start
    G = goal
    # = obstacle
    * = discovered path
    . = empty cell
