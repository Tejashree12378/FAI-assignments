"""
Breadth-First Search, Uniform-Cost Search, A*, and Greedy Best-First on a GridWorld.

Each search returns a small result object with path, cost, expansions, time, and peak memory.
Path reconstruction uses a parent map recorded while searching.

Node expansions (consistent for BFS, UCS, A*, Greedy):
  Count +1 when a state is removed from the frontier and processed (queue popleft
  or heap pop after discarding stale UCS/A* entries). The start state counts on
  its first removal; goal counts if it is removed before returning.

Peak memory: tracemalloc peak since tracemalloc.start() for the search call
  (bytes), via tracemalloc.get_traced_memory()[1].
"""

import heapq
import time
import tracemalloc
from collections import deque


def _peak_mem_bytes():
    """Peak traced memory since tracemalloc.start() (bytes)."""
    return tracemalloc.get_traced_memory()[1]


class SearchResult:
    """Outcome of a search run (easy to print and compare)."""

    __slots__ = ("found", "path", "path_cost", "nodes_expanded", "time_ms", "peak_mem_bytes")

    def __init__(self, found, path, path_cost, nodes_expanded, time_ms, peak_mem_bytes):
        self.found = found
        self.path = path
        self.path_cost = path_cost
        self.nodes_expanded = nodes_expanded
        self.time_ms = time_ms
        self.peak_mem_bytes = peak_mem_bytes


def _path_cost_from_path(env, path):
    """Sum step costs along path (0 if len < 2)."""
    if len(path) < 2:
        return 0
    total = 0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        for nxt, c in env.neighbors(a):
            if nxt == b:
                total += c
                break
        else:
            raise ValueError("invalid path edge")
    return total


def bfs(env):
    """
    Breadth-first search: frontier is a FIFO queue.
    With unit edge costs, the first time we reach the goal we have a shortest path
    in number of steps (and in total cost).
    """
    tracemalloc.start()
    t0 = time.perf_counter()
    start = env.start
    goal = env.goal

    if start == goal:
        elapsed = (time.perf_counter() - t0) * 1000.0
        peak = _peak_mem_bytes()
        tracemalloc.stop()
        return SearchResult(True, [start], 0, 0, elapsed, peak)

    frontier = deque([start])
    came_from = {start: None}
    nodes_expanded = 0

    while frontier:
        current = frontier.popleft()
        nodes_expanded += 1

        if env.is_goal(current):
            path = _reconstruct_path_from_start(came_from, start, current)
            cost = _path_cost_from_path(env, path)
            elapsed = (time.perf_counter() - t0) * 1000.0
            peak = _peak_mem_bytes()
            tracemalloc.stop()
            return SearchResult(True, path, cost, nodes_expanded, elapsed, peak)

        for nxt, _step_cost in env.neighbors(current):
            if nxt not in came_from:
                came_from[nxt] = current
                frontier.append(nxt)

    elapsed = (time.perf_counter() - t0) * 1000.0
    peak = _peak_mem_bytes()
    tracemalloc.stop()
    return SearchResult(False, [], 0, nodes_expanded, elapsed, peak)


def _reconstruct_path_from_start(came_from, start, goal):
    """Walk parent pointers from goal back to start; came_from[start] is None."""
    if goal == start:
        return [start]
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    assert path[0] == start and path[-1] == goal, "path reconstruction failed"
    return path


def ucs(env):
    """
    Uniform-cost search: expand the node with smallest g (path cost so far).
    Optimal for nonnegative costs; with all costs 1, behaves like Dijkstra on an unweighted graph.
    """
    tracemalloc.start()
    t0 = time.perf_counter()
    start = env.start
    goal = env.goal

    if start == goal:
        elapsed = (time.perf_counter() - t0) * 1000.0
        peak = _peak_mem_bytes()
        tracemalloc.stop()
        return SearchResult(True, [start], 0, 0, elapsed, peak)

    tie = 0
    heap = [(0, tie, start)]
    tie += 1
    best_g = {start: 0}
    came_from = {start: None}
    nodes_expanded = 0

    while heap:
        g, _, current = heapq.heappop(heap)

        if g > best_g.get(current, float("inf")):
            continue

        nodes_expanded += 1

        if env.is_goal(current):
            path = _reconstruct_path_from_start(came_from, start, current)
            elapsed = (time.perf_counter() - t0) * 1000.0
            peak = _peak_mem_bytes()
            tracemalloc.stop()
            return SearchResult(True, path, g, nodes_expanded, elapsed, peak)

        for nxt, step_cost in env.neighbors(current):
            new_g = g + step_cost
            if new_g < best_g.get(nxt, float("inf")):
                best_g[nxt] = new_g
                came_from[nxt] = current
                heapq.heappush(heap, (new_g, tie, nxt))
                tie += 1

    elapsed = (time.perf_counter() - t0) * 1000.0
    peak = _peak_mem_bytes()
    tracemalloc.stop()
    return SearchResult(False, [], 0, nodes_expanded, elapsed, peak)


def astar(env, heuristic_fn):
    """
    A* search: f = g + h(state). heuristic_fn(env, cell) must be nonnegative.

    With an admissible heuristic (e.g. Manhattan on this grid), the first goal
    dequeued at its final g is optimal.
    """
    tracemalloc.start()
    t0 = time.perf_counter()
    start = env.start
    goal = env.goal

    def h(cell):
        return heuristic_fn(env, cell)

    if start == goal:
        elapsed = (time.perf_counter() - t0) * 1000.0
        peak = _peak_mem_bytes()
        tracemalloc.stop()
        return SearchResult(True, [start], 0, 0, elapsed, peak)

    tie = 0
    start_g = 0
    heap = [(start_g + h(start), tie, start_g, start)]
    tie += 1
    best_g = {start: 0}
    came_from = {start: None}
    nodes_expanded = 0

    while heap:
        _f, _, g, current = heapq.heappop(heap)

        if g > best_g.get(current, float("inf")):
            continue

        nodes_expanded += 1

        if env.is_goal(current):
            path = _reconstruct_path_from_start(came_from, start, current)
            elapsed = (time.perf_counter() - t0) * 1000.0
            peak = _peak_mem_bytes()
            tracemalloc.stop()
            return SearchResult(True, path, g, nodes_expanded, elapsed, peak)

        for nxt, step_cost in env.neighbors(current):
            new_g = g + step_cost
            if new_g < best_g.get(nxt, float("inf")):
                best_g[nxt] = new_g
                came_from[nxt] = current
                f = new_g + h(nxt)
                heapq.heappush(heap, (f, tie, new_g, nxt))
                tie += 1

    elapsed = (time.perf_counter() - t0) * 1000.0
    peak = _peak_mem_bytes()
    tracemalloc.stop()
    return SearchResult(False, [], 0, nodes_expanded, elapsed, peak)


def greedy(env, heuristic_fn):
    """
    Greedy best-first search: priority f(n) = h(n) only (ignore g).

    Not guaranteed optimal. Uses expanded set to avoid re-expanding the same state.
    """
    tracemalloc.start()
    t0 = time.perf_counter()
    start = env.start
    goal = env.goal

    def h(cell):
        return heuristic_fn(env, cell)

    if start == goal:
        elapsed = (time.perf_counter() - t0) * 1000.0
        peak = _peak_mem_bytes()
        tracemalloc.stop()
        return SearchResult(True, [start], 0, 0, elapsed, peak)

    tie = 0
    heap = [(h(start), tie, start)]
    tie += 1
    came_from = {start: None}
    expanded = set()
    nodes_expanded = 0

    while heap:
        _hval, _, current = heapq.heappop(heap)

        if current in expanded:
            continue
        expanded.add(current)
        nodes_expanded += 1

        if env.is_goal(current):
            path = _reconstruct_path_from_start(came_from, start, current)
            cost = _path_cost_from_path(env, path)
            elapsed = (time.perf_counter() - t0) * 1000.0
            peak = _peak_mem_bytes()
            tracemalloc.stop()
            return SearchResult(True, path, cost, nodes_expanded, elapsed, peak)

        for nxt, _step_cost in env.neighbors(current):
            if nxt not in came_from:
                came_from[nxt] = current
                heapq.heappush(heap, (h(nxt), tie, nxt))
                tie += 1

    elapsed = (time.perf_counter() - t0) * 1000.0
    peak = _peak_mem_bytes()
    tracemalloc.stop()
    return SearchResult(False, [], 0, nodes_expanded, elapsed, peak)


def manhattan_h_fn(env, cell):
    """Adapter so A* can call heuristic as heuristic_fn(env, cell)."""
    return env.manhattan_heuristic(cell)


def zero_h_fn(env, cell):
    return env.zero_heuristic(cell)


def inadmissible_h_fn(env, cell):
    return env.inadmissible_heuristic(cell)
