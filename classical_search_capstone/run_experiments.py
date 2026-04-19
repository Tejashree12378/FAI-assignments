"""
Run classical search algorithms on sample grid maps.

Usage:
    python run_experiments.py

Optional: set PRINT_PATHS = True to print coordinates and a path overlay (S G # * .).
"""

from grid_environment import GridWorld, print_grid_with_path
from search_algorithms import (
    SearchResult,
    astar,
    bfs,
    greedy,
    inadmissible_h_fn,
    manhattan_h_fn,
    ucs,
    zero_h_fn,
)

# Set to True to print the path as a list of (row, col) cells after the table.
PRINT_PATHS = False

# Default runs for standard maps (open, obstacle, A* showcase).
_DEFAULT_ALGORITHMS = (
    ("BFS", bfs),
    ("UCS", ucs),
    ("A* (Manhattan)", lambda e: astar(e, manhattan_h_fn)),
    ("A* (h = 0)", lambda e: astar(e, zero_h_fn)),
    ("Greedy (Manhattan)", lambda e: greedy(e, manhattan_h_fn)),
)

# Extra inadmissible A* on the small trap map.
_INADMISSIBLE_ALGORITHMS = _DEFAULT_ALGORITHMS + (
    ("A* (inadmissible)", lambda e: astar(e, inadmissible_h_fn)),
)


def _map_open_maze():
    """Wide open grid: algorithms mainly differ in expansion order."""
    layout = [
        "..............",
        "..............",
        "..............",
        "..............",
        "..............",
    ]
    start = (2, 0)
    goal = (2, 13)
    return "Open corridor (5x14)", GridWorld(layout, start, goal)


def _map_obstacle_maze():
    """Narrow passages and dead ends: highlights search efficiency."""
    layout = [
        ".............",
        ".###.###.###.",
        "...#...#...#.",
        ".#.#.#.#.#.#.",
        "...#...#...#.",
        ".###.###.###.",
        ".............",
    ]
    start = (0, 0)
    goal = (6, 12)
    return "Obstacle maze with corridors", GridWorld(layout, start, goal)


def _map_astar_best():
    """
    Large chamber + narrow corridor: BFS/UCS flood the chamber; Greedy can be
    misled into local pockets; A* (Manhattan) stays focused and expands fewer nodes.
    Grid is 12x26 (>= 10x20).
    """
    layout = [
        ".#.#...#......############",
        "......#..#....############",
        "#.#....##..#.#############",
        "#....#......#.############",
        "...#....##..#.############",
        "...#...#..##..############",
        ".#....#.##................",
        "....###..#....############",
        "....#.....##..############",
        ".............#############",
        "......#.......############",
        "..........#...############",
    ]
    start = (11, 1)
    goal = (6, 25)
    return "A* showcase: chamber + corridor (12x26)", GridWorld(layout, start, goal)


def _map_inadmissible_test():
    """
    Small 6x8 trap: A* with h = 3 * Manhattan can return a longer path than BFS/UCS
    or A* with Manhattan, because the inflated heuristic changes expansion order.
    """
    layout = [
        "#....#..",
        "#......#",
        "........",
        ".#..#...",
        "........",
        "..##....",
    ]
    start = (5, 0)
    goal = (0, 7)
    return "Inadmissible-heuristic trap (6x8)", GridWorld(layout, start, goal)


def _format_result(name: str, res: SearchResult) -> str:
    mem_kb = res.peak_mem_bytes / 1024.0
    if not res.found:
        return (
            f"{name:<28}  {'no':>3}  {'—':>5}  {res.nodes_expanded:>7}  "
            f"{res.time_ms:>9.3f}  {mem_kb:>8.1f}"
        )
    return (
        f"{name:<28}  {'yes':>3}  {res.path_cost:5.0f}  {res.nodes_expanded:>7}  "
        f"{res.time_ms:>9.3f}  {mem_kb:>8.1f}"
    )


def _run_all_on_env(title: str, env: GridWorld, algorithms=_DEFAULT_ALGORITHMS):
    """Run selected algorithms and print a compact comparison."""
    print()
    print("=" * 86)
    print(title)
    print("=" * 86)
    print(env)
    print()

    runs = [(name, fn(env)) for name, fn in algorithms]

    hdr = (
        f"{'Algorithm':<28}  {'OK?':>3}  {'Cost':>5}  {'Expand':>7}  "
        f"{'ms':>9}  {'Mem(KB)':>8}"
    )
    print(hdr)
    print("-" * len(hdr))
    for name, res in runs:
        print(_format_result(name, res))

    print()
    print("Summary (path cost, expansions, peak memory):")
    found = [r for _, r in runs if r.found]
    if not found:
        print("  No algorithm found a path.")
    else:
        best_cost = min(r.path_cost for r in found)
        least_exp = min(r.nodes_expanded for r in found)
        least_mem = min(r.peak_mem_bytes for r in found)
        for name, res in runs:
            if res.found:
                same_cost = res.path_cost == best_cost
                fewest = res.nodes_expanded == least_exp
                low_mem = res.peak_mem_bytes == least_mem
                notes = []
                if same_cost:
                    notes.append("optimal cost (among runs)")
                if fewest:
                    notes.append("fewest expansions (tie possible)")
                if low_mem:
                    notes.append("lowest peak memory (tie possible)")
                extra = f" — {', '.join(notes)}" if notes else ""
                mem_kb = res.peak_mem_bytes / 1024.0
                print(
                    f"  {name:<26}  cost {res.path_cost:3.0f}   "
                    f"expansions {res.nodes_expanded:4d}   "
                    f"peak {mem_kb:6.1f} KB{extra}"
                )
            else:
                print(f"  {name:<26}  no path")

    if PRINT_PATHS:
        print()
        print("Paths (row, col) and grid overlay (S=start, G=goal, #=wall, *=path, .=free):")
        for name, res in runs:
            if res.found:
                print(f"  {name}: {res.path}")
                print_grid_with_path(env, res.path)
                print()
            else:
                print(f"  {name}: (none)")
                print()


def main():
    maps = [
        (_map_open_maze(), _DEFAULT_ALGORITHMS),
        (_map_obstacle_maze(), _DEFAULT_ALGORITHMS),
        (_map_astar_best(), _DEFAULT_ALGORITHMS),
        (_map_inadmissible_test(), _INADMISSIBLE_ALGORITHMS),
    ]
    print("Classical search — 4-neighbor grid, unit step cost")
    print("Expansions = times a cell is removed from the frontier and processed.")
    print("Mem(KB) = tracemalloc peak traced memory / 1024 for that search call.")
    print("Algorithms: BFS, UCS, A* (Manhattan), A* (h = 0), Greedy (Manhattan);")
    print("  plus A* (inadmissible) on the last map only.")
    for (title, env), algorithms in maps:
        _run_all_on_env(title, env, algorithms)
    print()
    print("Done.")


if __name__ == "__main__":
    main()
