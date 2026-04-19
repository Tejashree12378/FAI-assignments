"""
2D grid world for classical search demos.

Cells use (row, col) coordinates with row 0 at the top.
Movement is only up, down, left, right; each step has cost 1.
"""


class GridWorld:
    """
    A rectangular grid with passable cells, obstacles, a start, and a goal.

    The layout is given as a list of equal-length strings:
      '.' = free cell
      '#' = blocked obstacle
    Start and goal are explicit (row, col) tuples on free cells.
    """

    # Four directions: (delta_row, delta_col)
    _DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))

    def __init__(self, layout, start, goal):
        """
        layout: list of strings, one per row, same width.
        start, goal: (row, col) on passable cells.
        """
        if not layout:
            raise ValueError("layout must be non-empty")
        width = len(layout[0])
        if any(len(row) != width for row in layout):
            raise ValueError("all layout rows must have the same length")

        self._layout = [list(row) for row in layout]
        self._rows = len(self._layout)
        self._cols = width
        self.start = tuple(start)
        self.goal = tuple(goal)

        if not self._in_bounds(self.start) or not self._in_bounds(self.goal):
            raise ValueError("start and goal must be inside the grid")
        if self.is_blocked(self.start) or self.is_blocked(self.goal):
            raise ValueError("start and goal must not be on obstacles")

    # Helper function to check boundary
    def _in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self._rows and 0 <= c < self._cols

    # Helper function to check obstacles
    def is_blocked(self, cell):
        """True if cell is outside the grid or contains '#'."""
        if not self._in_bounds(cell):
            return True
        r, c = cell
        return self._layout[r][c] == "#"

    # returns valid and non obstacle cells in four directions
    def neighbors(self, cell):
        """
        Return list of (next_cell, step_cost) for legal moves from cell.
        Each legal move has step_cost 1.
        """
        r, c = cell
        out = []
        for dr, dc in self._DIRS:
            nxt = (r + dr, c + dc)
            if not self.is_blocked(nxt):
                out.append((nxt, 1))
        return out

    def manhattan_heuristic(self, cell):
        """Manhattan distance from cell to the goal (admissible with unit costs)."""
        r, c = cell
        gr, gc = self.goal
        return abs(r - gr) + abs(c - gc)

    def zero_heuristic(self, cell):
        """Trivial heuristic h=0 (A* becomes UCS-like ordering by g only)."""
        return 0

    def inadmissible_heuristic(self, cell):
        """
        Overestimates remaining cost (3 * Manhattan). Not admissible; A* may return
        a suboptimal path when guided by this heuristic.
        """
        return 3 * self.manhattan_heuristic(cell)

    def is_goal(self, cell):
        return cell == self.goal

    def __str__(self):
        lines = []
        for r in range(self._rows):
            line = []
            for c in range(self._cols):
                ch = self._layout[r][c]
                if (r, c) == self.start and ch == ".":
                    line.append("S")
                elif (r, c) == self.goal and ch == ".":
                    line.append("G")
                else:
                    line.append(ch)
            lines.append("".join(line))
        return "\n".join(lines)


def print_grid_with_path(env, path):
    """
    Print the grid to the terminal with a discovered path overlaid.

    Legend: S=start, G=goal, #=obstacle, *=path cell, .=empty free cell.
    Start and goal keep S/G even when they lie on the path; obstacles stay #.
    """
    on_path = set(path) if path else set()
    for r in range(env._rows):
        line = []
        for c in range(env._cols):
            cell = (r, c)
            if env.is_blocked(cell):
                line.append("#")
            elif cell == env.start:
                line.append("S")
            elif cell == env.goal:
                line.append("G")
            elif cell in on_path:
                line.append("*")
            else:
                line.append(".")
        print("".join(line))
