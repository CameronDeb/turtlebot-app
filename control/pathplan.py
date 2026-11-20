"""Route planning around obstacles for the object search, from one lidar scan (no map needed).

The robot's 2D lidar sees the legs of chairs and tables but not their low bases (chair sleds, star feet, cables),
and it sees *through* the gaps between legs. So the planner keeps well away from everything the lidar sees:

  KEEP_OUT  0.30 m  no route may pass closer than this to a lidar return (robot radius ~0.19 m + 0.11 m). A gap
                    between two legs narrower than ~0.6 m is therefore closed, which keeps the robot from trying to
                    squeeze under furniture.
  PREFER    0.50 m  routes closer than this cost extra, so the planner prefers open floor when it has the choice.

A* on a 5 cm grid around the robot and the target finds the cheapest route to any free spot at the requested
distance from the object; the route is then shortened to a few straight legs (line of sight on the keep-out map).
Space the lidar hasn't seen counts as free; the route is re-planned from a fresh scan after every leg, so new
obstacles are picked up as they come into view.
"""

import heapq
import math

CELL = 0.05
KEEP_OUT = 0.30
PREFER = 0.50
NEAR_COST = 4.0  # extra cost per cell at the keep-out edge, falling to 0 at PREFER
MARGIN = 2.0  # grid extends this far beyond the robot and the target (m)
MAX_SIZE = 14.0  # m, grid side limit
GOAL_TOL = 0.06  # m: goal ring half-width
START_ESCAPE = 0.30  # m: the robot may start inside the keep-out zone (it just approached something)


class Grid:
    def __init__(self, x0, y0, nx, ny):
        self.x0, self.y0, self.nx, self.ny = x0, y0, nx, ny
        self.clear = [[math.inf] * ny for _ in range(nx)]  # distance to the nearest lidar return (up to PREFER)

    def cell(self, x, y):
        return int((x - self.x0) / CELL), int((y - self.y0) / CELL)

    def centre(self, i, j):
        return self.x0 + (i + 0.5) * CELL, self.y0 + (j + 0.5) * CELL

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def free(self, i, j):
        return self.inside(i, j) and self.clear[i][j] > KEEP_OUT


def build_grid(start, target, points):
    lo_x = min(start[0], target[0]) - MARGIN
    lo_y = min(start[1], target[1]) - MARGIN
    hi_x = max(start[0], target[0]) + MARGIN
    hi_y = max(start[1], target[1]) + MARGIN
    if hi_x - lo_x > MAX_SIZE or hi_y - lo_y > MAX_SIZE:
        return None
    g = Grid(lo_x, lo_y, int((hi_x - lo_x) / CELL) + 1, int((hi_y - lo_y) / CELL) + 1)
    r = int(math.ceil(PREFER / CELL))
    offsets = [(di, dj, math.hypot(di, dj) * CELL) for di in range(-r, r + 1) for dj in range(-r, r + 1)]
    offsets = [o for o in offsets if o[2] <= PREFER]
    seen = set()
    for px, py in points:
        ci, cj = g.cell(px, py)
        if (ci, cj) in seen or not (-r <= ci < g.nx + r and -r <= cj < g.ny + r):
            continue
        seen.add((ci, cj))
        for di, dj, d in offsets:
            i, j = ci + di, cj + dj
            if 0 <= i < g.nx and 0 <= j < g.ny and d < g.clear[i][j]:
                g.clear[i][j] = d
    return g


def _step_cost(g, i, j, base):
    c = g.clear[i][j]
    if c >= PREFER:
        return base
    return base * (1.0 + NEAR_COST * (PREFER - c) / (PREFER - KEEP_OUT))


def plan(start, target, goal_radius, points):
    """Route from start (x, y) to a free spot goal_radius from target (x, y), avoiding lidar points.

    Returns a list of waypoints [(x, y), ...] ending at the goal spot (start excluded), [] if already there,
    or None if no clear route exists."""
    if math.dist(start, target) <= goal_radius + GOAL_TOL:
        return []
    g = build_grid(start, target, points)
    if g is None:
        return None
    si, sj = g.cell(*start)
    if not g.inside(si, sj):
        return None

    def passable(i, j):
        if g.free(i, j):
            return True
        # leaving the keep-out zone right after an approach is allowed, but only near the start
        return g.inside(i, j) and math.dist(g.centre(i, j), start) <= START_ESCAPE

    def is_goal(i, j):
        return abs(math.dist(g.centre(i, j), target) - goal_radius) <= GOAL_TOL and g.free(i, j)

    def h(i, j):
        return max(0.0, math.dist(g.centre(i, j), target) - goal_radius)

    moves = [(1, 0, CELL), (-1, 0, CELL), (0, 1, CELL), (0, -1, CELL)] + [
        (di, dj, CELL * math.sqrt(2)) for di in (-1, 1) for dj in (-1, 1)
    ]
    best = {(si, sj): 0.0}
    came = {}
    heap = [(h(si, sj), 0.0, si, sj)]
    end = None
    while heap:
        _, cost, i, j = heapq.heappop(heap)
        if cost > best.get((i, j), math.inf):
            continue
        if is_goal(i, j):
            end = (i, j)
            break
        for di, dj, base in moves:
            ni, nj = i + di, j + dj
            if not passable(ni, nj):
                continue
            if di and dj and not (passable(i + di, j) and passable(i, j + dj)):
                continue  # no corner cutting
            nc = cost + _step_cost(g, ni, nj, base)
            if nc < best.get((ni, nj), math.inf):
                best[(ni, nj)] = nc
                came[(ni, nj)] = (i, j)
                heapq.heappush(heap, (nc + h(ni, nj), nc, ni, nj))
    if end is None:
        return None
    cells = [end]
    while cells[-1] in came:
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = [g.centre(i, j) for i, j in cells]
    pts[0] = start
    return _shorten(g, pts, passable)


def _line_ok(g, a, b, passable):
    n = max(1, int(math.dist(a, b) / (CELL / 2)))
    for k in range(n + 1):
        x = a[0] + (b[0] - a[0]) * k / n
        y = a[1] + (b[1] - a[1]) * k / n
        if not passable(*g.cell(x, y)):
            return False
    return True


def _shorten(g, pts, passable):
    """Greedy line-of-sight shortcutting: the fewest straight legs that stay out of the keep-out zone."""
    out, k = [], 0
    while k < len(pts) - 1:
        nxt = k + 1
        for m in range(len(pts) - 1, k, -1):
            if _line_ok(g, pts[k], pts[m], passable):
                nxt = m
                break
        out.append(pts[nxt])
        k = nxt
    return out


def path_length(start, waypoints):
    total, prev = 0.0, start
    for w in waypoints:
        total += math.dist(prev, w)
        prev = w
    return total
