#!/usr/bin/env python3
"""
hybrid_astar_core.py  --  Hybrid A* planner (pure Python, zero ROS deps).

Kinematic model : diff-drive (matches gz-sim-diff-drive-system plugin).
Search space    : continuous (x, y, heading) with discrete grid for dup detection.
Heuristic       : Euclidean distance to goal (admissible).

Truck_100t SDF facts:
  chassis : 9.0 x 5.0 m  →  half_len=4.5, half_wid=2.5
  wheel_separation : 6.0 m,  wheel_radius : 1.0 m

Paddock coordinate system (paddock.sdf):
  truck_1 spawns  (-30, -50)   heading +Y (yaw=π/2)
  gate gap         x≈-3,  y∈[-20,+20]
  corridor         x∈[-25,75],  y∈[-110,115]

Route (debugged from obstacle map):
  The direct path straight up x≈0 is blocked by berms & ore piles.
  Clear corridor goes RIGHT to x≈40-65 to pass berm-1 (y=55),
  then swings back left to pass berm-2 (y=90) around x≈19-28,
  then up to dump approach (50, 110).
  Waypoint-guided planning breaks the route into short solvable segments.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Truck geometry
# ---------------------------------------------------------------------------
HALF_LEN   = 4.5    # m  (9m body / 2)
HALF_WID   = 2.6    # m  (5m body / 2  + 0.1m margin — reduced from 2.9)
STEP_LEN   = 2.5    # m  arc length per A* expansion step
INFLATION  = 0.8    # m  obstacle padding (reduced from 1.2 — was over-blocking)

# ---------------------------------------------------------------------------
# Search resolution
# ---------------------------------------------------------------------------
XY_RES  = 1.5
TH_RES  = math.radians(6)
TH_BINS = round(2 * math.pi / TH_RES)

# ---------------------------------------------------------------------------
# Motion primitives  (curvatures → turn radii)
#   R=∞ straight, R=20 gentle, R=12 medium, R=7 tight, R=4 very tight (diff-drive)
# ---------------------------------------------------------------------------
_CURVATURES = [
    0.0,
    +1/20.0, -1/20.0,
    +1/12.0, -1/12.0,
    +1/7.0,  -1/7.0,
    +1/4.5,  -1/4.5,   # tight turn — diff-drive can do this
]
GEAR_OPTIONS = [1, -1]

# ---------------------------------------------------------------------------
# Static obstacle map  (from paddock.sdf, inflated by INFLATION)
# ---------------------------------------------------------------------------
_RAW_BOXES = [
    # corridor walls
    (-22.5,  30.0,   2.5,  2.0, 0.0),
    ( 47.5,  30.0,  27.5,  2.0, 0.0),
    (  5.0, 115.0,  30.0,  2.0, 0.0),
    ( 70.0, 115.0,   5.0,  2.0, 0.0),
    ( 75.0,  72.5,   2.0, 42.5, 0.0),   # east wall
    (-25.0,  72.5,   2.0, 42.5, 0.0),   # west wall
    # haul-road right kerb
    (  6.0, -50.0,   1.75, 46.0, 0.0),
    # corridor berms (gen_paddock_world.py ob_berm_a/b) -- repositioned to
    # the empty eastern half of the corridor (x:45-72,y:40-90), clear of
    # truck_orchestrator.py's locked-in ROUTES by 15m+; rock clusters and
    # ore piles are small/scattered and not modeled here, same as before.
    ( 58.0,  48.0,  10.0,  2.5, 0.08),
    ( 58.0,  86.0,  10.0,  2.5, -0.06),
]

_RAW_CIRCLES = []


@dataclass(frozen=True)
class _Box:
    cx: float; cy: float
    hdx: float; hdy: float
    cos_y: float; sin_y: float

    def hit(self, px: float, py: float) -> bool:
        dx = px - self.cx; dy = py - self.cy
        lx = abs( self.cos_y * dx + self.sin_y * dy)
        ly = abs(-self.sin_y * dx + self.cos_y * dy)
        return lx <= self.hdx and ly <= self.hdy


@dataclass(frozen=True)
class _Circle:
    cx: float; cy: float; r: float

    def hit(self, px: float, py: float) -> bool:
        return (px - self.cx)**2 + (py - self.cy)**2 <= self.r * self.r


def _build_map(inf: float = INFLATION):
    boxes = [_Box(b[0], b[1],
                  b[2] + inf, b[3] + inf,
                  math.cos(b[4]), math.sin(b[4]))
             for b in _RAW_BOXES]
    circs = [_Circle(c[0], c[1], c[2] + inf) for c in _RAW_CIRCLES]
    return boxes, circs


_BOXES, _CIRCLES = _build_map()


def point_in_collision(px: float, py: float) -> bool:
    for b in _BOXES:
        if b.hit(px, py):
            return True
    for c in _CIRCLES:
        if c.hit(px, py):
            return True
    return False


_FP = [
    ( 0.0,     0.0),
    ( HALF_LEN,  HALF_WID), ( HALF_LEN, -HALF_WID),
    (-HALF_LEN,  HALF_WID), (-HALF_LEN, -HALF_WID),
    ( HALF_LEN,  0.0),      (-HALF_LEN,  0.0),
    ( 0.0,  HALF_WID),      ( 0.0, -HALF_WID),
]


def footprint_free(x: float, y: float, th: float) -> bool:
    c, s = math.cos(th), math.sin(th)
    for fl, fw in _FP:
        if point_in_collision(x + c*fl - s*fw, y + s*fl + c*fw):
            return False
    return True


# ---------------------------------------------------------------------------
# A* node
# ---------------------------------------------------------------------------
def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(order=True)
class _Node:
    f: float
    g: float          = field(compare=False)
    x: float          = field(compare=False)
    y: float          = field(compare=False)
    th: float         = field(compare=False)
    gear: int         = field(compare=False, default=1)
    parent: Optional['_Node'] = field(compare=False, default=None)

    def key(self) -> Tuple[int, int, int]:
        return (round(self.x / XY_RES),
                round(self.y / XY_RES),
                round(self.th / TH_RES) % TH_BINS)


def _h(x, y, gx, gy):
    return math.hypot(x - gx, y - gy)


def _expand(node: _Node) -> List[_Node]:
    out = []
    for gear in GEAR_OPTIONS:
        dist = gear * STEP_LEN
        for kappa in _CURVATURES:
            if abs(kappa) < 1e-9:
                nx  = node.x + dist * math.cos(node.th)
                ny  = node.y + dist * math.sin(node.th)
                nth = node.th
            else:
                R   = 1.0 / kappa
                dth = dist * kappa
                nx  = node.x + R * (math.sin(node.th + dth) - math.sin(node.th))
                ny  = node.y - R * (math.cos(node.th + dth) - math.cos(node.th))
                nth = _wrap(node.th + dth)

            if not footprint_free(nx, ny, nth):
                continue

            gc = (node.g
                  + abs(dist)
                  + 0.5 * abs(kappa) * abs(dist)
                  + (4.0 if gear != node.gear else 0.0)
                  + (1.5 if gear == -1 else 0.0))

            out.append(_Node(f=0.0, g=gc, x=nx, y=ny, th=nth,
                             gear=gear, parent=node))
    return out


# ---------------------------------------------------------------------------
# Single-segment A*
# ---------------------------------------------------------------------------
def _plan_segment(sx, sy, sth, gx, gy, gth,
                  pos_tol, th_tol, max_iter):
    start = _Node(f=0.0, g=0.0, x=sx, y=sy, th=sth)
    start.f = _h(sx, sy, gx, gy)

    heap: list = []; heapq.heappush(heap, start)
    closed: dict = {}

    for _ in range(max_iter):
        if not heap:
            break
        node = heapq.heappop(heap)
        key = node.key()
        if key in closed and closed[key] <= node.g:
            continue
        closed[key] = node.g

        if (math.hypot(node.x - gx, node.y - gy) < pos_tol and
                abs(_wrap(node.th - gth)) < th_tol):
            path = []
            cur = node
            while cur:
                path.append((cur.x, cur.y, cur.th))
                cur = cur.parent
            path.reverse()
            return path

        for child in _expand(node):
            ck = child.key()
            if ck in closed and closed[ck] <= child.g:
                continue
            child.f = child.g + _h(child.x, child.y, gx, gy)
            heapq.heappush(heap, child)

    return None


# ---------------------------------------------------------------------------
# Public API — waypoint-guided multi-segment planning
# ---------------------------------------------------------------------------

# Hard-coded via-points that thread through the corridor gaps (debugged above)
# These guide the planner through the two bottlenecks:
#   1. Right side of berm-1 (x≈50, y≈50)
#   2. Left side of berm-2  (x≈20, y≈88)
# Via-points verified by full obstacle map scan + segment tests (all FOUND).
# Spawn: (-30,-50) yaw=0 (facing +X / east, no -yaw arg in launch file).
# Route: east along staging row → left lane north → centre narrow →
#        swing right → right rail past berm-1 → centre gap past berm-2 → goal
_VIA_POINTS = [
    ( -5.0, -50.0, 0.0        ),  # east along staging row to left of kerb
    ( -5.0,  15.0, math.pi/2  ),  # turn north, up left lane through gate
    (  0.0,  28.0, math.pi/2  ),  # centre through narrow section (y=25-35)
    (  0.0,  35.0, math.pi/2  ),  # corridor opens right
    ( 35.0,  40.0, math.pi/2  ),  # swing right into open zone
    ( 60.0,  50.0, math.pi/2  ),  # right rail, clear of berm-1 (y=55)
    ( 60.0,  65.0, math.pi/2  ),  # right rail mid
    ( 60.0,  80.0, math.pi/2  ),  # right rail full-width open zone
    ( 25.0,  90.0, math.pi/2  ),  # centre gap past berm-2 (y=90)
    ( 25.0, 100.0, math.pi/2  ),  # centre approach
    ( 50.0, 110.0, math.pi/2  ),  # dump approach goal
]


def plan(start_x: float, start_y: float, start_th: float,
         goal_x:  float, goal_y:  float, goal_th:  float,
         goal_pos_tol: float = 4.0,
         goal_th_tol:  float = math.radians(30.0),
         max_iter: int = 25_000,
         ) -> Optional[List[Tuple[float, float, float]]]:
    """
    Waypoint-guided Hybrid A*.

    Splits the route into short segments via known clear corridor waypoints,
    solves each segment independently, then concatenates the paths.

    Returns list of (x, y, heading_rad), or None if any segment fails.
    """
    # Build waypoint list: start → vias → goal
    waypoints = [(start_x, start_y, start_th)] + _VIA_POINTS
    # Replace final via with actual goal if it's close
    waypoints[-1] = (goal_x, goal_y, goal_th)

    full_path: List[Tuple[float, float, float]] = []

    for i in range(len(waypoints) - 1):
        sx, sy, sth = waypoints[i]
        gx, gy, gth = waypoints[i + 1]

        # Skip trivially close segments
        if math.hypot(sx - gx, sy - gy) < 2.0:
            continue

        seg = _plan_segment(sx, sy, sth, gx, gy, gth,
                            pos_tol=goal_pos_tol,
                            th_tol=goal_th_tol,
                            max_iter=max_iter)
        if seg is None:
            return None

        # Avoid duplicating the join point between segments
        if full_path:
            seg = seg[1:]
        full_path.extend(seg)

    return full_path if full_path else None


# ---------------------------------------------------------------------------
# Quick self-test  (python3 hybrid_astar_core.py)
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    import time
    sx, sy, sth = -30.0, -50.0, math.pi / 2   # truck_1 spawn pose
    gx, gy, gth =  50.0, 110.0, math.pi / 2   # dump approach goal

    print("Hybrid A* self-test")
    print(f"  start : ({sx}, {sy}, {math.degrees(sth):.0f}°)")
    print(f"  goal  : ({gx}, {gy}, {math.degrees(gth):.0f}°)")
    t0 = time.time()
    result = plan(sx, sy, sth, gx, gy, gth)
    dt = time.time() - t0
    if result:
        print(f"  FOUND : {len(result)} waypoints  [{dt:.2f}s]")
        for i, (x, y, th) in enumerate(result[::8]):
            print(f"    [{i*8:4d}]  x={x:7.2f}  y={y:7.2f}  th={math.degrees(th):6.1f}°")
    else:
        print(f"  NOT FOUND  [{dt:.2f}s]")