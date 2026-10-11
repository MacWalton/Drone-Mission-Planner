# SPDX-License-Identifier: GPL-2.0-only
"""Obstacles: tree canopy, powerlines, towers, buildings - anything the aircraft
must stay above.

Each obstacle is a point, polyline or polygon (planning CRS, metres) with a
height above ground. The planner keeps every waypoint at least
`height + vertical margin` above ground wherever it passes within the
horizontal buffer of an obstacle, climbing only where needed.

A uniform grid index keeps lookups fast for thousands of features (building
footprints, tree polygons, powerline spans).
"""
import math
from dataclasses import dataclass, field
from typing import List

from . import geometry as G


@dataclass
class Obstacle:
    kind: str                 # "point" | "line" | "polygon"
    coords: list              # point: [(x, y)]; line: [(x, y), ...]; polygon: [ring, ring, ...]
    height_m: float
    name: str = ""

    def bbox(self):
        pts = [p for r in self.coords for p in r] if self.kind == "polygon" else self.coords
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return min(xs), min(ys), max(xs), max(ys)

    def edges(self):
        """([(p, q), ...] edges, [vertices])."""
        if self.kind == "point":
            return [], list(self.coords)
        rings = self.coords if self.kind == "polygon" else [self.coords]
        edges, verts = [], []
        for r in rings:
            seq = list(r) + ([r[0]] if self.kind == "polygon" and r[0] != r[-1] else [])
            edges += list(zip(seq[:-1], seq[1:]))
            verts += list(r)
        return edges, verts

    def distance(self, p):
        if self.kind == "point":
            return G.dist(p, self.coords[0])
        if self.kind == "polygon" and G.point_in_polygon(p, self.coords):
            return 0.0
        edges, _ = self.edges()
        return min(_seg_dist(p, a, b) for a, b in edges) if edges else G.dist(p, self.coords[0])

    def distance_to_segment(self, a, b):
        """Shortest distance between segment ab and the obstacle."""
        if self.kind == "point":
            return _seg_dist(self.coords[0], a, b)
        if self.kind == "polygon" and (G.point_in_polygon(a, self.coords) or G.point_in_polygon(b, self.coords)):
            return 0.0
        edges, _ = self.edges()
        return min(_seg_seg(a, b, e0, e1) for e0, e1 in edges) if edges else float("inf")


def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - (a[0] + dx * t), p[1] - (a[1] + dy * t))


def _proj_t(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    return 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))


def _cross_t(a, b, c, d):
    """Parameter t on ab where it crosses cd, or None."""
    r = (b[0] - a[0], b[1] - a[1])
    s_ = (d[0] - c[0], d[1] - c[1])
    den = r[0] * s_[1] - r[1] * s_[0]
    if abs(den) < 1e-12:
        return None
    t = ((c[0] - a[0]) * s_[1] - (c[1] - a[1]) * s_[0]) / den
    u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den
    return t if (0 <= t <= 1 and 0 <= u <= 1) else None


def _seg_seg(a, b, c, d):
    if _cross_t(a, b, c, d) is not None:
        return 0.0
    return min(_seg_dist(a, c, d), _seg_dist(b, c, d), _seg_dist(c, a, b), _seg_dist(d, a, b))


class ObstacleSet:
    """Obstacles + buffer/margin, answering 'how high must I be here?'."""

    def __init__(self, obstacles: List[Obstacle], h_buffer_m=15.0, v_margin_m=10.0, cell=200.0):
        self.obs = [o for o in obstacles if o.height_m is not None and o.height_m > 0]
        self.buf = max(0.0, h_buffer_m)
        self.margin = max(0.0, v_margin_m)
        self.cell = cell
        self._bb = [o.bbox() for o in self.obs]
        self._ev = [o.edges() for o in self.obs]
        self.grid = {}
        for i, (x0, y0, x1, y1) in enumerate(self._bb):
            for gx in range(int(math.floor((x0 - self.buf) / cell)), int(math.floor((x1 + self.buf) / cell)) + 1):
                for gy in range(int(math.floor((y0 - self.buf) / cell)), int(math.floor((y1 + self.buf) / cell)) + 1):
                    self.grid.setdefault((gx, gy), []).append(i)

    def __bool__(self):
        return bool(self.obs)

    def __len__(self):
        return len(self.obs)

    def nearby(self, p):
        return self.grid.get((int(math.floor(p[0] / self.cell)), int(math.floor(p[1] / self.cell))), [])

    def top(self, p):
        """Height (m above ground) of the tallest obstacle within the buffer of p, or 0."""
        best = 0.0
        for i in self.nearby(p):
            o = self.obs[i]
            if o.height_m > best and o.distance(p) <= self.buf:
                best = o.height_m
        return best

    def required_agl(self, p):
        """Minimum height above ground at p to clear obstacles (0 if none near)."""
        t = self.top(p)
        return t + self.margin if t > 0 else 0.0

    def tallest(self):
        return max((o.height_m for o in self.obs), default=0.0)

    def segment_params(self, a, b):
        """Extra sample parameters t in [0, 1] on segment ab: obstacle vertex
        projections and edge crossings for every obstacle whose safety zone the
        segment enters - so a narrow clip of a zone is never missed."""
        x0, x1 = sorted((a[0], b[0]))
        y0, y1 = sorted((a[1], b[1]))
        c = self.cell
        ids = set()
        for gx in range(int(math.floor((x0 - self.buf) / c)), int(math.floor((x1 + self.buf) / c)) + 1):
            for gy in range(int(math.floor((y0 - self.buf) / c)), int(math.floor((y1 + self.buf) / c)) + 1):
                ids.update(self.grid.get((gx, gy), ()))
        ts = []
        for i in ids:
            bx0, by0, bx1, by1 = self._bb[i]
            if bx1 + self.buf < x0 or bx0 - self.buf > x1 or by1 + self.buf < y0 or by0 - self.buf > y1:
                continue
            half_diag = math.hypot(bx1 - bx0, by1 - by0) / 2
            if _seg_dist(((bx0 + bx1) / 2, (by0 + by1) / 2), a, b) > half_diag + self.buf:
                continue
            if self.obs[i].distance_to_segment(a, b) > self.buf:
                continue
            edges, verts = self._ev[i]
            ts += [_proj_t(v, a, b) for v in verts]
            for e0, e1 in edges:
                t = _cross_t(a, b, e0, e1)
                if t is not None:
                    ts.append(t)
        return ts
