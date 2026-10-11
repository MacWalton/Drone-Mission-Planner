# SPDX-License-Identifier: GPL-2.0-only
"""Planar geometry used by the planner.

All coordinates are in a projected, metric CRS (metres). The GIS wrappers
reproject the user's boundary into such a CRS before calling these
functions and back to WGS84 for export.

A polygon is a list of rings; ring 0 is the exterior, further rings are
holes (no-fly areas inside the boundary). A ring is a list of (x, y)
tuples; closing vertex optional.
"""
import math


# ---------------------------------------------------------------- basics
def _open(ring):
    ring = [tuple(p[:2]) for p in ring]
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring = ring[:-1]
    return ring


def ring_area(ring):
    ring = _open(ring)
    a = 0.0
    for i in range(len(ring)):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % len(ring)]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def polygon_area(rings):
    if not rings:
        return 0.0
    return abs(ring_area(rings[0])) - sum(abs(ring_area(r)) for r in rings[1:])


def centroid(points):
    pts = _open(points)
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(pts):
    return sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def azimuth(a, b):
    """Grid azimuth a->b in degrees clockwise from north (0-360)."""
    return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.0


def rotate(pts, angle_rad, origin=(0.0, 0.0)):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    ox, oy = origin
    return [(ox + (x - ox) * c - (y - oy) * s, oy + (x - ox) * s + (y - oy) * c) for x, y in pts]


def _heading_to_math(heading_deg):
    """Rotation that maps the azimuth direction onto +x."""
    return -math.radians(90.0 - heading_deg)


def point_in_polygon(pt, rings):
    def inside(p, ring):
        ring = _open(ring)
        x, y = p
        c = False
        j = len(ring) - 1
        for i in range(len(ring)):
            xi, yi = ring[i]
            xj, yj = ring[j]
            if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi:
                c = not c
            j = i
        return c
    if not inside(pt, rings[0]):
        return False
    return not any(inside(pt, h) for h in rings[1:])


# ---------------------------------------------------- automatic heading
def min_width_heading(rings):
    """Flight-line azimuth (deg) that minimises the number of lines.

    Tests every edge direction of the convex hull (rotating calipers); the
    best line direction is parallel to the edge giving the smallest width.
    """
    hull = convex_hull(_open(rings[0]))
    best = (float("inf"), 0.0)
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        if dist(a, b) < 1e-9:
            continue
        h = azimuth(a, b)
        r = rotate(hull, _heading_to_math(h))
        ys = [p[1] for p in r]
        width = max(ys) - min(ys)
        if width < best[0]:
            best = (width, h)
    return best[1] % 180.0


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


# ------------------------------------------------------ area scan lines
def _scan_intervals(rings_rot, y):
    xs = []
    for ring in rings_rot:
        n = len(ring)
        for i in range(n):
            x1, y1 = ring[i]
            x2, y2 = ring[(i + 1) % n]
            if (y1 <= y < y2) or (y2 <= y < y1):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def scan_segments(rings, heading_deg, spacing, overshoot=0.0, margin=0.0,
                  min_segment=1.0):
    """Parallel survey segments covering a polygon.

    heading_deg : azimuth of the flight lines
    spacing     : distance between lines (m)
    overshoot   : extra run-in/run-out added to each end (m) - gives the
                  aircraft room to stabilise before the first photo
    margin      : extend coverage beyond the boundary (m, can be negative)
    Returns list of lines; each line is a list of ((x1,y1),(x2,y2)) segments
    (a concave boundary can split a line into several segments).
    """
    if spacing <= 0:
        raise ValueError("Line spacing must be > 0")
    ang = _heading_to_math(heading_deg)
    origin = centroid(rings[0])
    rot = [rotate(_open(r), ang, origin) for r in rings]
    ys = [p[1] for p in rot[0]]
    ymin, ymax = min(ys) - margin, max(ys) + margin
    h = ymax - ymin
    n = max(1, int(math.ceil(h / spacing)))
    start = ymin + (h - (n - 1) * spacing) / 2.0 if n > 1 else (ymin + ymax) / 2.0
    lines = []
    for k in range(n):
        y = start + k * spacing
        # sample slightly inside the band when margin pushes us outside
        yq = min(max(y, min(ys) + 1e-6), max(ys) - 1e-6)
        segs = []
        for x1, x2 in _scan_intervals(rot, yq):
            x1 -= margin + overshoot
            x2 += margin + overshoot
            if x2 - x1 >= min_segment:
                segs.append(((x1, y), (x2, y)))
        if segs:
            back = []
            for (a, b) in segs:
                pa, pb = rotate([a, b], -ang, origin)
                back.append((pa, pb))
            lines.append(back)
    return lines


# ------------------------------------------------------- corridor maths
def offset_polyline(pts, d, miter_limit=4.0):
    """Offset an open polyline to the LEFT by d metres (negative = right)."""
    pts = [tuple(p[:2]) for p in pts]
    clean = [pts[0]]
    for p in pts[1:]:
        if dist(p, clean[-1]) > 1e-6:
            clean.append(p)
    pts = clean
    if len(pts) < 2:
        return pts
    normals = []
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        L = math.hypot(x2 - x1, y2 - y1)
        normals.append((-(y2 - y1) / L, (x2 - x1) / L))
    out = [(pts[0][0] + normals[0][0] * d, pts[0][1] + normals[0][1] * d)]
    for i in range(1, len(pts) - 1):
        n1, n2 = normals[i - 1], normals[i]
        dot = n1[0] * n2[0] + n1[1] * n2[1]
        mx, my = n1[0] + n2[0], n1[1] + n2[1]
        denom = 1.0 + dot
        if denom < 1e-6:
            scale = d
            mx, my = n1
        else:
            scale = d / denom
            if abs(scale * math.hypot(mx, my)) > abs(d) * miter_limit:
                scale = math.copysign(abs(d) * miter_limit / math.hypot(mx, my), scale)
        out.append((pts[i][0] + mx * scale, pts[i][1] + my * scale))
    out.append((pts[-1][0] + normals[-1][0] * d, pts[-1][1] + normals[-1][1] * d))
    return out


def extend_polyline(pts, start_m, end_m):
    pts = list(pts)
    if len(pts) < 2:
        return pts
    (x0, y0), (x1, y1) = pts[0], pts[1]
    L = dist(pts[0], pts[1]) or 1.0
    pts[0] = (x0 - (x1 - x0) / L * start_m, y0 - (y1 - y0) / L * start_m)
    (xa, ya), (xb, yb) = pts[-2], pts[-1]
    L = dist(pts[-2], pts[-1]) or 1.0
    pts[-1] = (xb + (xb - xa) / L * end_m, yb + (yb - ya) / L * end_m)
    return pts


def corridor_polygon(centerline, width):
    left = offset_polyline(centerline, width / 2.0)
    right = offset_polyline(centerline, -width / 2.0)
    ring = left + list(reversed(right))
    return [ring + [ring[0]]]


def densify(pts, step):
    """Insert vertices so no segment is longer than step (m)."""
    if step <= 0 or len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        L = dist(a, b)
        n = int(math.ceil(L / step))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


# ------------------------------------------------- linear-feature helpers
def line_length(pts):
    return path_length(pts)


def point_at(pts, chainage):
    """Point at a distance along a polyline (clamped to its ends)."""
    if chainage <= 0:
        return tuple(pts[0][:2])
    run = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        L = dist(a, b)
        if run + L >= chainage and L > 0:
            t = (chainage - run) / L
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        run += L
    return tuple(pts[-1][:2])


def substring(pts, c0, c1):
    """Part of a polyline between two chainages (metres from its start)."""
    pts = [tuple(p[:2]) for p in pts]
    total = path_length(pts)
    c0, c1 = max(0.0, min(c0, total)), max(0.0, min(c1, total))
    if c1 < c0:
        c0, c1 = c1, c0
    out = [point_at(pts, c0)]
    run = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        run += dist(a, b)
        if c0 < run < c1:
            out.append(b)
    end = point_at(pts, c1)
    if dist(out[-1], end) > 1e-9:
        out.append(end)
    return out


def simplify(pts, tol):
    """Douglas-Peucker simplification (tol in metres). Keeps end points."""
    pts = [tuple(p[:2]) for p in pts]
    if tol <= 0 or len(pts) < 3:
        return pts

    def perp(p, a, b):
        L = dist(a, b)
        if L == 0:
            return dist(p, a)
        return abs((b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1])) / L

    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        best, idx = 0.0, None
        for k in range(i + 1, j):
            d = perp(pts[k], pts[i], pts[j])
            if d > best:
                best, idx = d, k
        if idx is not None and best > tol:
            keep[idx] = True
            stack += [(i, idx), (idx, j)]
    return [p for p, k in zip(pts, keep) if k]


def chain_lines(lines):
    """Join several line pieces (e.g. road segments) into one ordered polyline.

    Pieces may be in any order or direction. Returns (polyline, largest_gap_m):
    the gap is the biggest jump made between pieces that didn't touch.
    """
    lines = [[tuple(p[:2]) for p in l] for l in lines if len(l) >= 2]
    if not lines:
        return [], 0.0
    if len(lines) == 1:
        return lines[0], 0.0
    ends = [(l[0], l[-1]) for l in lines]

    def nearest_other(p, skip):
        return min(dist(p, e) for k, (s, t) in enumerate(ends) if k != skip for e in (s, t))

    # start from the most isolated end - a true terminus of the network
    best = (-1.0, 0, False)
    for k, (s, t) in enumerate(ends):
        for rev, p in ((False, s), (True, t)):
            d = nearest_other(p, k)
            if d > best[0]:
                best = (d, k, rev)
    _, k, rev = best
    chain = list(reversed(lines[k])) if rev else list(lines[k])
    used = {k}
    gap = 0.0
    while len(used) < len(lines):
        tail = chain[-1]
        cand = min(((dist(tail, l[0]), j, False) for j, l in enumerate(lines) if j not in used),
                   default=None)
        cand2 = min(((dist(tail, l[-1]), j, True) for j, l in enumerate(lines) if j not in used),
                    default=None)
        d, j, r = min(cand, cand2)
        piece = list(reversed(lines[j])) if r else list(lines[j])
        gap = max(gap, d)
        chain.extend(piece[1:] if d < 1e-6 else piece)
        used.add(j)
    return chain, gap


def chainage_points(pts, interval, start_chainage=0.0):
    """(x, y, chainage_m) every `interval` metres along a polyline."""
    total = path_length(pts)
    out = []
    first = math.ceil(start_chainage / interval) * interval - start_chainage if interval > 0 else 0
    c = first
    while c <= total + 1e-6 and interval > 0:
        x, y = point_at(pts, c)
        out.append((x, y, start_chainage + c))
        c += interval
    return out


def locate(pts, p):
    """Chainage (m along the polyline) of the point on it nearest to p."""
    best, run, best_c = float("inf"), 0.0, 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        q = (a[0] + dx * t, a[1] + dy * t)
        d = dist(p, q)
        if d < best:
            best, best_c = d, run + math.sqrt(L2) * t
        run += math.sqrt(L2)
    return best_c
