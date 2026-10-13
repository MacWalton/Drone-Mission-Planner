# SPDX-License-Identifier: GPL-2.0-only
"""Photo positions and image footprints along the flights actually planned.

Photos are placed along each flight's survey legs at the trigger distance
(continuing across the extra waypoints of terrain-following lines and
restarting at every line start); in stop-and-shoot mode the shoot waypoints are
used. Each footprint is the camera's ground coverage at that photo:

* size from the camera and the height above ground there - with a DEM the real
  ground under the photo, otherwise ground level with the launch station;
* turned to the flight direction (portrait setting respected);
* nadir camera assumed (gimbal pitch -90 deg). Oblique shots are flagged.
"""
import math

from . import photogrammetry as P


def photo_positions(mission):
    """[(flight_index, x, y, heading_rad, alt_above_launch, z_ref)]"""
    out = []
    trig = mission.trigger_m
    stop = getattr(mission.params, "photo_trigger", "distance") == "stop"
    for f in mission.flights:
        wps = f.waypoints
        if stop and any(getattr(w, "shoot", False) for w in wps):
            for i, w in enumerate(wps):
                if getattr(w, "shoot", False):
                    nb = wps[i + 1] if i + 1 < len(wps) else wps[i - 1]
                    hd = math.atan2(nb.x - w.x, nb.y - w.y) if (nb.x, nb.y) != (w.x, w.y) else 0.0
                    if nb is wps[i - 1] and i + 1 >= len(wps):
                        hd += math.pi
                    out.append((f.index, w.x, w.y, hd, w.alt, f.z_ref))
            continue
        carry = None                                    # distance to the next photo in this run
        for a, b in zip(wps[:-1], wps[1:]):
            if a.photo_dist <= 0:
                carry = None
                continue
            if a.kind in ("line_start", "resume") or carry is None:
                carry = 0.0
            L = math.hypot(b.x - a.x, b.y - a.y)
            if L <= 1e-9:
                continue
            hd = math.atan2(b.x - a.x, b.y - a.y)
            d = carry
            while d <= L + 1e-6:
                t = d / L
                out.append((f.index, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, hd,
                            a.alt + (b.alt - a.alt) * t, f.z_ref))
                d += trig
            carry = d - L
    return out


def photo_footprints(mission, ground=None):
    """One dict per photo: flight, n, x, y, agl_m, gsd_cm, across_m, along_m, ring.

    ground: f(x, y) -> m AMSL (defaults to the DEM the mission was planned with)."""
    ground = ground or getattr(mission, "ground", None)
    cam = mission.drone.camera
    portrait = bool(getattr(mission.params, "portrait", False))
    out = []
    for n, (fi, x, y, hd, alt, zref) in enumerate(photo_positions(mission)):
        agl = alt
        if ground is not None:
            try:
                g = ground(x, y)
                if g is not None:
                    agl = alt + zref - g
            except Exception:
                pass
        agl = max(agl, 1.0)
        across, along = P.footprint(cam, agl, portrait)
        ux, uy = math.sin(hd), math.cos(hd)              # along track
        vx, vy = uy, -ux                                  # across track (right)
        ha, hc = along / 2.0, across / 2.0
        ring = [(x + ux * ha + vx * hc, y + uy * ha + vy * hc), (x + ux * ha - vx * hc, y + uy * ha - vy * hc),
                (x - ux * ha - vx * hc, y - uy * ha - vy * hc), (x - ux * ha + vx * hc, y - uy * ha + vy * hc)]
        ring.append(ring[0])
        out.append({"flight": fi + 1, "n": n + 1, "x": x, "y": y, "agl_m": agl,
                    "gsd_cm": P.gsd_from_height(cam, agl), "across_m": across, "along_m": along,
                    "heading_deg": math.degrees(hd) % 360.0, "ring": ring})
    return out


def oblique(mission):
    return abs(getattr(mission.params, "gimbal_pitch_deg", -90.0) + 90.0) > 10.0


# ------------------------------------------------------------------ coverage extent
def coverage_extent(mission, ground=None, max_cells=1500000, fps=None):
    """Outline of the ground covered by the mission's photos (union of all image
    footprints) -> {"rings": [[(x, y), ...], ...], "area_m2": float, "photos": n}.

    Footprints are rasterised on a grid aligned with the survey lines (so the
    outline of straight lines has no stair steps), the covered cells are traced
    into closed rings (outer boundaries and holes) and simplified to the cell size."""
    fps = fps if fps is not None else photo_footprints(mission, ground)
    if not fps:
        return {"rings": [], "area_m2": 0.0, "photos": 0}
    ang = math.radians(getattr(mission, "heading_deg", 0.0) or 0.0)
    ca, sa = math.cos(ang), math.sin(ang)

    def fwd(x, y):                                     # planning CRS -> frame aligned with the lines
        return (x * ca - y * sa, x * sa + y * ca)

    def back(u, v):
        return (u * ca + v * sa, -u * sa + v * ca)
    polys = [[fwd(*p) for p in f["ring"][:4]] for f in fps]
    us = [p[0] for q in polys for p in q]
    vs = [p[1] for q in polys for p in q]
    u0, v0, u1, v1 = min(us), min(vs), max(us), max(vs)
    small = min(min(f["across_m"], f["along_m"]) for f in fps)
    cell = max(small / 12.0, math.sqrt((u1 - u0) * (v1 - v0) / max_cells), 0.25)
    nx, ny = int((u1 - u0) / cell) + 2, int((v1 - v0) / cell) + 2
    grid = bytearray(nx * ny)
    for q in polys:
        # scanline fill of a convex quad, cell centres
        qy0 = max(0, int((min(p[1] for p in q) - v0) / cell))
        qy1 = min(ny - 1, int((max(p[1] for p in q) - v0) / cell) + 1)
        edges = list(zip(q, q[1:] + q[:1]))
        for j in range(qy0, qy1 + 1):
            yc = v0 + (j + 0.5) * cell
            xs = []
            for (ax, ay), (bx, by) in edges:
                if (ay <= yc < by) or (by <= yc < ay):
                    xs.append(ax + (yc - ay) * (bx - ax) / (by - ay))
            if len(xs) < 2:
                continue
            i0 = max(0, int(math.ceil((min(xs) - u0) / cell - 0.5)))
            i1 = min(nx - 1, int(math.floor((max(xs) - u0) / cell - 0.5)))
            if i1 >= i0:
                row = j * nx
                grid[row + i0:row + i1 + 1] = b"\x01" * (i1 - i0 + 1)
    covered = sum(grid)
    # boundary edges with the covered cell on the left (outer rings counter-clockwise, holes clockwise);
    # several edges may start at one vertex (pinch points)
    starts = {}

    def add(a, b):
        starts.setdefault(a, []).append(b)
    for j in range(ny):
        row = j * nx
        for i in range(nx):
            if not grid[row + i]:
                continue
            if j == 0 or not grid[row - nx + i]:
                add((i, j), (i + 1, j))
            if i == nx - 1 or not grid[row + i + 1]:
                add((i + 1, j), (i + 1, j + 1))
            if j == ny - 1 or not grid[row + nx + i]:
                add((i + 1, j + 1), (i, j + 1))
            if i == 0 or not grid[row + i - 1]:
                add((i, j + 1), (i, j))
    rings = []
    while starts:
        a = next(iter(starts))
        ring = [a]
        cur = a
        prev_dir = None
        while True:
            outs = starts.get(cur)
            if not outs:
                break
            if len(outs) > 1 and prev_dir is not None:   # pinch: turn left first to keep rings simple
                def turn(b):
                    d = (b[0] - cur[0], b[1] - cur[1])
                    return (prev_dir[0] * d[1] - prev_dir[1] * d[0])
                outs.sort(key=turn, reverse=True)
            b = outs.pop(0)
            if not outs:
                del starts[cur]
            prev_dir = (b[0] - cur[0], b[1] - cur[1])
            cur = b
            if cur == a:
                break
            ring.append(cur)
        if len(ring) >= 4:
            pts = [(u0 + i * cell, v0 + j * cell) for i, j in ring]
            pts = _drop_collinear(pts)
            simp = _simplify_ring(pts, cell * 0.9)
            if len(simp) >= 3:
                rings.append([back(*p) for p in simp] + [back(*simp[0])])
    rings.sort(key=lambda r: -abs(_area(r)))
    return {"rings": rings, "area_m2": covered * cell * cell, "photos": len(fps), "cell_m": cell}


def _area(r):
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(r, r[1:] + r[:1]))


def _drop_collinear(pts):
    out = []
    n = len(pts)
    for k in range(n):
        a, b, c = pts[k - 1], pts[k], pts[(k + 1) % n]
        if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) != 0:
            out.append(b)
    return out or pts


def _simplify_ring(pts, tol):
    from .geometry import simplify
    if len(pts) < 8 or tol <= 0:
        return pts
    # split at the two farthest-apart points so Douglas-Peucker works on open chains
    k = max(range(len(pts)), key=lambda i: (pts[i][0] - pts[0][0]) ** 2 + (pts[i][1] - pts[0][1]) ** 2)
    a = simplify(pts[:k + 1], tol)
    b = simplify(pts[k:] + [pts[0]], tol)
    return a[:-1] + b[:-1]
