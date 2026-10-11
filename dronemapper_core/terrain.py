# SPDX-License-Identifier: GPL-2.0-only
"""Terrain safety checks and route elevation profiles (needs a DEM).

check_terrain() follows every flight's real 3D path - vertical climb at the
launch station, transit to the first line, the survey lines, transits between
lines and the return - and compares the aircraft altitude with the ground:

* clearance below a minimum (default 30 m)    -> "low" spots
* height above ground over a legal ceiling     -> "high" spots (e.g. 120 m AGL
  where the ground falls away from the launch station)
* actual GSD and overlap over the terrain      -> photo quality where ground rises

Altitudes in a mission are relative to each flight's launch station, so the
station's own ground height (from the DEM) is the reference - exactly how the
aircraft flies the exported files.

route_profile() gives ground and flight altitude by chainage (KP) for linear
missions; svg_profile() draws it for the field brief.
"""
import math
from dataclasses import dataclass, field
from typing import List

from . import geometry as G
from . import photogrammetry as P


@dataclass
class TerrainIssue:
    kind: str            # "low" | "high" | "obstacle"
    flight: int          # 1-based
    x: float
    y: float
    value_m: float       # clearance (low) or height above ground (high)
    ground_m: float
    alt_amsl: float
    kp: float = None     # route chainage, metres


@dataclass
class TerrainReport:
    min_clearance_m: float
    max_agl_m: float
    clearance_limit: float
    agl_limit: float
    per_flight: list                     # [(flight, min_clear, max_agl)]
    issues: List[TerrainIssue]
    gsd_min: float = None
    gsd_mean: float = None
    gsd_max: float = None
    agl_survey_min: float = None
    agl_survey_max: float = None
    side_overlap_min: float = None
    front_overlap_min: float = None
    samples: int = 0
    unknown: int = 0                     # samples outside the DEM
    suggestions: list = field(default_factory=list)

    @property
    def ok(self):
        return not self.issues

    def text(self, unit="m"):
        from .units import fmt
        out = ["Terrain & obstacle check:",
               "  Lowest clearance  : %s  (limit %s)" % (fmt(self.min_clearance_m, unit), fmt(self.clearance_limit, unit)),
               "  Highest above gnd : %s  (limit %s)" % (fmt(self.max_agl_m, unit), fmt(self.agl_limit, unit))]
        if self.gsd_min is not None:
            out.append("  GSD over terrain  : %.2f - %.2f cm/px (mean %.2f)"
                       % (self.gsd_min, self.gsd_max, self.gsd_mean))
            out.append("  Height over survey: %s - %s above ground"
                       % (fmt(self.agl_survey_min, unit), fmt(self.agl_survey_max, unit)))
            out.append("  Worst overlap     : %.0f%% front / %.0f%% side (where ground is highest)"
                       % (self.front_overlap_min, self.side_overlap_min))
        lows = [i for i in self.issues if i.kind == "low"]
        highs = [i for i in self.issues if i.kind == "high"]
        obst = [i for i in self.issues if i.kind == "obstacle"]
        if obst:
            out.append("  TOO CLOSE TO OBSTACLES at %d spot(s) in flight(s) %s" %
                       (len(obst), ", ".join(str(f) for f in sorted({i.flight for i in obst}))))
        if lows:
            out.append("  LOW CLEARANCE at %d spot(s) in flight(s) %s" %
                       (len(lows), ", ".join(str(f) for f in sorted({i.flight for i in lows}))))
        if highs:
            out.append("  ABOVE %s AGL at %d spot(s) in flight(s) %s" %
                       (fmt(self.agl_limit, unit), len(highs), ", ".join(str(f) for f in sorted({i.flight for i in highs}))))
        if self.unknown:
            out.append("  %d path samples fall outside the DEM (not checked)" % self.unknown)
        if not self.issues:
            out.append("  OK - no clearance or height-limit problems found.")
        for s in self.suggestions:
            out.append("  -> " + s)
        return "\n".join(out)


def _station_z(ground, pt, fallback):
    try:
        return ground(*pt)
    except Exception:
        return fallback


def _flight_path(f, z_st):
    """3D polyline (x, y, z_amsl) the aircraft flies, plus a flag per vertex
    marking the vertical climb/descent at the stations."""
    w = f.waypoints
    first_alt = w[1].alt if len(w) > 2 else w[-1].alt
    pts = [(w[0].x, w[0].y, z_st), (w[0].x, w[0].y, z_st + first_alt)]
    for q in w[1:]:
        pts.append((q.x, q.y, z_st + q.alt))
    pts.append((w[-1].x, w[-1].y, None))         # descent at the landing point (ground)
    return pts


def check_terrain(mission, ground, min_clearance=30.0, max_agl=120.0, step=10.0,
                  station_buffer=60.0, ground_checks=True):
    cam = mission.drone.camera
    z_default = mission.conditions.site_elevation_m
    obs = getattr(mission, "obstacles", None)
    issues, per_flight = [], []
    gmin, gmax = float("inf"), -float("inf")
    samples = unknown = 0
    gsds, aglsurv = [], []
    trig = mission.trigger_m
    for f in mission.flights:
        z_st = _station_z(ground, f.station_start, z_default)
        pts = _flight_path(f, z_st)
        fmin, fmax = float("inf"), -float("inf")
        run = None                                  # (kind, worst issue) of the current problem run
        land = (pts[-1][0], pts[-1][1])
        for (x0, y0, z0), (x1, y1, z1) in zip(pts[1:-2], pts[2:-1]):
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(L / step)))
            for k in range(n + 1):
                t = k / n
                x, y, z = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, z0 + (z1 - z0) * t
                if (G.dist((x, y), f.station_start) < station_buffer or
                        G.dist((x, y), land) < station_buffer):
                    continue                        # climbing out / descending in
                samples += 1
                try:
                    g = ground(x, y)
                except Exception:
                    unknown += 1
                    continue
                c = z - g
                top = obs.top((x, y)) if obs else 0.0
                c_obs = c - top                           # clearance above an obstacle top
                fmin, fmax = min(fmin, c_obs), max(fmax, c)
                if top > 0 and c_obs < obs.margin - 0.5:  # obstacles: checked against their margin
                    kind, c = "obstacle", c_obs
                elif ground_checks:
                    kind = "low" if c < min_clearance else ("high" if c > max_agl else None)
                else:
                    kind = None
                if kind:
                    val = c
                    if run is None or run[0] != kind:
                        run = [kind, TerrainIssue(kind, f.index + 1, x, y, val, g, z)]
                        issues.append(run[1])
                    elif (kind in ("low", "obstacle") and val < run[1].value_m) or (kind == "high" and val > run[1].value_m):
                        run[1].x, run[1].y, run[1].value_m, run[1].ground_m, run[1].alt_amsl = x, y, val, g, z
                else:
                    run = None
        # survey photos: actual height above ground and GSD
        w = f.waypoints
        for a, b in zip(w[1:-2], w[2:-1]):
            if a.photo_dist <= 0:
                continue
            L = G.dist((a.x, a.y), (b.x, b.y))
            n = max(1, int(L / max(trig, 1.0)))
            for k in range(n + 1):
                t = k / n
                x, y = a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t
                try:
                    g = ground(x, y)
                except Exception:
                    continue
                agl = z_st + a.alt + (b.alt - a.alt) * t - g
                if agl > 1:
                    aglsurv.append(agl)
                    gsds.append(P.gsd_from_height(cam, agl))
        if fmin < float("inf"):
            per_flight.append((f.index + 1, fmin, fmax))
            gmin, gmax = min(gmin, fmin), max(gmax, fmax)
    if mission.centreline:
        for i in issues:
            i.kp = mission.chainage_offset + G.locate(mission.centreline, (i.x, i.y))
    rep = TerrainReport(gmin if gmin < float("inf") else float("nan"),
                        gmax if gmax > -float("inf") else float("nan"),
                        min_clearance, max_agl, per_flight, issues, samples=samples, unknown=unknown)
    if gsds:
        rep.gsd_min, rep.gsd_max = min(gsds), max(gsds)
        rep.gsd_mean = sum(gsds) / len(gsds)
        rep.agl_survey_min, rep.agl_survey_max = min(aglsurv), max(aglsurv)
        across, along = P.footprint(cam, rep.agl_survey_min, mission.params.portrait)
        rep.side_overlap_min = max(0.0, 100.0 * (1 - mission.spacing_m / across))
        rep.front_overlap_min = max(0.0, 100.0 * (1 - mission.trigger_m / along))
    # ---- suggestions
    tf = mission.params.terrain_follow
    if any(i.kind == "obstacle" for i in issues):
        rep.suggestions.append("Raise the obstacle margin / buffer or the flying height, or move the launch "
                               "station away from the obstacles (climb-out and landing are vertical).")
    lows = [i for i in issues if i.kind == "low"]
    highs = [i for i in issues if i.kind == "high"]
    if lows:
        worst = min(i.value_m for i in lows)
        if not tf:
            rep.suggestions.append("Turn on terrain following, or raise the flying height by "
                                   "%.0f m." % (min_clearance - worst))
        else:
            rep.suggestions.append("Low points are on transit/return legs: raise the transit height "
                                   "or move the launch station; or raise the flying height by %.0f m."
                                   % (min_clearance - worst))
    if highs:
        rep.suggestions.append("Ground falls away below the flight: turn on terrain following, or "
                               "launch from lower ground nearer the low area." if not tf else
                               "Some transit legs exceed the AGL limit: launch closer to the survey "
                               "or lower the transit height.")
    if rep.side_overlap_min is not None and (rep.side_overlap_min < 50 or rep.front_overlap_min < 60):
        rep.suggestions.append("Overlap drops to %.0f%% front / %.0f%% side over the highest ground - "
                               "use terrain following or raise the overlaps to avoid gaps."
                               % (rep.front_overlap_min, rep.side_overlap_min))
    return rep


# ------------------------------------------------------------ profile
def route_profile(mission, ground, step=None):
    """Ground and flight altitude (AMSL) along a route by chainage.
    Returns rows: dict(kp, ground, flight_min, flight_max, clearance_min)."""
    cl = mission.centreline
    if not cl:
        return []
    total = G.line_length(cl)
    step = step or max(10.0, total / 400.0)
    nb = int(math.ceil(total / step)) + 1
    bins = [dict(kp=mission.chainage_offset + i * step, ground=None, flight_min=None,
                 flight_max=None, clearance_min=None) for i in range(nb)]
    for i, b in enumerate(bins):
        try:
            b["ground"] = ground(*G.point_at(cl, min(i * step, total)))
        except Exception:
            pass
    z_default = mission.conditions.site_elevation_m
    for f in mission.flights:
        z_st = _station_z(ground, f.station_start, z_default)
        w = f.waypoints
        for a, b in zip(w[1:-2], w[2:-1]):
            if a.photo_dist <= 0:
                continue
            L = G.dist((a.x, a.y), (b.x, b.y))
            n = max(1, int(L / (step / 2.0)))
            for k in range(n + 1):
                t = k / n
                x, y = a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t
                z = z_st + a.alt + (b.alt - a.alt) * t
                c = G.locate(cl, (x, y))
                bi = bins[min(nb - 1, int(round(c / step)))]
                bi["flight_min"] = z if bi["flight_min"] is None else min(bi["flight_min"], z)
                bi["flight_max"] = z if bi["flight_max"] is None else max(bi["flight_max"], z)
                try:
                    clr = z - ground(x, y)
                    bi["clearance_min"] = clr if bi["clearance_min"] is None else min(bi["clearance_min"], clr)
                except Exception:
                    pass
    return bins


def flat_ground(mission):
    """Ground model for obstacle checks without a DEM: level with the take-off."""
    z = mission.conditions.site_elevation_m
    return lambda x, y: z


def analyse(mission, ground=None, min_clearance=30.0, max_agl=120.0):
    """Run the checks (and the route profile) and attach them to the mission.
    ground=None: no DEM - ground assumed level with the take-off (obstacle checks only)."""
    flat = ground is None
    if not flat:
        mission.ground = ground                       # footprints use the real ground too
    if flat:
        ground = flat_ground(mission)
    rep = check_terrain(mission, ground, min_clearance, max_agl, ground_checks=not flat)
    mission.terrain = rep
    mission.profile = route_profile(mission, ground) if (mission.centreline and not flat) else []
    obst = [i for i in rep.issues if i.kind == "obstacle"]
    if obst:
        mission.warnings.insert(0, "OBSTACLES: only %.0f m above an obstacle top (limit %.0f m) at %d spot(s)."
                                % (min(i.value_m for i in obst), min_clearance, len(obst)))
    lows = [i for i in rep.issues if i.kind == "low"]
    highs = [i for i in rep.issues if i.kind == "high"]
    if lows:
        mission.warnings.insert(0, "TERRAIN: clearance drops to %.0f m (limit %.0f m) at %d spot(s) - "
                                "see the terrain check." % (min(i.value_m for i in lows), min_clearance, len(lows)))
    if highs:
        mission.warnings.insert(0, "TERRAIN: up to %.0f m above ground (limit %.0f m) at %d spot(s)."
                                % (max(i.value_m for i in highs), max_agl, len(highs)))
    return rep


# ------------------------------------------------------------- SVG chart
def svg_profile(rows, width=920, height=260, title="Route elevation profile", unit="m"):
    pts = [r for r in rows if r["ground"] is not None]
    if len(pts) < 2:
        return ""
    fl = [r for r in rows if r["flight_max"] is not None]
    zs = [r["ground"] for r in pts] + [r["flight_max"] for r in fl] + [r["flight_min"] for r in fl]
    zmin, zmax = min(zs), max(zs)
    pad = max(5.0, (zmax - zmin) * 0.08)
    zmin, zmax = zmin - pad, zmax + pad
    k0, k1 = pts[0]["kp"], pts[-1]["kp"]
    L, R, T, B = 56, 12, 28, 34
    pw, ph = width - L - R, height - T - B

    def X(kp):
        return L + (kp - k0) / max(k1 - k0, 1e-9) * pw

    def Y(z):
        return T + (zmax - z) / max(zmax - zmin, 1e-9) * ph
    ground = " ".join("%.1f,%.1f" % (X(r["kp"]), Y(r["ground"])) for r in pts)
    area = "%.1f,%.1f %s %.1f,%.1f" % (X(k0), T + ph, ground, X(k1), T + ph)
    band_top = " ".join("%.1f,%.1f" % (X(r["kp"]), Y(r["flight_max"])) for r in fl)
    band_bot = " ".join("%.1f,%.1f" % (X(r["kp"]), Y(r["flight_min"])) for r in reversed(fl))
    low = [r for r in fl if r["clearance_min"] is not None and r["clearance_min"] < 30]
    ticks = []
    nt = 6
    for i in range(nt + 1):
        z = zmin + (zmax - zmin) * i / nt
        ticks.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e5e7eb"/>'
                     '<text x="%d" y="%.1f" font-size="10" text-anchor="end" fill="#6b7280">%.0f</text>'
                     % (L, L + pw, Y(z), Y(z), L - 6, Y(z) + 3, z / 0.3048 if unit == "ft" else z))
    span = (k1 - k0) / 1000.0
    kstep = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50][min(range(9), key=lambda i: abs(span / [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50][i] - 8))]
    kk = math.ceil(k0 / 1000.0 / kstep) * kstep
    while kk <= k1 / 1000.0 + 1e-9:
        ticks.append('<text x="%.1f" y="%d" font-size="10" text-anchor="middle" fill="#6b7280">%g</text>'
                     % (X(kk * 1000), T + ph + 14, round(kk, 3)))
        kk += kstep
    marks = "".join('<circle cx="%.1f" cy="%.1f" r="3" fill="#dc2626"/>' % (X(r["kp"]), Y(r["flight_min"]))
                    for r in low)
    return ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="100%%" '
            'style="max-width:%dpx;font-family:sans-serif">'
            '<text x="%d" y="16" font-size="12" font-weight="600" fill="#1f2937">%s</text>%s'
            '<polygon points="%s" fill="#d6c7a1" stroke="#8b6f3e" stroke-width="1.2"/>'
            '<polygon points="%s %s" fill="#0e7490" fill-opacity="0.25" stroke="#0e7490" stroke-width="1"/>'
            '%s<text x="%d" y="%d" font-size="10" text-anchor="middle" fill="#6b7280">KP (km)</text>'
            '<text x="12" y="%d" font-size="10" fill="#6b7280" transform="rotate(-90 12 %d)">%s AMSL</text>'
            '<rect x="%d" y="6" width="10" height="8" fill="#d6c7a1" stroke="#8b6f3e"/>'
            '<text x="%d" y="14" font-size="10" fill="#374151">ground</text>'
            '<rect x="%d" y="6" width="10" height="8" fill="#0e7490" fill-opacity="0.35"/>'
            '<text x="%d" y="14" font-size="10" fill="#374151">flight altitude (all lines)</text></svg>'
            % (width, height, width, L, title, "".join(ticks), area, band_top, band_bot, marks,
               L + pw // 2, height - 4, T + ph // 2 + 20, T + ph // 2 + 20, unit,
               width - 330, width - 316, width - 260, width - 246))
