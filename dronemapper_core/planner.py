# SPDX-License-Identifier: GPL-2.0-only
"""Mission planner: area (grid / double-grid) and route (linear corridor) missions,
with battery-change planning across one or more launch stations."""
import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from . import geometry as G
from . import photogrammetry as P
from .performance import Conditions, EnduranceModel
from .logistics import LogisticsParams, build_schedule
from . import units as U

AREA, ROUTE = "area", "route"


@dataclass
class MissionParams:
    mission_type: str = AREA          # area | route
    mission_name: str = ""            # shown on the brief/report and used for file names
    height_m: Optional[float] = 100.0 # height above launch station (or terrain if DEM)
    gsd_cm: Optional[float] = None    # if given, overrides height_m
    front_overlap_pct: float = 75.0
    side_overlap_pct: float = 65.0
    heading_deg: Optional[float] = None   # None = automatic
    heading_mode: str = "auto"        # auto | manual | wind
    crosshatch: bool = False          # second grid at +90 deg (3D models)
    margin_m: float = 0.0             # extend coverage beyond boundary
    overshoot_m: float = 10.0         # run-in / run-out at line ends
    # ---- route / linear feature
    corridor_width_m: float = 60.0    # used when left/right widths are not given
    left_width_m: Optional[float] = None    # mapped width left of the line (direction of travel)
    right_width_m: Optional[float] = None
    corridor_lines: Optional[int] = None    # force line count
    chainage_start_m: Optional[float] = None   # map only part of a long feature
    chainage_end_m: Optional[float] = None
    simplify_m: float = 0.0           # remove digitising noise (Douglas-Peucker)
    reverse_route: bool = False       # chainage runs from the line's last vertex
    section_length_m: Optional[float] = None   # None = auto (one battery), 0 = no sections
    # ---- flight
    speed_ms: Optional[float] = None  # None = optimise
    portrait: bool = False
    shutter_s: float = 1 / 1000.0
    gimbal_pitch_deg: float = -90.0
    transit_height_m: Optional[float] = None   # None = mission height
    rth_height_m: Optional[float] = None       # final return to take-off/landing (None = flying height)
    battery_height_m: Optional[float] = None   # returns for battery changes and the flight back out
                                               # to resume (None = same as the RTH height)
    height_unit: str = "m"                     # "m" | "ft" - display only, all values are metres
    mass_unit: str = "kg"                      # "kg" | "lb" - display only, all masses are kg
    # ---- batteries
    split_batteries: bool = True
    split_midline: bool = True        # stop mid-line when the battery is due, resume there
    max_range_m: Optional[float] = None   # max distance from launch station (VLOS); None = no limit
    leapfrog: bool = False            # allow landing at a different station (crew moves during flight)
    resume_overlap_m: Optional[float] = None   # re-fly this much before the resume point (None = 1 photo)
    terrain_follow: bool = False      # needs ground_elevation callable
    terrain_step_m: float = 25.0      # waypoint spacing when terrain following
    photo_trigger: str = "distance"            # distance: autopilot triggers every N m
                                               # timed: camera timer (e.g. DJI Fly), speed matched to it
                                               # stop: stop, hover and shoot at every photo position
    stop_hover_s: float = 1.0                  # hover before each photo in 'stop' mode
    min_agl_m: Optional[float] = None          # minimum height above ground anywhere (trees, wires,
                                               # buildings); flying height is raised to it, and with a
                                               # DEM it is kept over rising ground too
    obstacle_buffer_m: float = 15.0            # horizontal distance kept from obstacles
    obstacle_margin_m: float = 10.0            # vertical clearance above obstacle tops
    floor_step_m: float = 10.0                 # sampling step for floors / obstacles
    transit_clearance_m: Optional[float] = None   # with a DEM and no terrain following:
                                                  # clearance on transit legs (None = max(30, h/2))


@dataclass
class Waypoint:
    x: float
    y: float
    alt: float                 # metres above THIS flight's launch station
    kind: str                  # takeoff | line_start | line_end | line_mid | resume | landing
    speed: float = 0.0
    photo_dist: float = 0.0    # >0 => distance-trigger active from this WP
    line_id: int = -1
    flight: int = 0
    shoot: bool = False        # stop-and-shoot photo position (exports only)


@dataclass
class Flight:
    index: int
    waypoints: List[Waypoint]
    duration_s: float
    battery_fraction: float
    distance_m: float
    photos: int
    station_start: Tuple[float, float] = (0.0, 0.0)
    station_end: Tuple[float, float] = (0.0, 0.0)
    z_ref: float = 0.0               # ground elevation of the launch station (DEM)
    ends_midline: bool = False       # battery change happens part-way along a line
    end_reason: str = ""             # "battery" | "range" | "" (finished on a line end)
    chainage: Tuple[float, float] = None   # route: (from, to) metres covered


@dataclass
class Mission:
    params: MissionParams
    drone: object
    conditions: Conditions
    boundary: list                    # rings (area) or corridor polygon
    lines: list                       # survey segments in fly order
    flights: List[Flight]
    takeoff: Tuple[float, float]
    landing: Tuple[float, float]
    height_m: float
    gsd_cm: float
    spacing_m: float
    trigger_m: float
    speed_ms: float
    heading_deg: float
    photos: int
    area_m2: float
    endurance: dict
    warnings: list = field(default_factory=list)
    takeoff_elev: float = 0.0
    stations: list = field(default_factory=list)
    station_labels: dict = field(default_factory=dict)
    schedule: object = None
    centreline: list = None           # route: the line actually mapped
    chainage_offset: float = 0.0      # route: chainage of centreline[0] on the source feature
    sections: list = field(default_factory=list)   # route: [(c0, c1), ...]
    terrain: object = None            # terrain.TerrainReport (set by terrain.analyse)
    profile: list = None              # route elevation profile rows
    rth_height_m: float = None        # metres above take-off
    battery_height_m: float = None
    obstacles: object = None          # obstacles.ObstacleSet
    min_agl_m: float = None

    @property
    def waypoints(self):
        return [w for f in self.flights for w in f.waypoints]

    @property
    def total_time_s(self):
        return sum(f.duration_s for f in self.flights)

    def label(self, pt):
        return self.station_labels.get(tuple(pt), "?")

    def photo_points(self):
        pts = []
        for a, b in self.lines:
            L = G.dist(a, b)
            n = P.photos_for_length(L, self.trigger_m)
            for k in range(n):
                t = min(1.0, k * self.trigger_m / L) if L else 0
                pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        return pts

    def chainage_markers(self, interval=100.0):
        if not self.centreline:
            return []
        return G.chainage_points(self.centreline, interval, self.chainage_offset)

    def trigger_text(self):
        iv = self.drone.camera.min_trigger_interval_s
        return {
            "distance": "every %.1f m (autopilot / app distance trigger)" % self.trigger_m,
            "timed": "camera Timed shot every %g s = %.1f m at %.1f m/s - start it before the mission"
                     % (iv, iv * self.speed_ms, self.speed_ms),
            "stop": "stop, hover %g s and shoot every %.1f m" % (self.params.stop_hover_s, self.trigger_m),
        }.get(self.params.photo_trigger, self.params.photo_trigger)

    def payload_lines(self):
        c, e = self.conditions, self.endurance
        items = c.items
        if not items and not c.payload_kg:
            return []
        mu = getattr(self.params, "mass_unit", "kg")
        out = ["Payload           : %s total, %.0f W accessory power"
               % (U.mass_fmt(c.total_payload_kg, mu), c.accessory_power_w)]
        for it in items:
            out.append("  - %-22s %s%s" % (it.name[:22], U.mass_fmt(it.mass_kg, mu),
                                          ("  %.0f W" % it.power_w) if it.power_w else ""))
        if c.payload_kg:
            out.append("  - %-22s %s" % ("other (unlisted)", U.mass_fmt(c.payload_kg, mu)))
        out.append("  Take-off mass %.2f kg; endurance %.1f min (%.1f min without accessory power)"
                   % (e["take_off_mass_kg"], e["usable_minutes_still_air"],
                      e.get("minutes_without_accessories", e["usable_minutes_still_air"])))
        return out

    def report(self):
        h, m = divmod(int(self.total_time_s // 60), 60)
        lines = [
            "Mission           : %s" % (self.params.mission_name or "(unnamed)"),
            "Mission type      : %s" % self.params.mission_type,
            "Aircraft          : %s" % self.drone.name,
            "Area covered      : %.2f ha" % (self.area_m2 / 10000.0),
        ]
        if self.centreline:
            c0 = self.chainage_offset
            lines.append("Route mapped      : %.2f km (chainage %.3f - %.3f km), %d section(s)"
                         % (G.line_length(self.centreline) / 1000, c0 / 1000,
                            (c0 + G.line_length(self.centreline)) / 1000, len(self.sections)))
        lines += [
            "Flight height     : %s" % U.fmt(self.height_m, self.params.height_unit, 1, True),
            "Min height (AGL)  : %s%s" % (U.fmt(self.min_agl_m, self.params.height_unit, 0, True)
                                          if self.min_agl_m else "not set",
                                          ("; %d obstacle(s), tallest %s" % (len(self.obstacles),
                                           U.fmt(self.obstacles.tallest(), self.params.height_unit, 0, True)))
                                          if self.obstacles else ""),
            "Return-to-home ht : %s above take-off" % U.fmt(self.rth_height_m, self.params.height_unit, 0, True),
            "Battery-change ht : %s (return & back out)" % U.fmt(self.battery_height_m, self.params.height_unit, 0, True),
            "GSD               : %.2f cm/px" % self.gsd_cm,
            "Line spacing      : %.1f m" % self.spacing_m,
            "Photo interval    : %.1f m" % self.trigger_m,
            "Line heading      : %.1f deg" % self.heading_deg,
            "Speed             : %.1f m/s" % self.speed_ms,
            "Wind              : %s from %.0f deg, %.0f C" % (
                U.wind_fmt(self.conditions.wind_speed_ms, getattr(self.conditions, "wind_unit", "kn")),
                self.conditions.wind_from_deg, self.conditions.temperature_c),
            "Photo trigger     : %s" % self.trigger_text(),
            "Photos (approx.)  : %d" % self.photos,
            "Survey legs       : %d" % len(self.lines),
            "Total flight time : %dh %02dm" % (h, m),
            "Flights/batteries : %d   Launch stations: %d" % (len(self.flights), len(self.stations)),
            "Usable endurance  : %.1f min per battery (after %.0f%% reserve)"
            % (self.endurance["usable_minutes_still_air"], self.conditions.reserve_pct),
        ]
        lines += self.payload_lines()
        for f in self.flights:
            extra = ""
            if f.chainage:
                extra = "  KP %.2f-%.2f" % (f.chainage[0] / 1000, f.chainage[1] / 1000)
            lines.append("  Flight %d: %s->%s %.1f min, %.0f%% battery, %d WPs, %d photos%s%s"
                         % (f.index + 1, self.label(f.station_start), self.label(f.station_end),
                            f.duration_s / 60, f.battery_fraction * 100, len(f.waypoints), f.photos,
                            ("  (ends mid-line: %s)" % f.end_reason) if f.ends_midline else "", extra))
        if self.terrain is not None:
            lines.append("")
            lines.append(self.terrain.text(self.params.height_unit))
        if self.schedule is not None and len(self.flights) > 1:
            lines.append("")
            lines.append(self.schedule.table())
        for w in self.warnings:
            lines.append("WARNING: " + w)
        return "\n".join(lines)


# ======================================================================
def _resolve_height(params, camera):
    if params.gsd_cm:
        h = P.height_from_gsd(camera, params.gsd_cm)
    else:
        h = float(params.height_m or 100.0)
    return h, P.gsd_from_height(camera, h)


def _order_segments(lines, start):
    """Boustrophedon order: walk lines in sequence, alternating direction;
    begin from the end of the block nearest the start point."""
    flat_lines = [list(l) for l in lines]
    if not flat_lines:
        return []

    def ends(line):
        return [line[0][0], line[-1][1]]
    d_first = min(G.dist(start, p) for p in ends(flat_lines[0]))
    d_last = min(G.dist(start, p) for p in ends(flat_lines[-1]))
    if d_last < d_first:
        flat_lines = list(reversed(flat_lines))
    out = []
    pos = start
    for segs in flat_lines:
        if G.dist(pos, segs[-1][1]) < G.dist(pos, segs[0][0]):
            segs = [(b, a) for (a, b) in reversed(segs)]
        out.extend(segs)
        pos = segs[-1][1]
    return out


def _order_polylines(polylines, start):
    """Snake through a block of parallel polylines, starting with the outer
    line nearest `start`; each line flown in the direction nearest to us."""
    if not polylines:
        return [], start
    d_first = min(G.dist(start, polylines[0][0]), G.dist(start, polylines[0][-1]))
    d_last = min(G.dist(start, polylines[-1][0]), G.dist(start, polylines[-1][-1]))
    block = list(reversed(polylines)) if d_last < d_first else list(polylines)
    out, pos = [], start
    for pl in block:
        if G.dist(pos, pl[-1]) < G.dist(pos, pl[0]):
            pl = list(reversed(pl))
        out.extend(zip(pl[:-1], pl[1:]))
        pos = pl[-1]
    return out, pos


def _pick_speed(params, drone, model, height, heading, cam):
    warnings = []
    v_cam, why = P.max_speed_for_camera(cam, height, params.front_overlap_pct,
                                        params.shutter_s, portrait=params.portrait)
    v_cap = min(drone.max_speed_ms, v_cam)
    if params.speed_ms:
        speed = params.speed_ms
        if speed > v_cap + 1e-6:
            warnings.append("Speed %.1f m/s is above the %.1f m/s limit (%s)."
                            % (speed, v_cap, why if v_cam < drone.max_speed_ms else "aircraft"))
        return speed, warnings
    v_min = 2.0
    if drone.fixed_wing:
        v_min = 0.6 * drone.max_speed_ms          # stay well above stall
        if v_cap < v_min:
            warnings.append("Camera limits speed to %.1f m/s (%s) but a fixed-wing must fly "
                            ">= %.1f m/s: use a faster shutter, lower overlap or fly higher."
                            % (v_cap, why, v_min))
            v_cap = v_min
    speed = model.best_speed([heading] + ([heading + 90] if params.crosshatch else []),
                             max(v_cap, v_min), v_min)
    return speed, warnings


def _split_by_station(a, b, stations, overlap, max_range=None):
    """Cut a survey segment where the owning launch station changes: the
    nearest station that can reach the point (within max_range), else the
    nearest. Returns [(station, a', b')] with `overlap` metres shared at cuts."""
    L = G.dist(a, b)
    if L < 1e-9:
        return []

    def owner(t):
        q = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        ks = range(len(stations))
        if max_range:
            ok = [k for k in ks if G.dist(q, stations[k]) <= max_range * 0.98]
            ks = ok or ks
        return min(ks, key=lambda k: G.dist(q, stations[k]))
    n = max(2, int(L / 5.0))
    cuts, prev = [], owner(0.0)
    for k in range(1, n + 1):
        t = k / n
        o = owner(t)
        if o != prev:
            lo, hi = (k - 1) / n, t
            for _ in range(25):
                mid = (lo + hi) / 2
                if owner(mid) == prev:
                    lo = mid
                else:
                    hi = mid
            cuts.append(((lo + hi) / 2, prev))
            prev = o
    cuts.append((1.0, prev))
    out, t0 = [], 0.0
    half = overlap / 2.0 / L
    for t1, own in cuts:
        s0 = max(0.0, t0 - (half if t0 > 0 else 0))
        s1 = min(1.0, t1 + (half if t1 < 1 else 0))
        if (s1 - s0) * L > 1.0:
            out.append((stations[own], (a[0] + (b[0] - a[0]) * s0, a[1] + (b[1] - a[1]) * s0),
                        (a[0] + (b[0] - a[0]) * s1, a[1] + (b[1] - a[1]) * s1)))
        t0 = t1
    return out


def _make_zones(groups, launch, overlap, max_range=None):
    """Assign line pieces to their nearest (reachable) launch station."""
    zones = {s: {} for s in launch}
    for li, line in enumerate(groups):
        for a, b in line:
            for st, a2, b2 in _split_by_station(a, b, launch, overlap, max_range):
                zones[st].setdefault(li, []).append((a2, b2))
    return zones


def _greedy_zone_order(zones, launch):
    todo = [s for s in launch if zones[s]]
    order, cur = [], launch[0]
    while todo:
        nxt = min(todo, key=lambda s: G.dist(cur, s))
        order.append(nxt)
        todo.remove(nxt)
        cur = nxt
    return order


class _Order(list):
    """Ordered survey segments plus the launch station that owns each one."""
    owners = None


def _zone_order(zones, order, continuous=False):
    """Fly zone by zone in the given station order, lines snaking within each
    zone. continuous=True enters each zone from where the last one ended."""
    ordered, pos, owners = _Order(), None, []
    for st in order:
        blocks = [zones[st][li] for li in sorted(zones[st])]
        segs = _order_segments(blocks, pos if (ordered and continuous) else st)
        ordered += segs
        owners += [st] * len(segs)
        pos = segs[-1][1]
    ordered.owners = owners
    return ordered


def prepare_centreline(params, line):
    """Apply simplification and the chainage window to a linear feature.
    Returns (centreline, chainage_offset_m)."""
    pts = [tuple(p[:2]) for p in line]
    if params.reverse_route:
        pts = list(reversed(pts))
    centre = G.simplify(pts, params.simplify_m)
    total = G.line_length(centre)
    c0 = params.chainage_start_m or 0.0
    c1 = params.chainage_end_m if params.chainage_end_m else total
    if c0 > 0 or c1 < total:
        if c1 - c0 < 1.0:
            raise ValueError("Chainage window %.0f-%.0f m is empty (feature is %.0f m long)."
                             % (c0, c1, total))
        centre = G.substring(centre, c0, c1)
    return centre, max(0.0, min(c0, total))


def corridor_widths(params):
    left = params.left_width_m if params.left_width_m is not None else params.corridor_width_m / 2.0
    right = params.right_width_m if params.right_width_m is not None else params.corridor_width_m / 2.0
    return max(0.0, left), max(0.0, right)


def route_layout(params, drone, conditions, line):
    """Work out corridor lines and battery-sized sections for a linear feature.

    Returns dict(centre, offset, offsets, spacing, sections, height, speed, heading, warnings).
    """
    cam = drone.camera
    height, _ = _resolve_height(params, cam)
    spacing = P.line_spacing(cam, height, params.side_overlap_pct, params.portrait)
    centre, c_off = prepare_centreline(params, line)
    left, right = corridor_widths(params)
    width = left + right
    n = params.corridor_lines or max(1, int(math.ceil(width / spacing)) if width > 0 else 1)
    if n == 1:
        offsets = [(left - right) / 2.0]
    else:
        offsets = [-right + width / (2 * n) + i * width / n for i in range(n)]
        spacing = width / n
    heading = G.azimuth(centre[0], centre[-1])
    model = EnduranceModel(drone, conditions, height)
    speed, warns = _pick_speed(params, drone, model, height, heading, cam)
    total = G.line_length(centre)
    sec = params.section_length_m
    if sec is None:
        if params.split_batteries:
            # line-metres one battery can fly, spread over the n parallel lines
            per_battery = model.usable_minutes * 60.0 * speed * 0.8
            sec = per_battery / n
        else:
            sec = 0
    if params.max_range_m:
        # a station mid-section must reach both section ends and the outer lines
        lateral = max(abs(o) for o in offsets) + params.overshoot_m
        reach = math.sqrt(max(params.max_range_m ** 2 - lateral ** 2, 1.0)) * 0.9
        sec = min(sec, 2.0 * reach) if sec else 2.0 * reach
    if not sec or sec >= total * 0.98:
        sections = [(0.0, total)]
    else:
        k = max(1, int(math.ceil(total / sec)))
        s = total / k
        sections = [(i * s, (i + 1) * s) for i in range(k)]
    return dict(centre=centre, offset=c_off, offsets=offsets, spacing=spacing, sections=sections,
                height=height, speed=speed, heading=heading, warnings=warns, left=left, right=right,
                model=model)


def suggest_stations(params, drone, conditions, line, side_offset_m=0.0):
    """Launch stations at the middle of each battery-sized section of a route
    (offset sideways by side_offset_m, + = left). Drag them to accessible spots."""
    lay = route_layout(params, drone, conditions, line)
    out = []
    for c0, c1 in lay["sections"]:
        mid = (c0 + c1) / 2.0
        if side_offset_m:
            piece = G.substring(lay["centre"], max(0, mid - 1), mid + 1)
            p = G.offset_polyline(piece, side_offset_m)
            out.append(G.point_at(p, G.line_length(p) / 2.0))
        else:
            out.append(G.point_at(lay["centre"], mid))
    return out


def suggest_area_stations(rings, max_range_m, inset=0.85):
    """Launch stations on a grid over a large area so every point is within
    max_range_m of one of them. Each station sits inside the boundary
    (centre of the part of its grid cell that lies inside). Drag them to
    accessible spots and re-plan."""
    cell = max_range_m * math.sqrt(2.0) * inset          # square cell whose half-diagonal fits R
    xs = [p[0] for p in rings[0]]
    ys = [p[1] for p in rings[0]]
    nx = max(1, int(math.ceil((max(xs) - min(xs)) / cell)))
    ny = max(1, int(math.ceil((max(ys) - min(ys)) / cell)))
    cw, ch = (max(xs) - min(xs)) / nx, (max(ys) - min(ys)) / ny
    out = []
    for i in range(nx):
        for j in range(ny):
            x0, y0 = min(xs) + i * cw, min(ys) + j * ch
            inside = [(x0 + (u + 0.5) * cw / 7, y0 + (v + 0.5) * ch / 7)
                      for u in range(7) for v in range(7)]
            inside = [q for q in inside if G.point_in_polygon(q, rings)]
            if not inside:
                continue
            cx = sum(q[0] for q in inside) / len(inside)
            cy = sum(q[1] for q in inside) / len(inside)
            out.append(min(inside, key=lambda q: (q[0] - cx) ** 2 + (q[1] - cy) ** 2))
    return out


# ======================================================================
def plan_mission(params: MissionParams, drone, conditions: Conditions,
                 geometry, takeoff, landing=None,
                 ground_elevation: Optional[Callable] = None,
                 stations=None, logistics: Optional[LogisticsParams] = None,
                 obstacles=None) -> Mission:
    """Build a mission.

    geometry  : polygon rings (area) or a polyline [(x,y),...] (route), in metres
    takeoff   : first launch station (x, y)
    landing   : optional final landing point (defaults to the nearest station)
    stations  : extra launch stations for battery changes along big sites / long routes
    ground_elevation : optional f(x, y) -> metres AMSL (terrain following & per-station heights)
    logistics : battery sets / chargers / timing for the field schedule
    obstacles : list of obstacles.Obstacle (powerlines, buildings, trees) to stay above
    """
    from .obstacles import ObstacleSet
    cam = drone.camera
    warnings = []
    height, gsd = _resolve_height(params, cam)
    min_agl = params.min_agl_m or 0.0
    if min_agl and height < min_agl - 1e-6:
        warnings.append("Flying height raised from %s to the %s minimum height above ground; GSD is now "
                        "%.2f cm/px instead of %.2f." % (U.fmt(height, params.height_unit, 0, True),
                                                       U.fmt(min_agl, params.height_unit, 0, True),
                                                       P.gsd_from_height(cam, min_agl), gsd))
        height = min_agl
        gsd = P.gsd_from_height(cam, height)
    obs = ObstacleSet(obstacles or [], params.obstacle_buffer_m, params.obstacle_margin_m)
    spacing = P.line_spacing(cam, height, params.side_overlap_pct, params.portrait)
    trig = P.trigger_distance(cam, height, params.front_overlap_pct, params.portrait)
    model = EnduranceModel(drone, conditions, height)
    warnings += model.warnings

    launch = [tuple(takeoff)] + [tuple(s) for s in (stations or []) if tuple(s) != tuple(takeoff)]
    land_final = tuple(landing) if landing else None
    land_pts = launch + ([land_final] if land_final and land_final not in launch else [])
    centreline, c_off, sections = None, 0.0, []

    alt_orders = []                    # other candidate orderings to try
    # ---- coverage lines
    if params.mission_type == ROUTE:
        lay = route_layout(params, drone, conditions, geometry)
        centreline, c_off, sections = lay["centre"], lay["offset"], lay["sections"]
        spacing, heading, speed = lay["spacing"], lay["heading"], lay["speed"]
        warnings += lay["warnings"]
        left, right = lay["left"], lay["right"]
        if left + right > 0:
            ring = G.offset_polyline(centreline, left) + list(reversed(G.offset_polyline(centreline, -right)))
        else:
            ring = G.offset_polyline(centreline, 5) + list(reversed(G.offset_polyline(centreline, -5)))
        boundary = [ring + [ring[0]]]
        area = G.polygon_area(boundary)
        ordered = []
        seg_chain = []                 # chainage of each ordered segment (for reporting)
        forward = G.dist(launch[0], centreline[0]) <= G.dist(launch[0], centreline[-1])
        pos = centreline[0] if forward else centreline[-1]
        for (s0, s1) in (sections if forward else list(reversed(sections))):
            piece = G.substring(centreline, s0, s1)
            pls = [G.extend_polyline(G.offset_polyline(piece, off),
                                     params.overshoot_m, params.overshoot_m)
                   for off in lay["offsets"]]
            segs, pos = _order_polylines(pls, pos)
            ordered += segs
            seg_chain += [(c_off + s0, c_off + s1)] * len(segs)
    else:
        rings = [list(r) for r in geometry]
        boundary = rings
        area = G.polygon_area(rings)
        if params.heading_mode == "manual" and params.heading_deg is not None:
            heading = params.heading_deg % 180.0
        elif params.heading_mode == "wind" and conditions.wind_speed_ms > 0:
            heading = conditions.wind_from_deg % 180.0   # fly into / with the wind
        else:
            heading = G.min_width_heading(rings)
        speed, w = _pick_speed(params, drone, model, height, heading, cam)
        warnings += w
        groups = G.scan_segments(rings, heading, spacing, params.overshoot_m, params.margin_m)
        cross = G.scan_segments(rings, heading + 90.0, spacing, params.overshoot_m,
                                params.margin_m) if params.crosshatch else []
        if len(launch) > 1:
            zones = _make_zones(groups + cross, launch, trig, params.max_range_m)
            greedy = _greedy_zone_order(zones, launch)
            ordered = _zone_order(zones, greedy)
            alt_orders.append(_zone_order(zones, greedy, continuous=True))
            if 2 < len(greedy) <= 4:            # few stations: try every visiting order
                import itertools
                for perm in itertools.permutations(greedy):
                    if list(perm) != greedy:
                        alt_orders.append(_zone_order(zones, list(perm)))
                        alt_orders.append(_zone_order(zones, list(perm), continuous=True))
        else:
            ordered = _order_segments(groups, launch[0])
            if cross:
                ordered += _order_segments(cross, ordered[-1][1] if ordered else launch[0])
        seg_chain = [None] * len(ordered)

    if not ordered:
        raise ValueError("No flight lines could be generated - check the boundary and parameters.")
    transit_h = params.transit_height_m or height
    rth_h = params.rth_height_m or transit_h
    batt_h = params.battery_height_m or rth_h
    out_h = params.battery_height_m             # flight back out after a battery change
    if max(rth_h, batt_h) > 120:
        warnings.append("Return / battery-change height %s exceeds the 120 m (400 ft) limit used in "
                        "most jurisdictions." % U.fmt(max(rth_h, batt_h), params.height_unit, 0, True))

    # ---- heights: relative to each flight's launch station
    terrain_misses = [0]
    use_dem = bool(ground_elevation)

    def ground(p, fallback):
        try:
            return ground_elevation(*p)
        except Exception:
            return fallback

    z_main = conditions.site_elevation_m
    if use_dem:
        try:
            z_main = ground_elevation(*launch[0])
        except Exception:
            warnings.append("Take-off point is outside the DEM; using the entered take-off "
                            "elevation (%.0f m) as the height reference." % z_main)
    station_z = {s: (ground(s, z_main) if use_dem else 0.0) for s in land_pts}
    if not use_dem:
        station_z = {s: 0.0 for s in land_pts}

    def alt_at(x, y, zref):
        if params.terrain_follow and use_dem:
            try:
                return height + ground_elevation(x, y) - zref
            except Exception:
                terrain_misses[0] += 1
                return height
        return height

    def seg_wps(a, b, lid, photo, zref, first_kind="line_start"):
        pts = [a, b]
        if params.terrain_follow and use_dem:
            pts = G.densify(pts, params.terrain_step_m)
        out = []
        for i, p in enumerate(pts):
            kind = first_kind if i == 0 else ("line_end" if i == len(pts) - 1 else "line_mid")
            out.append(Waypoint(p[0], p[1], alt_at(p[0], p[1], zref), kind, speed,
                                photo if i < len(pts) - 1 else 0.0, lid))
        return out

    # ---- battery-limited flights
    def leg_cost(a, b, v):
        t, f, _ = model.leg(a, b, v)
        return t, f

    clear_m = (params.transit_clearance_m if params.transit_clearance_m is not None
               else max(30.0, 0.5 * height))

    def _gnd(x, y):
        try:
            return ground_elevation(x, y)
        except Exception:
            return None

    def _max_ground(a, b, step=20.0):
        L = G.dist(a, b)
        n = max(1, int(math.ceil(L / step)))
        vals = [_gnd(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]
        vals = [v for v in vals if v is not None]
        return max(vals) if vals else None

    def _follow(a, b, zref, a_alt, b_alt, floor=0.0):
        """Terrain-following transit waypoints strictly between a and b
        (never below `floor` above the launch station)."""
        out = []
        pts = G.densify([a, b], params.terrain_step_m)[1:-1]
        for q in pts:
            g = _gnd(*q)
            alt = height + g - zref if g is not None else max(a_alt, b_alt)
            out.append(Waypoint(q[0], q[1], max(alt, floor), "transit", ret_speed))
        return out

    def _safe_transits(wps, zref, out_floor=0.0, home_floor=0.0):
        """With a DEM, keep the climb-out and the flight home clear of the ground.
        Terrain following: transits follow the ground at the flying height.
        Otherwise: climb just enough to clear the highest ground by clear_m.
        Returns extra (seconds, battery fraction) for the climbing."""
        extra_t = extra_f = 0.0
        st_wp, first = wps[0], wps[1]
        last, land_wp = wps[-2], wps[-1]
        if params.terrain_follow:
            # drop the flat "set height" transits; terrain following replaces them (with floors)
            while len(wps) > 2 and wps[1].kind == "transit":
                del wps[1]
            while len(wps) > 2 and wps[-2].kind == "transit":
                del wps[-2]
            first, last = wps[1], wps[-2]
            g_st = _gnd(st_wp.x, st_wp.y)
            out = _follow((st_wp.x, st_wp.y), (first.x, first.y), zref, height, first.alt, out_floor)
            climb_to = max((height + g_st - zref) if g_st is not None else height, out_floor)
            wps[1:1] = [Waypoint(st_wp.x, st_wp.y, climb_to, "transit", ret_speed)] + out
            g_l = _gnd(land_wp.x, land_wp.y)
            if g_l is not None:
                land_wp.alt = max(height + g_l - zref, home_floor)
            if home_floor > last.alt + 1.0:
                wps.insert(len(wps) - 1, Waypoint(last.x, last.y, home_floor, "transit", ret_speed))
                last = wps[-2]
            home = _follow((last.x, last.y), (land_wp.x, land_wp.y), zref, last.alt, land_wp.alt,
                           home_floor)
            wps[len(wps) - 1:len(wps) - 1] = home
            ups = [max(0.0, b_.alt - a_.alt) for a_, b_ in zip(wps[:-1], wps[1:]) if b_.kind == "transit"]
            if ups:
                ct, cf = model.climb(sum(ups))
                extra_t, extra_f = ct, cf
            return extra_t, extra_f
        if wps[1].kind == "transit" and len(wps) > 3:        # already flying out at a set height
            first = next(w for w in wps[1:] if w.kind != "transit")
        g = _max_ground((st_wp.x, st_wp.y), (first.x, first.y))
        if g is not None:
            safe = g - zref + clear_m
            if safe > max(first.alt, wps[1].alt) + 1.0:
                wps[1:1] = [Waypoint(st_wp.x, st_wp.y, safe, "transit", ret_speed),
                            Waypoint(first.x, first.y, safe, "transit", ret_speed)]
                ct, cf = model.climb(safe - first.alt)
                dt, df = model.climb(-(safe - first.alt))
                extra_t += ct + dt
                extra_f += cf + df
        g = _max_ground((last.x, last.y), (land_wp.x, land_wp.y))
        if g is not None:
            safe = g - zref + clear_m
            if safe > min(last.alt, land_wp.alt) + 1.0:
                if safe > last.alt + 1.0:
                    wps.insert(len(wps) - 1, Waypoint(last.x, last.y, safe, "transit", ret_speed))
                    ct, cf = model.climb(safe - last.alt)
                    extra_t += ct
                    extra_f += cf
                land_wp.alt = max(land_wp.alt, safe)
        return extra_t, extra_f

    has_floor = bool(obs) or (min_agl > 0 and use_dem)
    floor_hits = {"obstacle": 0, "ground": 0}

    def floor_at(x, y, zref):
        """Lowest allowed altitude (m above the launch station) at x, y."""
        g = None
        if use_dem:
            try:
                g = ground_elevation(x, y) - zref
            except Exception:
                g = None
        base = g if g is not None else 0.0
        need = 0.0
        if min_agl and g is not None:
            need = base + min_agl
        if obs:
            r = obs.required_agl((x, y))
            if r > 0 and base + r > need:
                need = base + r
                floor_hits["obstacle"] += 1
        return need

    def _apply_floor(wps, zref):
        """Raise any part of the flight that would go below the floor (minimum
        height / obstacles), adding waypoints only where the altitude changes.
        Returns the extra (seconds, battery fraction) for climbing."""
        step = params.floor_step_m
        # the aircraft climbs vertically at the launch point before flying off
        if len(wps) > 1 and wps[0].kind == "takeoff" and G.dist((wps[0].x, wps[0].y), (wps[1].x, wps[1].y)) > 0.5:
            wps.insert(1, Waypoint(wps[0].x, wps[0].y, wps[1].alt, "transit", wps[1].speed))
        out = [wps[0]]
        extra_up = 0.0
        for a, b in zip(wps[:-1], wps[1:]):
            L = G.dist((a.x, a.y), (b.x, b.y))
            if L < 1e-6:                               # vertical move: never below the floor
                if b.kind != "landing":
                    b.alt = max(b.alt, floor_at(b.x, b.y, zref))
                out.append(b)
                continue
            n = max(1, int(math.ceil(L / step)))
            ts = {k / n for k in range(n + 1)}
            if obs:
                ts.update(round(t_, 6) for t_ in obs.segment_params((a.x, a.y), (b.x, b.y)))
            samples = []
            for t_ in sorted(ts):
                x, y = a.x + (b.x - a.x) * t_, a.y + (b.y - a.y) * t_
                base_alt = a.alt + (b.alt - a.alt) * t_
                samples.append((t_, x, y, base_alt, max(base_alt, floor_at(x, y, zref))))
            # climb before reaching a raised floor (and stay up until past it): each sample
            # takes the highest floor of itself and its neighbours, so the straight line
            # between kept points never cuts the corner of an obstacle zone
            fl = [s_[4] for s_ in samples]
            dil = [max(fl[max(0, k - 1):k + 2]) for k in range(len(fl))]
            samples = [(s_[0], s_[1], s_[2], s_[3], max(s_[3], dl)) for s_, dl in zip(samples, dil)]
            if all(s_[4] <= s_[3] + 0.5 for s_ in samples):
                out.append(b)
                continue
            if a.kind != "takeoff" and samples[0][4] > a.alt + 0.5:
                # start the climb at the previous waypoint (e.g. a line end before a turn
                # towards a powerline); raising it only lifts the previous leg's end
                extra_up += samples[0][4] - a.alt
                a.alt = samples[0][4]
            # keep the raised profile, thinned (Douglas-Peucker) so the straight legs between
            # kept points are never more than 0.3 m below it (or more than 5 m above it)
            prof = samples
            keep = {0, len(prof) - 1}
            stack = [(0, len(prof) - 1)]
            while stack:
                i0, i1 = stack.pop()
                worst, wk = 0.0, None
                for k in range(i0 + 1, i1):
                    f = (prof[k][0] - prof[i0][0]) / max(prof[i1][0] - prof[i0][0], 1e-9)
                    line = prof[i0][4] + (prof[i1][4] - prof[i0][4]) * f
                    dev = prof[k][4] - line                # > 0: profile above the straight leg
                    score = dev / 0.3 if dev > 0 else -dev / 5.0
                    if score > worst:
                        worst, wk = score, k
                if wk is not None and worst > 1.0:
                    keep.add(wk)
                    stack += [(i0, wk), (wk, i1)]
            keep = sorted(keep)
            survey = a.photo_dist > 0
            for k in keep[1:]:
                t_, x, y, base_alt, alt = prof[k]
                if k == len(prof) - 1:
                    b.alt = max(b.alt, alt)
                    out.append(b)
                else:
                    out.append(Waypoint(x, y, alt, "line_mid" if survey else "transit", a.speed,
                                        a.photo_dist if survey else 0.0, a.line_id))
            ups = [max(0.0, prof[k2][4] - prof[k1][4]) for k1, k2 in zip(keep[:-1], keep[1:])]
            extra_up += sum(ups)
        wps[:] = out
        if extra_up > 0:
            ct, cf = model.climb(extra_up)
            dt, df = model.climb(-extra_up)
            return ct + dt, cf + df
        return 0.0, 0.0

    # stop-and-shoot: decelerate, hover, shoot, accelerate at every photo position
    stop_s = 0.0
    if params.photo_trigger == "stop":
        stop_s = params.stop_hover_s + 1.0 + 2.0 * speed / 2.5     # hover + shutter + brake/accelerate

    def survey_cost(a, b):
        tb, fb = leg_cost(a, b, speed)
        if stop_s:
            n = P.photos_for_length(G.dist(a, b), trig)
            tb += n * stop_s
            fb += model.hover(n * stop_s)
        return tb, fb

    turn_t, turn_f = (0.0, 0.0) if drone.fixed_wing else (4.0, model.hover(4.0))
    climb_t, climb_f = model.climb(max(transit_h, out_h or 0.0))
    if out_h and out_h > height:                  # come back down to survey height
        ct_, cf_ = model.climb(-(out_h - height))
        climb_t, climb_f = climb_t + ct_, climb_f + cf_
    ret_max = max(rth_h, batt_h)
    desc_t, desc_f = model.climb(-ret_max)
    if ret_max > height:                          # climb to the return height before heading home
        ct_, cf_ = model.climb(ret_max - height)
        desc_t, desc_f = desc_t + ct_, desc_f + cf_
    ret_speed = min(drone.max_speed_ms, max(speed, 0.7 * drone.max_speed_ms))

    home = [launch[0]]                  # set per flight: where this flight may land

    def best_return(p):
        best = None
        for s in home:
            t, f = leg_cost(p, s, ret_speed)
            if best is None or f < best[1]:
                best = (t, f, s)
        return best

    resume_back = trig if params.resume_overlap_m is None else params.resume_overlap_m
    min_part = max(2.0 * trig, 20.0)
    R = params.max_range_m or 0.0

    def lerp(a, b, t):
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    def range_t(a, b, st):
        """Largest t in [0,1] with a + t(b-a) within R of st (a must be inside)."""
        if not R:
            return 1.0
        dx, dy = b[0] - a[0], b[1] - a[1]
        fx, fy = a[0] - st[0], a[1] - st[1]
        A = dx * dx + dy * dy
        if A < 1e-12:
            return 1.0
        B = 2 * (fx * dx + fy * dy)
        C = fx * fx + fy * fy - R * R
        disc = B * B - 4 * A * C
        if disc < 0:
            return 0.0
        return max(0.0, min(1.0, (-B + math.sqrt(disc)) / (2 * A)))

    def choose_station(seg, owner=None):
        p = seg[0]
        cands = [s_ for s_ in launch if not R or G.dist(s_, p) <= R + 1e-6]
        L_seg = G.dist(seg[0], seg[1])
        useful = [s_ for s_ in cands if range_t(seg[0], seg[1], s_) >= 1.0 - 1e-9
                  or range_t(seg[0], seg[1], s_) * L_seg >= min_part]
        cands = useful or cands
        if owner is not None and owner in useful:
            return owner                       # the station whose zone this is
        if not cands:
            near = min(launch, key=lambda s_: G.dist(s_, p))
            raise ValueError(
                "Part of the survey is %.2f km from the nearest launch station (S%d) - beyond the "
                "%.0f m range limit. Add a launch station near it or use 'Auto-place stations'."
                % (G.dist(near, p) / 1000.0, launch.index(near) + 1, R))
        best = min(cands, key=lambda s_: (round(G.dist(s_, p), 1), leg_cost(s_, p, ret_speed)[1]))
        if prev_station[0] in cands and prev_station[0] != best:
            # stay where the crew already is unless another station is clearly closer
            if G.dist(prev_station[0], p) <= 1.3 * G.dist(best, p) + 100.0:
                return prev_station[0]
        return best

    prev_station = [None]

    def build(order_in):
        warnings = []
        prev_station[0] = None
        queue = list(order_in)
        flights = []
        i = 0
        while i < len(queue):
            q_snap, i_snap, cap = list(queue), i, 1.0
            good = None                       # last attempt that built a flight
            for attempt in range(6):
                queue[:] = q_snap
                i = i_snap
                try:
                    owners = getattr(order_in, "owners", None)
                    st = choose_station(queue[i], owners[i] if owners else None)
                    prev_station[0] = st
                    home[:] = land_pts if params.leapfrog else [st]
                    zref = station_z.get(st, 0.0)
                    wps = [Waypoint(st[0], st[1], 0.0, "takeoff", ret_speed)]
                    t, frac, d, photos = climb_t, climb_f, 0.0, 0
                    pos, start_i, reason, midline = st, i, "", False
                    resumed = bool(flights) and flights[-1].ends_midline

                    def take(a, b, last_kind=None):
                        nonlocal t, frac, d, photos, pos
                        ta, fa = leg_cost(pos, a, ret_speed)
                        tb, fb = survey_cost(a, b)
                        t += ta + tb + turn_t
                        frac += fa + fb + turn_f
                        d += G.dist(pos, a) + G.dist(a, b)
                        photos += P.photos_for_length(G.dist(a, b), trig)
                        kind = "resume" if (i == start_i and resumed) else "line_start"
                        wps.extend(seg_wps(a, b, i, trig, zref, kind))
                        pos = b

                    while i < len(queue):
                        a, b = queue[i]
                        L = G.dist(a, b)
                        if R and G.dist(st, a) > R + 1e-6:
                            reason = reason or "range"
                            break                                   # next flight picks a nearer station
                        t_lim = range_t(a, b, st)
                        p_lim = lerp(a, b, t_lim)
                        fa = leg_cost(pos, a, ret_speed)[1]
                        need = frac + fa + survey_cost(a, p_lim)[1] + turn_f + best_return(p_lim)[1] + desc_f
                        fits = (not params.split_batteries) or need <= cap
                        if fits and t_lim >= 1.0 - 1e-9:
                            take(a, b)
                            i += 1
                            continue
                        empty = frac <= climb_f + 1e-9
                        if fits:                                    # range limit reached part-way
                            cut_t, reason = t_lim, "range"
                        elif params.split_midline:                  # battery limit part-way
                            lo, hi = 0.0, t_lim
                            for _ in range(30):
                                mid = (lo + hi) / 2
                                q = lerp(a, b, mid)
                                if frac + fa + survey_cost(a, q)[1] + turn_f + best_return(q)[1] + desc_f <= cap:
                                    lo = mid
                                else:
                                    hi = mid
                            cut_t, reason = lo, "battery"
                        else:
                            cut_t, reason = 0.0, "battery"
                        cut = cut_t * L
                        if cut >= min_part or (empty and cut > 1.0 and reason == "range"):
                            p = lerp(a, b, cut_t)
                            take(a, p)
                            back = min(resume_back, cut * 0.5)
                            queue[i] = (lerp(a, b, cut_t - back / L), b)   # resume here with photo overlap
                            midline = True
                            break
                        if empty:
                            if reason == "range":
                                raise ValueError("Launch station S%d cannot reach the next part of the survey "
                                                 "within the %.0f m range limit - add a station closer or use "
                                                 "'Auto-place stations'." % (launch.index(st) + 1, R))
                            if params.split_midline:
                                raise ValueError(
                                    "A fresh battery cannot reach the survey from the nearest launch station "
                                    "and return (segment %d). Add a launch station closer to it, lower the "
                                    "reserve or check the wind/payload settings." % (i + 1))
                            warnings.append("A single flight line needs more than one battery; enable "
                                            "'split mid-line' or add launch stations.")
                            take(a, b)
                            i += 1
                        break
                    if i >= len(queue) and land_final:
                        end = land_final
                        tr, fr = leg_cost(pos, end, ret_speed)
                    else:
                        tr, fr, end = best_return(pos)
                    t += tr + desc_t
                    frac += fr + desc_f
                    d += G.dist(pos, end)
                    z_end = station_z.get(end, zref) if use_dem else zref
                    final = i >= len(queue)
                    ret_alt = rth_h if final else batt_h
                    if len(wps) > 1:
                        last = wps[-1]
                        up = max(ret_alt, last.alt)        # never descend over high ground
                        if up > last.alt + 1.0:
                            wps.append(Waypoint(last.x, last.y, up, "transit", ret_speed))
                    land_alt = ret_alt + (z_end - zref if params.terrain_follow else 0.0)
                    wps.append(Waypoint(end[0], end[1], land_alt, "landing", ret_speed))
                    if out_h and flights and len(wps) > 2 and wps[1].kind != "transit":
                        first = wps[1]                     # battery-change flight: out at set height
                        if abs(out_h - first.alt) > 1.0:
                            wps[1:1] = [Waypoint(st[0], st[1], out_h, "transit", ret_speed),
                                        Waypoint(first.x, first.y, out_h, "transit", ret_speed)]
                    if use_dem and len(wps) > 2:
                        ct, cf = _safe_transits(wps, zref, out_floor=(out_h if flights else 0.0) or 0.0,
                                                home_floor=ret_alt)
                        t += ct
                        frac += cf
                    if has_floor and len(wps) > 2:
                        ct, cf = _apply_floor(wps, zref)
                        t += ct
                        frac += cf
                except ValueError:
                    if good is None:
                        raise
                    (queue[:], i, wps, t, frac, d, photos, st, end, zref, midline, reason) = good
                    break
                good = (list(queue), i, wps, t, frac, d, photos, st, end, zref, midline, reason)
                # safe-transit climbing is only known once the flight is built:
                # if it pushed the flight over one battery, shrink the budget and redo it
                if ((use_dem or has_floor) and params.split_batteries and frac > 1.0 + 1e-6 and attempt < 5):
                    cap -= (frac - 1.0) + 0.005
                    continue
                break
            fl = Flight(len(flights), wps, t, frac, d, photos, st, end, zref, midline,
                        reason if (midline or i < len(queue)) else "")
            if centreline:
                surv = [(w.x, w.y) for w in wps if w.kind in ("line_start", "line_mid", "line_end", "resume")]
                if surv:
                    cs = [c_off + G.locate(centreline, q) for q in surv]
                    fl.chainage = (min(cs), max(cs))
            for w in wps:
                w.flight = fl.index
            flights.append(fl)
            if frac > 1.0 + 1e-6 and params.split_batteries:
                warnings.append("Flight %d needs %.0f%% of a battery." % (fl.index + 1, frac * 100))
            if len(flights) > 2000:
                raise ValueError("Mission needs more than 2000 flights - check the parameters.")
        return flights, warnings

    flights, w_build = build(ordered)
    for alt in alt_orders:
        try:
            f2, w2 = build(alt)
        except ValueError:
            continue
        def score(fl):
            moves = sum(1 for a_, b_ in zip(fl[:-1], fl[1:]) if a_.station_end != b_.station_start)
            return (len(fl), moves, sum(f.duration_s for f in fl))
        if score(f2) < score(flights):
            flights, w_build, ordered = f2, w2, alt
    warnings += w_build

    # ---- warnings
    if obs:
        warnings.append("Obstacles: %d feature(s), tallest %s; the aircraft keeps %s above them within "
                        "%s horizontally." % (len(obs), U.fmt(obs.tallest(), params.height_unit, 0, True),
                                              U.fmt(params.obstacle_margin_m, params.height_unit),
                                              U.fmt(params.obstacle_buffer_m, params.height_unit)))
        over = obs.tallest() + params.obstacle_margin_m
        if over > 120:
            warnings.append("Clearing the tallest obstacle needs %s above ground - above the usual "
                            "120 m (400 ft) limit." % U.fmt(over, params.height_unit, 0, True))
    if obs:
        near = [lbl for st, lbl in ((s_, "S%d" % (k + 1)) for k, s_ in enumerate(launch)) if obs.top(st) > 0]
        if near:
            warnings.append("Launch station(s) %s are within %s of an obstacle - the aircraft climbs "
                            "vertically there; move them to open ground." % (", ".join(near),
                                                                            U.fmt(params.obstacle_buffer_m, params.height_unit)))
    if min_agl and not use_dem and params.mission_type:
        warnings.append("Minimum height above ground is applied to the flying height; add a DEM so it is "
                        "also kept where the ground rises above the launch point.")
    if terrain_misses[0]:
        warnings.append("%d waypoint(s) fall outside the DEM and use the flat height - "
                        "check clearance there." % terrain_misses[0])
    if params.terrain_follow and use_dem:
        lows = [w for f in flights for w in f.waypoints if w.kind.startswith("line") and w.alt < 5]
        if lows:
            warnings.append("Terrain drops below a launch station by more than the flying height; "
                            "%d waypoints would be < 5 m above it." % len(lows))
    if len(land_pts) > 1 and not use_dem:
        warnings.append("Heights are relative to each flight's launch station. Without a DEM the "
                        "ground is assumed level between stations - add a DEM if they differ.")
    max_wp = max(len(f.waypoints) for f in flights)
    if max_wp > drone.max_waypoints:
        warnings.append("Flights have up to %d waypoints; %s supports %d per mission - "
                        "exporters will split files automatically."
                        % (max_wp, drone.name, drone.max_waypoints))
    if height > 120:
        warnings.append("Height %.0f m AGL exceeds the 120 m (400 ft) limit used in most "
                        "jurisdictions (CASA, FAA, EASA) without an approval." % height)
    far = [f.index + 1 for f in flights
           if max(G.dist(f.station_start, (w.x, w.y)) for w in f.waypoints[:-1]) > (R + 1 if R else 2000)]
    if far and not R:
        warnings.append("Flight(s) %s go more than 2 km from the launch station - check visual "
                        "line of sight rules; add launch stations to keep flights closer."
                        % ", ".join(map(str, far[:10])))

    total_photos = sum(f.photos for f in flights)
    mission = Mission(params, drone, conditions, boundary, ordered, flights, tuple(launch[0]),
                      land_final or tuple(launch[0]), height, gsd, spacing, trig, speed, heading,
                      total_photos, area, model.summary(), warnings, z_main if use_dem else 0.0,
                      stations=launch, centreline=centreline, chainage_offset=c_off,
                      sections=[(c_off + a, c_off + b) for a, b in sections])
    mission.rth_height_m, mission.battery_height_m = rth_h, batt_h
    mission.obstacles = obs
    mission.min_agl_m = min_agl or None
    sched, labels = build_schedule(flights, launch, logistics or LogisticsParams(), land_final)
    mission.schedule, mission.station_labels = sched, labels
    if sched.waiting_min > 1 and len(flights) > 1:
        warnings.append("Charging is the bottleneck: %.0f min waiting in total; %d battery sets "
                        "would avoid it." % (sched.waiting_min, sched.recommended_sets))
    return mission


def default_mission_name(mission_type=AREA, source_name=None, date=None):
    """'Lot 12 boundary - area mission 11 Oct 2026' or 'Area mission 11 Oct 2026'."""
    import time as _t
    d = date or _t.strftime("%d %b %Y")
    kind = "route mission" if mission_type == ROUTE else "area mission"
    src = (source_name or "").strip()
    for junk in ("Mission boundary", "Route centreline", "probe"):
        if src == junk:
            src = ""
    if src:
        return "%s - %s %s" % (src, kind, d)
    return "%s %s" % (kind.capitalize(), d)


def safe_filename(name, fallback="mission"):
    import re as _re
    s = _re.sub(r"[^\w\-. ]+", "", name or "").strip().replace(" ", "_")
    return s[:80] or fallback
