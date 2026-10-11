import sys, os, math, time
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import terrain as TR, exporters, geometry as G
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
d = get_drone("DJI Mavic 3 Enterprise")
blk = [[(x0, y0), (x0 + 900, y0), (x0 + 900, y0 + 700), (x0, y0 + 700)]]
tk = (x0 - 40, y0 - 40)
print("== 1. min AGL raises flying height")
m = plan_mission(MissionParams(height_m=40, min_agl_m=60), d, Conditions(), blk, tk)
print("  height %.0f m GSD %.2f |" % (m.height_m, m.gsd_cm), [w for w in m.warnings if "raised" in w][0][:90])
print("  report:", [l for l in m.report().splitlines() if l.startswith("Min height")])

print("\n== 2. powerline (28 m) + building (18 m) + tree stand (25 m), flying at 30 m")
line = Obstacle("line", [(x0 - 100, y0 + 350), (x0 + 1000, y0 + 380)], 28, "33 kV powerline")
bld = Obstacle("polygon", [[(x0 + 600, y0 + 100), (x0 + 680, y0 + 100), (x0 + 680, y0 + 160), (x0 + 600, y0 + 160)]], 18, "Shed")
tree = Obstacle("polygon", [[(x0 + 100, y0 + 500), (x0 + 250, y0 + 500), (x0 + 250, y0 + 650), (x0 + 100, y0 + 650)]], 25, "Tree stand")
mast = Obstacle("point", [(x0 + 450, y0 + 600)], 45, "Comms mast")
obs = [line, bld, tree, mast]
m0 = plan_mission(MissionParams(height_m=30), d, Conditions(), blk, tk)
m1 = plan_mission(MissionParams(height_m=30, obstacle_margin_m=10, obstacle_buffer_m=15), d, Conditions(), blk, tk, obstacles=obs)
S = ObstacleSet(obs, 15, 10)
worst = min(w.alt - S.top((w.x, w.y)) for f in m1.flights for w in f.waypoints[1:] if S.top((w.x, w.y)) > 0)
# check the full path between waypoints, every 2 m
def min_clear(mission):
    best = 1e9
    for f in mission.flights:
        w = f.waypoints
        for a, b in zip(w[:-1], w[1:]):
            if G.dist((a.x, a.y), (w[0].x, w[0].y)) < 1 or G.dist((b.x, b.y), (w[-1].x, w[-1].y)) < 1:
                continue                              # vertical climb / descent at the stations
            L = G.dist((a.x, a.y), (b.x, b.y)); n = max(1, int(L / 2))
            for k in range(n + 1):
                q = (a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n); z = a.alt + (b.alt - a.alt) * k / n
                top = S.top(q)
                if top > 0: best = min(best, z - top)
    return best
print("  without obstacles: min clearance over obstacle tops %.1f m (would hit)" % min_clear(m0))
print("  with obstacles   : min clearance %.1f m (margin 10) | waypoints %d -> %d | flights %d -> %d | time %.1f -> %.1f min"
      % (min_clear(m1), len(m0.waypoints), len(m1.waypoints), len(m0.flights), len(m1.flights), m0.total_time_s / 60, m1.total_time_s / 60))
alts = sorted({round(w.alt) for w in m1.waypoints if w.kind.startswith("line")}); print("  survey altitudes used:", alts)
print("  battery all <=100%%: %s" % all(f.battery_fraction <= 1.0001 for f in m1.flights))
r = TR.analyse(m1, None, min_clearance=8)
print("  check (no DEM):", r.text().splitlines()[1], "| issues:", len(r.issues))
r0 = TR.analyse(m0, None, min_clearance=8); print("  check of plan WITHOUT obstacle avoidance:", [l.strip() for l in r0.text().splitlines() if "OBSTACLE" in l.upper()][:1])
print("  report:", [l for l in m1.report().splitlines() if l.startswith(("Min height", "WARNING: Obstacles"))])

print("\n== 3. min AGL with DEM over a hill (no terrain follow)")
ground = lambda x, y: 20 + 50 * math.exp(-((x - (x0 + 450)) / 150.0) ** 2)
m2 = plan_mission(MissionParams(height_m=70, min_agl_m=60), d, Conditions(), blk, tk, ground_elevation=ground)
r2 = TR.analyse(m2, ground, min_clearance=59.5)
print("  lowest AGL %.1f m (min 60) | issues %d | survey alts %s" % (r2.min_clearance_m, len(r2.issues), sorted({round(w.alt) for w in m2.waypoints if w.kind.startswith('line')})[:6]))

print("\n== 4. route + 2000 building footprints (speed)")
import random; random.seed(1)
blds = [Obstacle("polygon", [[(x0 + bx, y0 + by), (x0 + bx + 15, y0 + by), (x0 + bx + 15, y0 + by + 12), (x0 + bx, y0 + by + 12)]], random.choice([6, 9, 12, 20]))
        for bx, by in ((random.uniform(-200, 5200), random.uniform(-300, 300)) for _ in range(2000))]
road = [(x0 + i * 100, y0 + 20 * math.sin(i / 5)) for i in range(51)]
t0 = time.time()
m3 = plan_mission(MissionParams(mission_type=ROUTE, height_m=25, left_width_m=30, right_width_m=30), d, Conditions(), road, (x0, y0 - 60), obstacles=blds)
print("  station warning:", [x for x in m3.warnings if "Launch station" in x][:1])
print("  planned in %.1f s, %d flights, %d waypoints; clearance check:" % (time.time() - t0, len(m3.flights), len(m3.waypoints)), end=" ")
S = ObstacleSet(blds, 15, 10); print("%.1f m min over buildings" % min_clear(m3))
exporters.export(m1, "brief", REPO + "/tests/out/brief_obstacles.html", to84)
print("  brief written")
