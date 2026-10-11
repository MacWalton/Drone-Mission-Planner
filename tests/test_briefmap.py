import sys, os, math, random, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, terrain as TR, geometry as G
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
d = get_drone("DJI Mavic 3 Enterprise")
out = REPO + "/tests/out"; os.makedirs(out, exist_ok=True)
# A: area, 2 stations, ridge -> terrain warnings, named
ground = lambda x, y: 20 + 85 * math.exp(-((x - (x0 + 900)) / 160.0) ** 2)
blk = [[(x0, y0), (x0 + 1800, y0), (x0 + 1800, y0 + 900), (x0 + 900, y0 + 1300), (x0, y0 + 900)]]
mA = plan_mission(MissionParams(height_m=80, mission_name="Lot 12 Stockpile Survey"), d, Conditions(wind_speed_ms=5), blk, (x0 - 60, y0 - 60), stations=[(x0 + 1860, y0 + 400)])
TR.analyse(mA, ground)
exporters.export(mA, "brief", os.path.join(out, "brief_area.html"), to84)
# B: 18 km route, 11 stations, unnamed -> filename used
pts = [(x0 + i * 200, y0 + 900 * math.sin(i / 14.0)) for i in range(91)]
mp = MissionParams(mission_type=ROUTE, gsd_cm=2.0, left_width_m=40, right_width_m=25, max_range_m=1000)
sts = suggest_stations(mp, d, Conditions(), pts, -60)
mB = plan_mission(mp, d, Conditions(), pts, sts[0], stations=sts[1:])
exporters.export(mB, "brief", os.path.join(out, "brief_route.html"), to84)
print("A flights", len(mA.flights), "terrain issues", len(mA.terrain.issues), "| B flights", len(mB.flights))
print("defaults:", default_mission_name(), "|", default_mission_name(ROUTE, "Gas pipeline KP0-18"), "|", safe_filename("Lot 12: Stockpile/Survey?"))
for f in ("brief_area.html", "brief_route.html"):
    h = open(os.path.join(out, f)).read()
    print(f, "| map:", "<h2>Mission map</h2>" in h and "<svg" in h, "| title:", h[h.index("<h1>") + 4:h.index("</h1>")])
