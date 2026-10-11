import sys, math, os, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import terrain as TR, exporters, geometry as G
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
d = get_drone("DJI Mavic 3 Enterprise")
# a ridge 80 m high running N-S through the middle of the block, ground 20 m elsewhere
def ground(x, y):
    return 20 + 80 * math.exp(-((x - (x0 + 500)) / 120.0) ** 2)
blk = [[(x0, y0), (x0 + 1000, y0), (x0 + 1000, y0 + 600), (x0, y0 + 600)]]
print("== 1. flat-height plan over a ridge (100 m AGL at take-off)")
m = plan_mission(MissionParams(height_m=100), d, Conditions(), blk, (x0 - 50, y0 - 50))
rep = TR.analyse(m, ground)
print(rep.text())
lows = [i for i in rep.issues if i.kind == "low"]
print("  low spots near the ridge crest:", all(abs(i.x - (x0 + 500)) < 150 for i in lows), "| first warning:", m.warnings[0][:70])
print("\n== 2. same with terrain following (planner given the DEM)")
m2 = plan_mission(MissionParams(height_m=100, terrain_follow=True), d, Conditions(), blk, (x0 - 50, y0 - 50), ground_elevation=ground)
rep2 = TR.analyse(m2, ground)
print(rep2.text())
print("\n== 3. valley: ground drops 90 m below the launch point -> above 120 m AGL")
def valley(x, y): return 100 - 90 * math.exp(-((x - (x0 + 500)) / 150.0) ** 2)
m3 = plan_mission(MissionParams(height_m=100), d, Conditions(), blk, (x0 - 50, y0 - 50))
rep3 = TR.analyse(m3, valley)
print("\n".join(rep3.text().splitlines()[1:3] + [l for l in rep3.text().splitlines() if "ABOVE" in l or "->" in l]))
print("\n== 4. route over a hill with 2 stations: profile + brief")
road = [(x0 + i * 100, y0 + 30 * math.sin(i / 3)) for i in range(41)]       # 4 km
def hill(x, y): return 30 + 60 * math.exp(-((x - (x0 + 2000)) / 500.0) ** 2) + 0.004 * (x - x0)
mp = MissionParams(mission_type=ROUTE, height_m=80, left_width_m=30, right_width_m=30, terrain_follow=True, max_range_m=1500)
sts = suggest_stations(mp, d, Conditions(), road)
mr = plan_mission(mp, d, Conditions(), road, sts[0], stations=sts[1:], ground_elevation=hill)
TR.analyse(mr, hill)
prof = mr.profile
clr = [r["clearance_min"] for r in prof if r["clearance_min"] is not None]
print("  profile rows %d, KP %.2f-%.2f km, ground %.0f-%.0f m, clearance over lines %.1f-%.1f m"
      % (len(prof), prof[0]["kp"] / 1000, prof[-1]["kp"] / 1000, min(r["ground"] for r in prof), max(r["ground"] for r in prof), min(clr), max(clr)))
out = tempfile.mkdtemp(); exporters.export(mr, "brief", os.path.join(out, "route.html"), to84)
html = open(os.path.join(out, "route.html")).read()
print("  brief has terrain section:", "Terrain (DEM)" in html, "| svg profile:", "<svg" in html and "ground" in html)
os.makedirs(REPO + "/tests/out", exist_ok=True)
open(REPO + "/tests/out/profile.svg", "w").write(TR.svg_profile(prof))
import shutil; shutil.copy(os.path.join(out, "route.html"), REPO + "/tests/out/route_brief.html")
print("\n== 5. route over the hill WITHOUT terrain follow -> low clearance at the crest with KP")
mp.terrain_follow = False
mr2 = plan_mission(mp, d, Conditions(), road, sts[0], stations=sts[1:])
r5 = TR.analyse(mr2, hill, min_clearance=40)
print("  issues:", [(i.kind, i.flight, round(i.kp / 1000, 2), round(i.value_m)) for i in r5.issues][:6])
