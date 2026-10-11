import sys, os, random, math, tempfile, json, zipfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, geometry as G, photogrammetry as P
from pyproj import Transformer
import xml.dom.minidom as md
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
d = get_drone("DJI Mavic 3 Enterprise")
lp = LogisticsParams(battery_sets=3, chargers=2, charge_min=70, swap_min=4)

print("=== A. 400 ha block, 2 launch stations, mid-line battery changes")
big = [[(x0, y0), (x0+2000, y0), (x0+2000, y0+2000), (x0, y0+2000)]]
m = plan_mission(MissionParams(gsd_cm=2.5), d, Conditions(wind_speed_ms=5, wind_from_deg=200, temperature_c=30),
                 big, (x0-50, y0-50), stations=[(x0+2050, y0+2050)], logistics=lp)
print(m.report())
mids = sum(f.ends_midline for f in m.flights)
# coverage check: union of flown survey length == planned survey length (+ overlaps)
flown = sum(G.dist((a.x,a.y),(b.x,b.y)) for f in m.flights for a,b in zip(f.waypoints[1:-2], f.waypoints[2:-1]) if a.photo_dist>0)
planned = sum(G.dist(a,b) for a,b in m.lines)
print("\nmid-line changes:", mids, "| survey length planned %.1f km, flown %.1f km (incl. resume overlaps)" % (planned/1000, flown/1000))
assert flown >= planned - 1, "coverage gap!"
assert all(f.battery_fraction <= 1.0001 for f in m.flights), [f.battery_fraction for f in m.flights]
used = {m.label(f.station_start) for f in m.flights}; print("stations used:", used)

print("\n=== B. same block, split at line ends only (old behaviour)")
m2 = plan_mission(MissionParams(gsd_cm=2.5, split_midline=False), d, Conditions(wind_speed_ms=5, wind_from_deg=200, temperature_c=30),
                  big, (x0-50, y0-50), stations=[(x0+2050, y0+2050)], logistics=lp)
print("flights mid-line split: %d (%.0f min) vs whole lines: %d (%.0f min)" % (len(m.flights), m.total_time_s/60, len(m2.flights), m2.total_time_s/60))

print("\n=== C. 25 km pipeline from 9 shuffled/reversed pieces, chainage window, L/R widths, auto stations")
pts = [(x0 + i*250, y0 + 600*math.sin(i/12.0) + 30*math.sin(i*1.7)) for i in range(101)]  # wiggly ~25 km
pieces = [pts[i:i+12] for i in range(0, 100, 11)]
random.seed(3); random.shuffle(pieces); pieces = [list(reversed(p)) if random.random()<0.5 else p for p in pieces]
line, gap = G.chain_lines(pieces)
print("chained %d pieces -> %.2f km, largest gap %.1f m, starts at x=%.0f" % (len(pieces), G.line_length(line)/1000, gap, line[0][0]))
assert abs(G.line_length(line) - G.line_length(pts)) < 1
mp = MissionParams(mission_type=ROUTE, gsd_cm=2.0, left_width_m=50, right_width_m=20, chainage_start_m=2500, chainage_end_m=20500, simplify_m=5, max_range_m=1000)
stations = suggest_stations(mp, d, Conditions(), line, side_offset_m=-40)
print("auto stations:", len(stations))
mr = plan_mission(mp, d, Conditions(wind_speed_ms=6, wind_from_deg=270), line, stations[0], stations=stations[1:], logistics=lp)
print(mr.report())
kms = mr.chainage_markers(1000); print("chainage markers:", len(kms), "first", round(kms[0][2]), "last", round(kms[-1][2]))
far = max(max(G.dist(f.station_start,(w.x,w.y)) for w in f.waypoints) for f in mr.flights); print("max distance from launch station: %.0f m" % far); assert far <= 1001
flown = sum(G.dist((a.x,a.y),(b.x,b.y)) for f in mr.flights for a,b in zip(f.waypoints[1:-2], f.waypoints[2:-1]) if a.photo_dist>0); print("route survey planned %.1f km flown %.1f km" % (sum(G.dist(a,b) for a,b in mr.lines)/1000, flown/1000)); assert flown >= sum(G.dist(a,b) for a,b in mr.lines)-1
# left/right asymmetry: lines offsets
lay = route_layout(mp, d, Conditions(), line); print("offsets (m):", [round(o,1) for o in lay["offsets"]])

print("\n=== D. exports incl. brief; multi-station homes")
out = tempfile.mkdtemp()
for fmt in exporters.FORMATS:
    exporters.export(mr, fmt, os.path.join(out, "pipe" + exporters.FORMATS[fmt][1] if fmt!="csv" else "pipe_wp.csv"), to84)
files = sorted(os.listdir(out)); print(len(files), "files, e.g.", files[:4], "...")
homes = set()
for f in files:
    if f.endswith(".plan"):
        homes.add(tuple(round(v,5) for v in json.load(open(os.path.join(out,f)))["mission"]["plannedHomePosition"][:2]))
print("distinct home positions in .plan files:", len(homes))
for f in files:
    if f.endswith(".kmz"):
        z=zipfile.ZipFile(os.path.join(out,f)); [md.parseString(z.read(n)) for n in z.namelist()]
html=open(os.path.join(out,"pipe.html")).read(); print("brief html bytes", len(html), "has schedule:", "Battery" in html and "S2" in html)
os.makedirs(REPO + "/tests/out", exist_ok=True)
import shutil; shutil.copy(os.path.join(out,"pipe.html"), REPO + "/tests/out/field_brief_example.html")

print("\n=== E. DEM: per-station height reference")
dem = lambda x, y: 0.02*(x-x0)    # ground rises 20 m per km eastwards
mt = plan_mission(mp, d, Conditions(), line, stations[0], stations=stations[1:], ground_elevation=dem)
print("station z_refs:", [round(f.z_ref,1) for f in mt.flights][:8], "(no terrain follow: line alts", {round(w.alt,1) for w in mt.waypoints if w.kind.startswith('line')}, ")")
mp.terrain_follow=True
mt = plan_mission(mp, d, Conditions(), line, stations[0], stations=stations[1:], ground_elevation=dem)
for f in mt.flights[:3]:
    a=[w.alt for w in f.waypoints if w.kind.startswith("line")]; print("  flight %d alt above its station %.1f..%.1f" % (f.index+1, min(a), max(a)))
print("\n=== E2. 400 ha block with 1 km range limit + auto-placed stations")
R=1000
sts = suggest_area_stations(big, R)
ma = plan_mission(MissionParams(gsd_cm=2.5, max_range_m=R), d, Conditions(wind_speed_ms=5, wind_from_deg=200), big, sts[0], stations=sts[1:], logistics=LogisticsParams(battery_sets=6))
far = max(max(G.dist(f.station_start,(w.x,w.y)) for w in f.waypoints) for f in ma.flights)
reasons = {}
for f in ma.flights: reasons[f.end_reason or "done"] = reasons.get(f.end_reason or "done",0)+1
print("stations", len(sts), "flights", len(ma.flights), "max dist from launch %.0f m" % far, "end reasons", reasons)
flown = sum(G.dist((a.x,a.y),(b.x,b.y)) for f in ma.flights for a,b in zip(f.waypoints[1:-2], f.waypoints[2:-1]) if a.photo_dist>0)
print("survey planned %.1f km flown %.1f km" % (sum(G.dist(a,b) for a,b in ma.lines)/1000, flown/1000))
assert far <= R + 1 and flown >= sum(G.dist(a,b) for a,b in ma.lines) - 1
print(ma.schedule.table())
try:
    plan_mission(MissionParams(gsd_cm=2.5, max_range_m=R), d, Conditions(), big, sts[0])
except ValueError as e: print("single station + range -> OK:", str(e)[:110])

print("\n=== F. unreachable survey -> clear error")
try:
    plan_mission(MissionParams(gsd_cm=2.5), d, Conditions(), big, (x0-30000, y0))
except ValueError as e: print("OK:", e)
print("\n=== G. leapfrog option (land at next station)")
mp.terrain_follow=False; mp.leapfrog=True
ml = plan_mission(mp, d, Conditions(), line, stations[0], stations=stations[1:])
print("leapfrog: %d flights, %.0f min flying, land stations differ in %d flights; drive time %.0f min (vs %.0f without)" % (
    len(ml.flights), ml.total_time_s/60, sum(f.station_end!=f.station_start for f in ml.flights), ml.schedule.relocating_min, mr.schedule.relocating_min))
