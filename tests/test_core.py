import sys, os, json, zipfile, math, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, geometry as G, photogrammetry as P
from pyproj import Transformer
import xml.dom.minidom as md

t = Transformer.from_crs(32750, 4326, always_xy=True)
to84 = lambda x, y: t.transform(x, y)
# L-shaped (concave) block with a hole, UTM 50S near Rockingham
x0, y0 = 381000, 6426000
poly = [[(x0,y0),(x0+600,y0),(x0+600,y0+250),(x0+250,y0+250),(x0+250,y0+500),(x0,y0+500),(x0,y0)],
        [(x0+50,y0+50),(x0+120,y0+50),(x0+120,y0+120),(x0+50,y0+120)]]
print("area ha", G.polygon_area(poly)/1e4)
d = get_drone("DJI Mavic 3 Enterprise")
cam = d.camera
print("GSD@100m", P.gsd_from_height(cam,100), "h for 2cm", P.height_from_gsd(cam,2.0))
for cond in [Conditions(), Conditions(wind_speed_ms=8, wind_from_deg=250, temperature_c=38, site_elevation_m=20, payload_kg=0.1)]:
    m = plan_mission(MissionParams(gsd_cm=2.5), d, cond, poly, (x0-30, y0-30))
    print(m.report()); print()
# every segment lies inside polygon (midpoint test), excluding overshoot
for a,b in m.lines:
    mid=((a[0]+b[0])/2,(a[1]+b[1])/2)
    assert G.point_in_polygon(mid, [poly[0]]), mid
# manual heading + crosshatch
m2 = plan_mission(MissionParams(height_m=80, heading_mode="manual", heading_deg=30, crosshatch=True), d, Conditions(), poly, (x0,y0))
print("crosshatch lines", len(m2.lines), "flights", len(m2.flights))
# route corridor
road = [(x0,y0),(x0+1500,y0+400),(x0+3000,y0+300)]
mr = plan_mission(MissionParams(mission_type=ROUTE, height_m=60, corridor_width_m=80), d, Conditions(), road, (x0+10,y0-20), landing=(x0+3000,y0+320))
print(mr.report())
# terrain follow
mt = plan_mission(MissionParams(height_m=100, terrain_follow=True), d, Conditions(), poly, (x0,y0), ground_elevation=lambda x,y: 0.05*(x-x0))
alts=[w.alt for w in mt.waypoints if w.kind.startswith("line")]; print("terrain alt range", min(alts), max(alts))
# fixed wing & mavlink
fw = get_drone("ArduPilot / PX4 Fixed-wing (generic)")
mf = plan_mission(MissionParams(height_m=110), fw, Conditions(wind_speed_ms=6), poly, (x0,y0))
print("fixed wing speed", mf.speed_ms, "min", mf.total_time_s/60)

out = tempfile.mkdtemp()
for fmt,(label,ext) in exporters.FORMATS.items():
    files = exporters.export(m, fmt, os.path.join(out,"m"+ext if fmt!="csv" else "m_wp.csv"), to84)
    print(fmt, [os.path.basename(f) for f in files])
# validations
plan = json.load(open(os.path.join(out,"m_flight01.plan") if len(m.flights)>1 else os.path.join(out,"m.plan")))
print("plan items", len(plan["mission"]["items"]), plan["mission"]["plannedHomePosition"])
kz = sorted(f for f in os.listdir(out) if f.endswith(".kmz"))[0]
z = zipfile.ZipFile(os.path.join(out,kz)); 
for n in z.namelist(): md.parseString(z.read(n)); print(n, "XML ok", len(z.read(n)))
md.parse(os.path.join(out,"m.kml")); print("kml ok")
print(open(sorted(os.path.join(out,f) for f in os.listdir(out) if f.endswith(".waypoints"))[0]).read()[:400])
json.dump({"out":out}, open(REPO + "/tests/out.json","w"))
