import sys, os, math, random, importlib.util, importlib.machinery, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO + "/tests/fake_arcpy")
import arcpy
pyt = REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt"
loader = importlib.machinery.SourceFileLoader("dmp_pyt2", pyt); spec = importlib.util.spec_from_loader("dmp_pyt2", loader)
T = importlib.util.module_from_spec(spec); loader.exec_module(T)
W = arcpy.SpatialReference(4326)
def run(tool, **vals):
    arcpy.MSGS.clear(); ps = tool.getParameterInfo(); d = T._by_name(ps)
    for k, v in vals.items(): d[k].value = v; d[k].altered = True
    tool.updateParameters(ps); tool.updateMessages(ps)
    errs = [(p.name, p.msg) for p in ps if p.msg]
    if errs: print("  validation:", errs)
    tool.execute(ps, None); return d
# pipeline: 7 shuffled reversed pieces (as separate features, like a GIS road layer)
pts = [(115.70 + i * 0.0015, -32.40 + 0.01 * math.sin(i / 9.0)) for i in range(91)]
pieces = [pts[i:i + 14] for i in range(0, 90, 13)]; random.seed(7); random.shuffle(pieces)
pieces = [list(reversed(pc)) if k % 2 else pc for k, pc in enumerate(pieces)]
arcpy.INPUTS["pipe"] = {"shapeType": "Polyline", "sr": W, "geoms": [arcpy.Polyline(arcpy.Array([arcpy.Point(*q) for q in pc]), W) for pc in pieces]}
gdb = tempfile.mkdtemp(); out = tempfile.mkdtemp()
print("== Route: chained pieces, KP 1.5-11, L/R 40/25, 800 m VLOS, auto stations, schedule + brief")
d = run(T.RouteMission(), in_features="pipe", out_gdb=gdb, gsd=2.0, left_width=40.0, right_width=25.0,
        kp_from=1.5, kp_to=11.0, max_range=800.0, auto_stations=True, station_offset=-60.0, sets=3, charge_min=70.0,
        start_time="07:30", export_folder=out, formats=["Field brief & battery plan (.html)", "DJI WPML (.kmz)"])
for l, m in arcpy.MSGS:
    if not m.startswith("Exported") and not m.startswith("  ") or "Flight " in m or "Flt" in m: print(" ", l, m)
sched = arcpy.FCS[d["out_schedule"].value]; print("  schedule rows:", len(sched["rows"]), "first:", sched["rows"][0][:8])
site = arcpy.FCS[d["out_site"].value]["rows"]; print("  site:", [(r[1], r[2], r[3]) for r in site][:4], "...")
ch = arcpy.FCS[d["out_chainage"].value]["rows"]; print("  chainage markers:", len(ch), ch[0][1], "->", ch[-1][1])
rt = arcpy.FCS[d["out_routes"].value]["rows"][0]; print("  route row:", rt[1:])
print("  exports:", sorted(os.listdir(out))[:3], "... (%d files)" % len(os.listdir(out)))

print("\n== Area: 400 ha, user stations as multi-point input, no range, 6 sets")
x0, y0 = 115.80, -32.40
blk = [(x0, y0), (x0 + 0.0212, y0), (x0 + 0.0212, y0 + 0.018), (x0, y0 + 0.018), (x0, y0)]
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in blk]), W)]}
arcpy.INPUTS["sts"] = {"shapeType": "Point", "sr": W, "geoms": [arcpy.PointGeometry(arcpy.Point(*q), W) for q in [(x0 - 0.0005, y0 - 0.0005), (x0 + 0.0217, y0 + 0.0185)]]}
d = run(T.AreaMission(), in_features="blk", takeoff="sts", out_gdb=gdb, gsd=2.5, sets=6)
print("\n".join("  " + m for l, m in arcpy.MSGS if m.startswith(("Flights/", "  Flight", "  On site"))))
print("\n== Area: auto stations without range -> warning + 1000 m grid")
d = run(T.AreaMission(), in_features="blk", out_gdb=gdb, gsd=2.5, auto_stations=True)
print("\n".join("  " + l + " " + m for l, m in arcpy.MSGS if m.startswith(("Flights/", "Auto-placed", "Range limit"))))
print("\n== Route: no stations, range too short -> clean error")
try:
    run(T.RouteMission(), in_features="pipe", out_gdb=gdb, max_range=500.0)
except arcpy.ExecuteError as e:
    print("  ExecuteError:", e)
