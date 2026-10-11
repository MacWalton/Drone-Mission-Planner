import sys, os, math, tempfile, threading, http.server, socketserver, importlib.util, importlib.machinery
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO + "/tests/fake_arcpy")
import arcpy
ROOT = REPO + "/tests/demsrv"
src = open(REPO + "/tests/test_qgis_dem.py").read()
exec(src[src.index("class H("):src.index("import drone_mission_planner as pkg")])
pyt = REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt"
loader = importlib.machinery.SourceFileLoader("dmp_pyt3", pyt); spec = importlib.util.spec_from_loader("dmp_pyt3", loader)
T = importlib.util.module_from_spec(spec); loader.exec_module(T)
T.core.dem.COPERNICUS_URL = base + "/{name}/{name}.tif"; T.core.dem.TERRARIUM_URL = base + "/terrarium/{z}/{x}/{y}.png"
W = arcpy.SpatialReference(4326)
def run(tool, **vals):
    arcpy.MSGS.clear(); ps = tool.getParameterInfo(); d = T._by_name(ps)
    for k, v in vals.items(): d[k].value = v; d[k].altered = True
    tool.updateParameters(ps); tool.updateMessages(ps)
    errs = [(p.name, p.msg) for p in ps if p.msg]
    if errs: print("  validation:", errs)
    tool.execute(ps, None); return d
def truth(lon, lat): return 50 + 3000*(lon-115.70) + 120*math.exp(-(((lon-115.75)/0.01)**2 + ((lat+32.40)/0.01)**2))
gdb = tempfile.mkdtemp()
ring = [(115.738, -32.410), (115.762, -32.410), (115.762, -32.392), (115.738, -32.392), (115.738, -32.410)]
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring]), W)]}
arcpy.INPUTS["tk"] = {"shapeType": "Point", "sr": W, "geoms": [arcpy.PointGeometry(arcpy.Point(115.737, -32.411), W)]}
line = [(115.735 + i * 0.0015, -32.40 + 0.002 * math.sin(i / 3)) for i in range(21)]
arcpy.INPUTS["road"] = {"shapeType": "Polyline", "sr": W, "geoms": [arcpy.Polyline(arcpy.Array([arcpy.Point(*q) for q in line]), W)]}

print("== Download DEM tool (Terrain Tiles)")
out = os.path.join(gdb, "dem_tt.tif")
run(T.DownloadDEM(), in_features="blk", out_raster=out)
from osgeo import gdal, osr; ds = gdal.Open(out); s_ = osr.SpatialReference(wkt=ds.GetProjection()); s_.AutoIdentifyEPSG()
print("  raster %dx%d EPSG:%s | progress steps %d" % (ds.RasterXSize, ds.RasterYSize, s_.GetAuthorityCode(None), sum(1 for e in arcpy.PROGRESS if e[0] == "pos")))
from pyproj import Transformer; tu = Transformer.from_crs(4326, 32750, always_xy=True)
g = T._dem_sampler(out, arcpy.SpatialReference(32750), [tu.transform(115.74, -32.41), tu.transform(115.76, -32.39)])
print("  sample at hill top %.1f (truth %.1f)" % (g(*tu.transform(115.75, -32.40)), truth(115.75, -32.40)))
print("== Download DEM tool (Copernicus, gdal present in this env)")
out2 = os.path.join(gdb, "dem_cop.tif"); run(T.DownloadDEM(), in_features="blk", source="Copernicus GLO-30 (needs GDAL)", out_raster=out2)
print("  ok:", os.path.exists(out2))

print("\n== Area mission, auto-download DEM, flat height -> terrain warnings")
d = run(T.AreaMission(), in_features="blk", takeoff="tk", out_gdb=gdb, size_mode="Flying height", height=60.0, auto_dem="AWS Terrain Tiles")
txt = [m for l, m in arcpy.MSGS]
print("\n".join("  " + m for m in txt if m.startswith(("Terrain check", "  Lowest", "  Highest", "  LOW", "  ABOVE", "  ->", "DEM saved", "WARNING: TERRAIN"))))
print("  out_dem:", os.path.basename(d["out_dem"].value or ""), "| terrain fc rows:", len(arcpy.FCS[d["out_terrain"].value]["rows"]))
print("\n== Same with terrain follow (user DEM)")
d = run(T.AreaMission(), in_features="blk", takeoff="tk", out_gdb=gdb, size_mode="Flying height", height=60.0, dem=out2, terrain=True)
print("\n".join("  " + m for l, m in arcpy.MSGS if m.startswith(("  Lowest", "  Highest", "  OK", "  LOW", "  ABOVE"))))
print("\n== Route mission with DEM -> profile table")
d = run(T.RouteMission(), in_features="road", out_gdb=gdb, height=60.0, size_mode="Flying height", dem=out2, terrain=True)
prof = arcpy.FCS[d["out_profile"].value]["rows"]
print("  profile rows %d, first %s, last %s" % (len(prof), prof[0], prof[-1]))
print("  brief/export untouched; messages incl. profile:", any("Elevation profile table" in m for l, m in arcpy.MSGS))
print("\n== no network -> clean error")
T.core.dem.TERRARIUM_URL = "http://127.0.0.1:9/{z}/{x}/{y}.png"
try: run(T.DownloadDEM(), in_features="blk", out_raster=os.path.join(gdb, "x.tif"))
except arcpy.ExecuteError as e: print("  ExecuteError:", str(e)[:120])
srv.shutdown()
