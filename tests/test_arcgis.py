import sys, os, importlib.util, importlib.machinery, tempfile, json, zipfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0,REPO + "/tests/fake_arcpy")
import arcpy
pyt=REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt"
loader=importlib.machinery.SourceFileLoader("dmp_pyt",pyt); spec=importlib.util.spec_from_loader("dmp_pyt",loader)
T=importlib.util.module_from_spec(spec); loader.exec_module(T)
from pyproj import Transformer
tb=T.Toolbox(); print("tools:", [t().label for t in tb.tools])
to7850=Transformer.from_crs(4326,7850,always_xy=True)
def poly4326(ll):
    return arcpy.Polygon(arcpy.Array([arcpy.Point(*p) for p in ll]), arcpy.SpatialReference(4326))
ll=[(115.735,-32.300),(115.741,-32.300),(115.741,-32.2975),(115.738,-32.2975),(115.738,-32.295),(115.735,-32.295),(115.735,-32.300)]
arcpy.INPUTS["boundary"]={"shapeType":"Polygon","sr":arcpy.SpatialReference(4326),"geoms":[poly4326(ll)]}
arcpy.INPUTS["tk"]={"shapeType":"Point","sr":arcpy.SpatialReference(4326),"geoms":[arcpy.PointGeometry(arcpy.Point(115.7345,-32.3005),arcpy.SpatialReference(4326))]}
# CAD closed polyline in MGA2020 z50
cad=[to7850.transform(*p) for p in ll]
arcpy.INPUTS["cad"]={"shapeType":"Polyline","sr":arcpy.SpatialReference(7850),"geoms":[arcpy.Polyline(arcpy.Array([arcpy.Point(*p) for p in cad]),arcpy.SpatialReference(7850))]}
arcpy.INPUTS["road"]={"shapeType":"Polyline","sr":arcpy.SpatialReference(4326),"geoms":[arcpy.Polyline(arcpy.Array([arcpy.Point(*p) for p in [(115.73,-32.31),(115.75,-32.305),(115.77,-32.307)]]),arcpy.SpatialReference(4326))]}

def run(tool, **vals):
    arcpy.MSGS.clear()
    ps=tool.getParameterInfo(); d=T._by_name(ps)
    for k,v in vals.items(): d[k].value=v; d[k].altered=True
    tool.updateParameters(ps); tool.updateMessages(ps)
    errs=[(p.name,p.msg) for p in ps if p.msg]
    if errs: print("  validation:", errs)
    tool.execute(ps, None)
    return d
gdb=tempfile.mkdtemp()
out=tempfile.mkdtemp(); dev=tempfile.mkdtemp()
print("\n== Area (drawn polygon, GSD mode, exports + device copy)")
d=run(T.AreaMission(), in_features="boundary", takeoff="tk", out_gdb=gdb, gsd=2.0, wind=7.0, wind_dir=250.0, temp=35.0,
      export_folder=out, formats=[v[0] for v in T.exporters.FORMATS.values()], device_folder=dev)
for lvl,m in arcpy.MSGS: print(" ", lvl, m)
print("  outputs:", {k:len(arcpy.FCS[d[k].value]["rows"]) for k in ("out_boundary","out_legs","out_routes","out_waypoints","out_photos","out_site")})
print("  files:", sorted(os.listdir(out)), "device:", sorted(os.listdir(dev))[:3])
print("\n== Area from CAD closed polyline, manual CRS MGA2020, crosshatch, manual heading, aircraft change")
d=run(T.AreaMission(), in_features="cad", out_gdb=gdb, aircraft="DJI Matrice 350 RTK + P1", size_mode="Flying height", height=130.0,
      out_crs=arcpy.SpatialReference(7850), crosshatch=True, heading_mode="Manual", heading=30.0)
print(" ", [m for l,m in arcpy.MSGS if "Planning coord" in m or "Built" in m or "Flights/" in m or "WARNING" in m or "heading" in m])
print("  M350 overrides populated:", d["focal"].value, d["mass"].value, "height msg:", d["height"].msg)
print("\n== Route with DEM terrain following")
from osgeo import gdal, osr; import numpy as np
dem=os.path.join(gdb,"dem.tif"); r=gdal.GetDriverByName("GTiff").Create(dem,500,200,1,gdal.GDT_Float32)
r.SetGeoTransform((115.725,0.0001,0,-32.300,0,-0.0001)); s=osr.SpatialReference(); s.ImportFromEPSG(4326); r.SetProjection(s.ExportToWkt())
r.GetRasterBand(1).WriteArray(np.fromfunction(lambda i,j: 10+j*0.2,(200,500))); r=None
d=run(T.RouteMission(), in_features="road", out_gdb=gdb, dem=dem, terrain=True, left_width=40.0, right_width=40.0, gsd=1.8, payload=0.3, aircraft="DJI Mavic 3 Enterprise")
wps=arcpy.FCS[d["out_waypoints"].value]["rows"]; alts=[r[4] for r in wps if r[3].startswith("line")]
print("  route flights:", len(arcpy.FCS[d["out_routes"].value]["rows"]), "terrain alts %.1f..%.1f" % (min(alts),max(alts)))
for l,m in arcpy.MSGS:
    if l=="W": print("  W", m)
print("\n== Flight time calculator")
run(T.FlightTimeCalculator(), aircraft="DJI Mavic 3 Enterprise", temp=0.0, elev=1500.0, payload=0.15, wind=5.0)
for l,m in arcpy.MSGS: print(" ", l, m)
print("\n== Upload tool: copy + live via mock vehicle")
plan=[f for f in os.listdir(out) if f.endswith(".plan")][0]
dev2=tempfile.mkdtemp()
run(T.UploadMission(), in_file=os.path.join(out,plan), method="Copy to controller / SD card folder", device_folder=dev2)
print("  copied:", os.listdir(dev2))
ps=T.UploadMission().getParameterInfo(); dd=T._by_name(ps); dd["in_file"].value=os.path.join(out,"area_mission.kmz") ; T.UploadMission().updateMessages(ps); print("  kmz live validation:", dd["in_file"].msg)
