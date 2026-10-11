import os, sys, math, time, tempfile, json, importlib.util, importlib.machinery
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, REPO + "/qgis"); sys.path.insert(0, REPO + "/tests")
from qgis.core import *
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
from fake_iface import FakeIface
from qgis.PyQt.QtCore import QSettings
QSettings().setValue("dmp/height_unit", "m")
iface = FakeIface(); canvas = iface.mapCanvas(); canvas.resize(900, 600)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857"); canvas.setDestinationCrs(c3857)
import drone_mission_planner as pkg
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
def pump(ms=1000):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ring = [(115.735, -32.30), (115.752, -32.30), (115.752, -32.285), (115.735, -32.285)]
canvas.setExtent(xf.transformBoundingBox(QgsRectangle(115.73, -32.305, 115.76, -32.28)))
print("== QGIS: metres -> feet")
d.height.setValue(100); d.rth_h.setValue(110); d.batt_h.setValue(90); d.elev.setValue(20)
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
gsd_m = d.gsd.value()
d.unit_cb.setCurrentIndex(1); pump()
print("  height %s | RTH %s | batt %s | elev %s | GSD unchanged: %s" % (d.height.text(), d.rth_h.text(), d.batt_h.text(), d.elev.text(), abs(d.gsd.value() - gsd_m) < 0.005))
print("  report:", [l for l in d.report.toPlainText().splitlines() if "ht" in l or "Flight height" in l])
d.height.setValue(400); pump(1200)
print("  set 400 ft -> mission height %.2f m, GSD %.2f" % (d.mission.height_m, d.gsd.value()))
d.rth_h.setValue(0); pump(1200); print("  RTH blank:", d.rth_h.text(), "->", round(d.mission.rth_height_m, 1), "m")
print("  remembered unit:", QSettings().value("dmp/height_unit"))
d.unit_cb.setCurrentIndex(0); pump(); print("  back to metres: height", d.height.text())
pl.unload()

print("\n== ArcGIS: feet input")
sys.path.insert(0, REPO + "/tests/fake_arcpy"); import arcpy
loader = importlib.machinery.SourceFileLoader("dmp_u", REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt")
spec = importlib.util.spec_from_loader("dmp_u", loader); T = importlib.util.module_from_spec(spec); loader.exec_module(T)
W = arcpy.SpatialReference(4326)
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring + [ring[0]]]), W)]}
tool = T.AreaMission(); ps = tool.getParameterInfo(); p = T._by_name(ps)
for k, v in dict(in_features="blk", out_gdb=tempfile.mkdtemp(), height_units="Feet", size_mode="Flying height", height=300.0, rth_height=380.0, batt_height=350.0, export_folder=tempfile.mkdtemp(), formats=["QGroundControl plan (.plan)"]).items():
    p[k].value = v; p[k].altered = True
tool.updateParameters(ps); tool.updateMessages(ps)
print("  validation msgs:", [(q.name, q.msg) for q in ps if q.msg])
arcpy.MSGS.clear(); tool.execute(ps, None)
print("  ", [m for l, m in arcpy.MSGS if "ht" in m or "Flight height" in m])
fold = p["export_folder"].value; f = sorted(x for x in os.listdir(fold) if x.endswith(".plan"))[-1]
it = json.load(open(os.path.join(fold, f)))["mission"]["items"]
print("  last flight file: approach %.2f m (380 ft = %.2f m) then LAND" % (it[-2]["params"][6], 380 * 0.3048), it[-1]["command"] == 21)
# unit switch converts entered values
tool2 = T.AreaMission(); ps2 = tool2.getParameterInfo(); p2 = T._by_name(ps2)
p2["size_mode"].value = "Flying height"; p2["height"].value = 120.0; tool2.updateParameters(ps2)
p2["height_units"].value = "Feet"; p2["height_units"].altered = True; tool2.updateParameters(ps2)
print("  switch m->ft converts 120 m ->", p2["height"].value, "ft")
