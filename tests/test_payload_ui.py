import os, sys, time, tempfile, importlib.util, importlib.machinery
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else REPO + "/qgis"); sys.path.insert(0, REPO + "/tests")
from qgis.core import *
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
from qgis.PyQt.QtCore import QSettings
QSettings().remove("dmp/payload_items"); QSettings().setValue("dmp/height_unit", "m")
from fake_iface import FakeIface
iface = FakeIface(); canvas = iface.mapCanvas(); canvas.resize(900, 600)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857"); canvas.setDestinationCrs(c3857)
import drone_mission_planner as pkg
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
def pump(ms=1000):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ring = [(115.735, -32.30), (115.745, -32.30), (115.745, -32.292), (115.735, -32.292)]
canvas.setExtent(xf.transformBoundingBox(QgsRectangle(115.73, -32.305, 115.75, -32.285)))
print("== QGIS payload table")
d.drone_cb.setCurrentText("DJI Matrice 350 RTK + P1")
print("  battery capacity loaded from profile:", d.batt_wh.value(), "Wh")
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
e0 = d.mission.endurance["usable_minutes_still_air"]; n0 = len(d.mission.flights)
t = d.payload_tbl
t.add_item("LiDAR", 0.905, 28); t.add_item("Spotlight", 0.4, 30); t.add_item("Methane sensor", 0.5, 6); pump(1200)
e1 = d.mission.endurance
print("  total label:", t.total_lbl.text().replace("<b>", "").replace("</b>", ""))
print("  endurance %.1f -> %.1f min (%.1f without accessory power); flights %d -> %d" % (e0, e1["usable_minutes_still_air"], e1["minutes_without_accessories"], n0, len(d.mission.flights)))
print("  report:", [l.strip() for l in d.report.toPlainText().splitlines() if l.startswith(("Payload", "  -"))])
t.unit_cb.setCurrentIndex(1); pump(1200)
print("  in lb:", t.rows(), "| planner kg unchanged:", round(d.mission.conditions.total_payload_kg, 3))
t.table.item(1, 1).setText("2.2"); pump(1200)
print("  edit spotlight to 2.2 lb -> %.3f kg total" % d.mission.conditions.total_payload_kg)
t.add_item("Heavy winch", 4.0, 0); pump(1200)
print("  over limit label:", "Over the aircraft" in t.total_lbl.text(), "| warning:", [w for w in d.mission.warnings if "Payload" in w][:1])
t.table.selectRow(3); t.remove_selected(); pump(1200)
pl.unload()
saved = QSettings().value("dmp/payload_items"); print("  saved for next session:", saved[:90], "...")
pl2 = pkg.classFactory(iface); pl2.initGui(); pl2.toggle(True)
print("  restored on reopen:", pl2.dock.payload_tbl.rows(), pl2.dock.payload_tbl.unit); pl2.unload()

print("\n== ArcGIS payload value table")
sys.path.insert(0, REPO + "/tests/fake_arcpy"); import arcpy
loader = importlib.machinery.SourceFileLoader("dmp_p", REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt")
spec = importlib.util.spec_from_loader("dmp_p", loader); T = importlib.util.module_from_spec(spec); loader.exec_module(T)
W = arcpy.SpatialReference(4326)
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring + [ring[0]]]), W)]}
def run(tool, **vals):
    arcpy.MSGS.clear(); ps = tool.getParameterInfo(); p = T._by_name(ps)
    for k, v in vals.items(): p[k].value = v; p[k].altered = True
    tool.updateParameters(ps); tool.updateMessages(ps)
    msgs = [(q.name, q.msg) for q in ps if q.msg]
    tool.execute(ps, None); return p, msgs
rows = [["LiDAR", 2.0, 28], ["Spotlight", 0.9, 30], ["Methane sensor", 1.1, 6]]
p, msgs = run(T.FlightTimeCalculator(), aircraft="DJI Matrice 350 RTK + P1", payload_items=rows, payload_unit="lb")
print("  columns:", [c[1] for c in p["payload_items"].columns])
print("\n".join("  " + m for l, m in arcpy.MSGS[:9]))
p, msgs = run(T.AreaMission(), in_features="blk", out_gdb=tempfile.mkdtemp(), aircraft="DJI Mavic 3 Enterprise", payload_items=[["Light", 0.3, 10]])
print("  M3E with 0.3 kg light -> dialog:", msgs, "| report:", [m.strip() for l, m in arcpy.MSGS if m.startswith(("Payload", "  -", "WARNING: Payload"))])
