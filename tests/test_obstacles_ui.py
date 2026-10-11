import os, sys, time, tempfile, math, importlib.util, importlib.machinery
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, sys.argv[1] if len(sys.argv) > 1 else REPO + "/qgis"); sys.path.insert(0, REPO + "/tests")
from qgis.core import *
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
from fake_iface import FakeIface
iface = FakeIface(); canvas = iface.mapCanvas(); canvas.resize(900, 600)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857"); canvas.setDestinationCrs(c3857)
import drone_mission_planner as pkg
from drone_mission_planner import layers as L
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
def pump(ms=1200):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ring = [(115.735, -32.300), (115.745, -32.300), (115.745, -32.293), (115.735, -32.293)]
canvas.setExtent(xf.transformBoundingBox(QgsRectangle(115.73, -32.305, 115.75, -32.288)))
# obstacle layers: powerline (no height field -> default) and buildings with HEIGHT field
pw = QgsVectorLayer("LineString?crs=EPSG:4326&field=name:string", "Powerlines", "memory")
f = QgsFeature(pw.fields()); f.setGeometry(QgsGeometry.fromPolylineXY([P(115.733, -32.2965), P(115.747, -32.2962)])); f.setAttributes(["33 kV"]); pw.dataProvider().addFeatures([f])
bl = QgsVectorLayer("Polygon?crs=EPSG:4326&field=HEIGHT:double", "Buildings", "memory")
fs = []
for i, (lo, la, h) in enumerate([(115.737, -32.2985, 12.0), (115.742, -32.2945, 22.0), (115.744, -32.2990, None)]):
    g = QgsGeometry.fromPolygonXY([[P(lo, la), P(lo + 0.0002, la), P(lo + 0.0002, la + 0.00015), P(lo, la + 0.00015)]])
    ft = QgsFeature(bl.fields()); ft.setGeometry(g); ft.setAttributes([h]); fs.append(ft)
bl.dataProvider().addFeatures(fs)
QgsProject.instance().addMapLayers([pw, bl])
d.height.setValue(25)
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
print("== min height above ground")
d.min_agl.setValue(35); pump()
print("  ", [l for l in d.report.toPlainText().splitlines() if l.startswith(("Flight height", "Min height"))], "|", [w for w in d.mission.warnings if "raised" in w][:1])
d.min_agl.setValue(0); pump()
print("== obstacles: buildings layer (HEIGHT field auto-picked)")
d.obs_cb.setLayer(bl); pump()
print("  height field:", d.obs_field.currentField(), "|", d.obs_lbl.text())
print("  ", [l for l in d.report.toPlainText().splitlines() if l.startswith("Min height")])
alts = sorted({round(w.alt) for w in d.mission.waypoints if w.kind.startswith("line")}); print("  survey altitudes:", alts)
print("== obstacles: powerline (default 30 m), feet units")
d.obs_cb.setLayer(pw); d.obs_default.setValue(30); pump()
print("  field:", repr(d.obs_field.currentField()), "|", d.obs_lbl.text())
d.unit_cb.setCurrentIndex(1); pump()
print("  in feet:", d.obs_default.text(), d.obs_margin.text(), d.obs_buffer.text(), "| report:", [l for l in d.report.toPlainText().splitlines() if l.startswith("Min height")])
d.unit_cb.setCurrentIndex(0); pump()
m = d.mission
from dronemapper_core import geometry as G, ObstacleSet
S = m.obstacles
worst = 1e9
for fl in m.flights:
    w = fl.waypoints
    for a, b in zip(w[:-1], w[1:]):
        if G.dist((a.x, a.y), (w[0].x, w[0].y)) < 1 or G.dist((b.x, b.y), (w[-1].x, w[-1].y)) < 1: continue
        L_ = G.dist((a.x, a.y), (b.x, b.y)); n = max(1, int(L_))
        for k in range(n + 1):
            q = (a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n); z = a.alt + (b.alt - a.alt) * k / n
            t = S.top(q)
            if t: worst = min(worst, z - t)
print("  min clearance above powerline (every 1 m): %.1f m (margin %.0f)" % (worst, d.obs_margin.value()))
tl = L.find(L.TERRAIN); print("  terrain warnings layer:", tl.featureCount() if tl else None)
d.obs_margin.setValue(2); pump(); print("  margin 2 m -> warnings (plan meets its own margin):", L.find(L.TERRAIN).featureCount(), [w[:60] for w in d.mission.warnings if w.startswith("OBSTACLES")])
from qgis.PyQt.QtWidgets import QFileDialog, QMessageBox
QMessageBox.information = staticmethod(lambda *a, **k: None)
d.obs_margin.setValue(10); pump()
out = tempfile.mkdtemp(); QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(out, "b.html"), ""))
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief")); d.export()
h = open(os.path.join(out, "b.html")).read(); print("  brief: obstacles drawn:", 'stroke-dasharray="1 4"' in h, "| legend:", "Obstacle (hover for height)" in h)
import shutil; shutil.copy(os.path.join(out, "b.html"), REPO + "/tests/out/brief_obstacles_qgis.html")
pl.unload()

print("\n== ArcGIS obstacles")
sys.path.insert(0, REPO + "/tests/fake_arcpy"); import arcpy
loader = importlib.machinery.SourceFileLoader("dmp_o", REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt")
spec = importlib.util.spec_from_loader("dmp_o", loader); T = importlib.util.module_from_spec(spec); loader.exec_module(T)
W = arcpy.SpatialReference(4326)
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring + [ring[0]]]), W)]}
arcpy.INPUTS["bld"] = {"shapeType": "Polygon", "sr": W, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(lo, la), arcpy.Point(lo + 0.0002, la), arcpy.Point(lo + 0.0002, la + 0.00015), arcpy.Point(lo, la + 0.00015), arcpy.Point(lo, la)]), W) for lo, la in [(115.737, -32.2985), (115.742, -32.2945), (115.744, -32.2990)]],
                       "attrs": [{"HGT_FT": 40}, {"HGT_FT": 72}, {"HGT_FT": None}]}
tool = T.AreaMission(); ps = tool.getParameterInfo(); p = T._by_name(ps)
for k, v in dict(in_features="blk", out_gdb=tempfile.mkdtemp(), height_units="Feet", size_mode="Flying height", height=80.0, min_agl=60.0, obstacles="bld", obstacle_field="HGT_FT", obstacle_default=30.0, obstacle_margin=30.0, obstacle_buffer=50.0).items():
    p[k].value = v; p[k].altered = True
tool.updateParameters(ps); tool.updateMessages(ps); arcpy.MSGS.clear(); tool.execute(ps, None)
print("  ", [m_ for l, m_ in arcpy.MSGS if m_.startswith(("Obstacles", "Min height", "Flight height", "WARNING: Obstacles"))])
print("  terrain fc rows:", len(arcpy.FCS[p["out_terrain"].value]["rows"]) if p["out_terrain"].value else "n/a")
