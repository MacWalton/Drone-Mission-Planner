import os, sys, math, random, tempfile, time
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
PLUG = sys.argv[1] if len(sys.argv) > 1 else REPO + "/qgis"
sys.path.insert(0, PLUG)
from qgis.core import *
from qgis.PyQt.QtCore import QPoint, QEvent, Qt, QSize
from qgis.PyQt.QtGui import QColor
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
from qgis.testing.mocked import get_iface
from qgis.gui import QgsMapMouseEvent
iface = get_iface(); canvas = iface.mapCanvas(); canvas.resize(1000, 700)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857"); canvas.setDestinationCrs(c3857)
import drone_mission_planner as pkg
from drone_mission_planner import layers as L
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
def pump(ms=1200):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()

# ---- a "roads" layer: a ~14 km pipeline easement digitised as 7 shuffled, reversed pieces + a distractor road
roads = QgsVectorLayer("LineString?crs=EPSG:4326&field=name:string", "Pipeline easement", "memory")
pts = [(115.70 + i * 0.0015, -32.40 + 0.01 * math.sin(i / 9.0)) for i in range(91)]
pieces = [pts[i:i + 14] for i in range(0, 90, 13)]
random.seed(7); random.shuffle(pieces)
feats = []
for k, pc in enumerate(pieces):
    if k % 2: pc = list(reversed(pc))
    f = QgsFeature(roads.fields()); f.setGeometry(QgsGeometry.fromPolylineXY([P(*p) for p in pc])); f.setAttributes(["pipe %d" % k]); feats.append(f)
f = QgsFeature(roads.fields()); f.setGeometry(QgsGeometry.fromPolylineXY([P(115.72, -32.43), P(115.74, -32.43)])); f.setAttributes(["other road"]); feats.append(f)
roads.dataProvider().addFeatures(feats); QgsProject.instance().addMapLayer(roads)
canvas.setLayers([roads])
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ext = xf.transformBoundingBox(roads.extent()); ext.grow(500); canvas.setExtent(ext); canvas.refresh(); pump(300)

print("== 1. Route mode: pick the 7 pipeline pieces by clicking on the map")
d.type_cb.setCurrentIndex(1)
d.left_w.setValue(40); d.right_w.setValue(25); d.simplify.setValue(3)
d.drone_cb.setCurrentText("DJI Mavic 3 Enterprise"); d.gsd.setValue(2.0)
d.start_pick_lines(); tool = canvas.mapTool()
m2p = canvas.getCoordinateTransform()
def click(lonlat, button=Qt.MouseButton.LeftButton):
    px = m2p.transform(xf.transform(P(*lonlat)))
    ev = QgsMapMouseEvent(canvas, QEvent.Type.MouseButtonRelease, QPoint(int(px.x()), int(px.y())), button, button, Qt.KeyboardModifier.NoModifier)
    tool.canvasReleaseEvent(ev)
for pc in pieces:
    mid = pc[len(pc) // 2]; click(mid)
click(pieces[0][len(pieces[0]) // 2]); click(pieces[0][len(pieces[0]) // 2])   # toggle off & on again
print("picked:", len(tool.picked), "pieces (distractor road not clicked)")
click((115.73, -32.43), Qt.MouseButton.RightButton)   # right-click anywhere = finish
pump()
print("feature:", d.feat_lbl.text())
print(d.report.toPlainText().split("\n")[3])

print("\n== 2. KP window + 800 m VLOS + auto-placed stations")
d.kp_from.setValue(1.5); d.kp_to.setValue(11.0); d.kp_int.setValue(500)
d.range_cb.setChecked(True); d.range_m.setValue(800); d.st_side.setValue(-60)
d.sets.setValue(3); d.charge_min.setValue(70); d.start_t.setTime(d.start_t.time().fromString("07:30", "HH:mm"))
pump(); d.auto_stations(); pump()
print(d.report.toPlainText())
routes = L.find(L.ROUTES); r0 = next(routes.getFeatures())
print("routes attrs:", {k: r0[k] for k in ("flight","launch","land","battery_set","start","end","kp_from","kp_to","ends_by")})
site = L.find(L.SITE); print("site names:", sorted([(f["role"], f["name"]) for f in site.getFeatures()], key=lambda x: (len(x[1]), x[1]))[:5], "...")
ch = L.find(L.CHAINAGE); kps = [f["kp_km"] for f in ch.getFeatures()]; print("chainage markers:", len(kps), kps[:3], "...", kps[-1])
far = max(max(((w.x - f.station_start[0])**2 + (w.y - f.station_start[1])**2) ** 0.5 for w in f.waypoints) for f in d.mission.flights)
print("max distance from pilot: %.0f m" % far); assert far <= 801

print("\n== 3. drag a station (vertex edit) -> live re-plan")
before = d.mission.flights[1].station_start
site.startEditing(); f = [f for f in site.getFeatures() if f["role"] == "station"][0]
g = f.geometry(); p = g.asPoint(); site.changeGeometry(f.id(), QgsGeometry.fromPointXY(P(p.x() + 150, p.y() + 150))); site.commitChanges(); pump(1500)
print("station moved; flight 2 launch", [round(v) for v in before], "->", [round(v) for v in d.mission.flights[1].station_start])

print("\n== 4. leapfrog + whole-line splitting")
t_drive = d.mission.schedule.relocating_min
d.leapfrog.setChecked(True); pump()
print("leapfrog: drive %.0f -> %.0f min; flights landing elsewhere: %d" % (t_drive, d.mission.schedule.relocating_min, sum(f.station_end != f.station_start for f in d.mission.flights)))
d.leapfrog.setChecked(False); d.midline.setChecked(False); pump()
print("whole lines only:", len(d.mission.flights), "flights")
d.midline.setChecked(True); pump()

print("\n== 5. Area: 1.6 x 1.4 km block, auto stations, field brief export")
d.type_cb.setCurrentIndex(0)
ring = [(115.80, -32.40), (115.817, -32.40), (115.817, -32.3875), (115.80, -32.3875)]
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
d.range_m.setValue(1000); d.auto_stations(); pump()
print("\n".join(l for l in d.report.toPlainText().split("\n") if l.startswith(("Flights/", "  Flight", "  On site", "WARNING"))))
out = tempfile.mkdtemp()
from qgis.PyQt.QtWidgets import QFileDialog, QMessageBox
QMessageBox.information = staticmethod(lambda *a, **k: None)
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief"))
QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(out, "block_brief"), ""))
d.export()
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("dji_kmz"))
QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(out, "block.kmz"), ""))
d.export()
print("exports:", sorted(os.listdir(out)))
import shutil; os.makedirs(REPO + "/tests/out", exist_ok=True); shutil.copy(os.path.join(out, "block_brief.html"), REPO + "/tests/out/block_brief.html")

# ---- render the route mission for a visual check
d.type_cb.setCurrentIndex(1); pump()
order = [L.SITE, L.CHAINAGE, L.WAYPOINTS, L.ROUTES, L.LEGS, L.CENTRELINE]
cl = L.find(L.CENTRELINE); d.set_boundary(next(cl.getFeatures()).geometry(), cl.crs()); pump(); print("ROUTE REPLAN (area stations):", d.report.toPlainText()[:300].replace(chr(10)," | "), "| mission:", d.mission); d.auto_stations(); pump(); d.rev_kp.setChecked(True); pump(); d.auto_stations(); pump(); kp0 = min(L.find(L.CHAINAGE).getFeatures(), key=lambda f: f["kp_km"]); print("reversed: KP %.1f marker now at x=%.0f (was east end)" % (kp0["kp_km"], kp0.geometry().asPoint().x()))
print("after auto stations:", d.report.toPlainText().split(chr(10))[13]); print({n: L.find(n).featureCount() for n in order if L.find(n)})
order0 = None
order = [L.SITE, L.CHAINAGE, L.WAYPOINTS, L.ROUTES, L.LEGS, L.CENTRELINE]
lyrs = [L.find(n) for n in order if L.find(n)] + [roads]
ms = QgsMapSettings(); ms.setLayers(lyrs); ms.setDestinationCrs(c3857)
e = xf.transformBoundingBox(roads.extent()); e.grow(400); ms.setExtent(e); ms.setOutputSize(QSize(1600, 520)); ms.setBackgroundColor(QColor(246, 244, 239))
job = QgsMapRendererParallelJob(ms); job.start(); job.waitForFinished(); job.renderedImage().save(REPO + "/tests/out/route_map.png")
pl.unload(); print("\nunloaded OK")
