import os, sys, math, time, threading, http.server, socketserver, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
PLUG = sys.argv[1] if len(sys.argv) > 1 else REPO + "/qgis"
sys.path.insert(0, PLUG)
from qgis.core import *
from qgis.PyQt.QtCore import QSize
from qgis.PyQt.QtGui import QColor
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
sys.path.insert(0, REPO + '/tests'); from fake_iface import FakeIface
iface = FakeIface(); canvas = iface.mapCanvas(); canvas.resize(1000, 700)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857"); canvas.setDestinationCrs(c3857)
# ---- mock download servers (same files as test_dem.py: Copernicus S33E115 + terrarium z13)
ROOT = REPO + "/tests/demsrv"
class H(http.server.SimpleHTTPRequestHandler):
    def __init__(s, *a, **k): super().__init__(*a, directory=ROOT, **k)
    def log_message(s, *a): pass
    def send_head(s):
        path = s.translate_path(s.path)
        if not os.path.isfile(path): s.send_error(403); return None
        rng = s.headers.get("Range"); size = os.path.getsize(path); f = open(path, "rb")
        if rng:
            a, b = rng.split("=")[1].split("-"); a = int(a); b = min(int(b) if b else size - 1, size - 1)
            f.seek(a); s.send_response(206); s.send_header("Content-Range", "bytes %d-%d/%d" % (a, b, size))
            s.send_header("Content-Length", str(b - a + 1)); s.send_header("Accept-Ranges", "bytes"); s.end_headers(); s._len = b - a + 1; return f
        s.send_response(200); s.send_header("Content-Length", str(size)); s.end_headers(); return f
    def copyfile(s, src, dst):
        n = getattr(s, "_len", None); dst.write(src.read(n) if n else src.read())
srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), H); threading.Thread(target=srv.serve_forever, daemon=True).start()
base = "http://127.0.0.1:%d" % srv.server_address[1]
import drone_mission_planner as pkg
import dronemapper_core as core
core.dem.COPERNICUS_URL = base + "/{name}/{name}.tif"
core.dem.TERRARIUM_URL = base + "/terrarium/{z}/{x}/{y}.png"
from drone_mission_planner import layers as L
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
from qgis.PyQt.QtWidgets import QMessageBox, QFileDialog
QMessageBox.information = staticmethod(lambda *a, **k: None)
QMessageBox.warning = staticmethod(lambda *a, **k: print("  WARN DIALOG:", a[2][:150]))

def PROBE(tag):
    from osgeo import osr as _o
    w = 'GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]]'
    print("  PROBE %-28s fromWkt valid=%s" % (tag, QgsCoordinateReferenceSystem.fromWkt(w).isValid()))
def pump(ms=1200):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
def truth(lon, lat): return 50 + 3000*(lon-115.70) + 120*math.exp(-(((lon-115.75)/0.01)**2 + ((lat+32.40)/0.01)**2))

print("== 1. area over a hill, no DEM yet -> plan, then Download DEM (Copernicus)")
ring = [(115.738, -32.410), (115.762, -32.410), (115.762, -32.392), (115.738, -32.392)]
canvas.setExtent(xf.transformBoundingBox(QgsRectangle(115.73, -32.42, 115.77, -32.385)))
d.height.setValue(60)
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
d._picked("takeoff", xf.transform(P(115.737, -32.411))); pump()
print("  before DEM: terrain report?", "obstacle check" in d.report.toPlainText())
d.dem_src.setCurrentIndex(d.dem_src.findData("copernicus")); d.download_dem(); pump(1500)
lyr = d.dem_cb.currentLayer()
print("  DEM layer:", lyr.name() if lyr else None, "| CRS", lyr.crs().authid() if lyr else "", "| take-off elev set to %.1f m (truth %.1f)" % (d.elev.value(), truth(115.737, -32.411)))
txt = d.report.toPlainText(); print("\n".join("  " + l for l in txt[txt.index("Terrain & obstacle check"):].splitlines()[:9]))
tl = L.find(L.TERRAIN); print("  terrain warnings layer:", tl.featureCount(), "points;", sorted({f["kind"] for f in tl.getFeatures()}))
print("  profile chart visible (area):", not d.profile.isHidden())

print("\n== 2. terrain following on")
d.terrain.setChecked(True); pump(1500)
txt = d.report.toPlainText(); print("\n".join("  " + l for l in txt[txt.index("Terrain & obstacle check"):].splitlines()[:7]))
print("  terrain warnings:", L.find(L.TERRAIN).featureCount())

print("\n== 3. route over the hill -> elevation profile")
d.type_cb.setCurrentIndex(1)
line = [xf.transform(P(115.735 + i * 0.0015, -32.40 + 0.002 * math.sin(i / 3))) for i in range(21)]
d.left_w.setValue(30); d.right_w.setValue(30)
d.set_boundary(QgsGeometry.fromPolylineXY(line), c3857); pump(1500)
rows = d.mission.profile
print("  profile rows:", len(rows), "| KP %.2f-%.2f km | ground %.0f-%.0f m | chart shown:" % (rows[0]["kp"]/1000, rows[-1]["kp"]/1000, min(r["ground"] for r in rows), max(r["ground"] for r in rows)), not d.profile.isHidden())
d.profile.resize(380, 180); d.profile.grab().save(REPO + "/tests/out/profile_widget.png")
d.terrain.setChecked(False); pump(1500)
print("  without terrain follow:", [l for l in d.report.toPlainText().splitlines() if "LOW" in l or "Lowest" in l])

print("\n== 4. AWS Terrain Tiles source")
d.dem_src.setCurrentIndex(d.dem_src.findData("terrarium")); d.download_dem(); pump(1500)
lyr2 = d.dem_cb.currentLayer(); print("  DEM layer:", lyr2.name(), lyr2.crs().authid(), "| elev %.1f m" % d.elev.value())

print("\n== 5. field brief with terrain + profile")
d.terrain.setChecked(True); pump(1500)
out = tempfile.mkdtemp(); d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief"))
QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(out, "brief.html"), ""))
d.export(); html = open(os.path.join(out, "brief.html")).read()
print("  brief: terrain section", "Terrain (DEM)" in html, "| profile svg", "<svg" in html)
import shutil; shutil.copy(os.path.join(out, "brief.html"), REPO + "/tests/out/route_brief_qgis.html")

print("\n== 6. download with no network -> friendly message")
core.dem.TERRARIUM_URL = "http://127.0.0.1:9/{z}/{x}/{y}.png"; d.download_dem()
pl.unload(); srv.shutdown(); print("\nunloaded OK")
