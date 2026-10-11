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
pl = pkg.classFactory(iface); pl.initGui(); pl.toggle(True); d = pl.dock
from qgis.core import QgsPointXY as P
from qgis.PyQt.QtWidgets import QFileDialog, QMessageBox
QMessageBox.information = staticmethod(lambda *a, **k: None)
def pump(ms=1000):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
# synthetic "aerial photo": RGB GeoTIFF with fields and a road
from osgeo import gdal, osr; import numpy as np
img = REPO + "/tests/qgis_data/aerial.tif"
W, H = 900, 700
ds = gdal.GetDriverByName("GTiff").Create(img, W, H, 3, gdal.GDT_Byte)
ds.SetGeoTransform((115.725, 0.035 / W, 0, -32.280, 0, -0.025 / H)); s = osr.SpatialReference(); s.ImportFromEPSG(4326); ds.SetProjection(s.ExportToWkt())
yy, xx = np.mgrid[0:H, 0:W]
fields = ((xx // 120 + yy // 90) % 3)
base = np.stack([110 + fields * 25, 140 + fields * 18, 70 + fields * 10]).astype(np.uint8)
road = np.abs(yy - (350 + 60 * np.sin(xx / 140.0))) < 5
for b in range(3): base[b][road] = 205
noise = np.random.RandomState(0).randint(-12, 12, (3, H, W)); base = np.clip(base.astype(int) + noise, 0, 255).astype(np.uint8)
for b in range(3): ds.GetRasterBand(b + 1).WriteArray(base[b])
ds = None
rl = QgsRasterLayer(img, "Aerial imagery"); rl.setCrs(QgsCoordinateReferenceSystem("EPSG:4326")); QgsProject.instance().addMapLayer(rl)
canvas.setLayers([rl])
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ring = [(115.732, -32.300), (115.752, -32.300), (115.752, -32.288), (115.742, -32.284), (115.732, -32.288)]
canvas.setExtent(xf.transformBoundingBox(rl.extent()))
print("== mission name")
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
print("  placeholder (drawn):", d.name_le.placeholderText(), "| report:", d.report.toPlainText().splitlines()[0])
# use a layer -> its name in default
lyr = QgsVectorLayer("Polygon?crs=EPSG:4326", "Lot 12 boundary", "memory"); f = QgsFeature(); f.setGeometry(QgsGeometry.fromPolygonXY([[P(*q) for q in ring]])); lyr.dataProvider().addFeatures([f]); QgsProject.instance().addMapLayer(lyr, False)
d.use_layer(lyr); pump()
print("  placeholder (from layer):", d.name_le.placeholderText())
d.name_le.setText("Lot 12 Stockpile Survey"); pump(800)
print("  typed -> report:", d.report.toPlainText().splitlines()[0])
d._picked("takeoff", xf.transform(P(115.7315, -32.3005))); d._picked("station", xf.transform(P(115.7525, -32.2930))); pump(1200)
seen = {}
out = tempfile.mkdtemp()
def dlg(*a, **k):
    seen["default"] = os.path.basename(a[2]); return (os.path.join(out, seen["default"]), "")
QFileDialog.getSaveFileName = staticmethod(dlg)
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief")); d.export()
print("  default export file name:", seen["default"])
h = open(os.path.join(out, seen["default"])).read()
print("  brief title:", h[h.index("<h1>") + 4:h.index("</h1>")], "| background embedded:", "data:image/jpeg;base64" in h, "| size %d KB" % (len(h) // 1024))
import shutil; shutil.copy(os.path.join(out, seen["default"]), REPO + "/tests/out/brief_bg.html")
# online basemap (tile fetch faked: sandbox has no internet) + current map with no visible layers -> OSM fallback
import zlib, struct
def png(r, g, b):
    raw = b"".join(b"\x00" + bytes([r, g, b]) * 256 for _ in range(256))
    ch = lambda tag, dd: struct.pack(">I", len(dd)) + tag + dd + struct.pack(">I", zlib.crc32(tag + dd) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", 256, 256, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(raw)) + ch(b"IEND", b"")
urls = []
core = sys.modules["dronemapper_core"]
_tc = os.path.join(QgsApplication.qgisSettingsDirPath(), "cache", "dmp_tiles")
import shutil; shutil.rmtree(_tc, ignore_errors=True)       # count real fetches, not cached tiles
core.basemap.TileBasemap._http = lambda self, u: (urls.append(u), png(200, 220, 200))[1]
d.brief_bg.setCurrentIndex(d.brief_bg.findData("esri_imagery")); d.export(); h3 = open(os.path.join(out, seen["default"])).read()
print("  Esri imagery: tiles", len(urls), "World_Imagery" in urls[0], "| <image>", h3.count("<image"), "| attribution", "Imagery \u00a9 Esri" in h3)
vis = [l for l in iface.mapCanvas().layers()]
iface.mapCanvas().setLayers([l for l in vis if l.customProperty("dmp_role")]); urls.clear(); shutil.rmtree(_tc, ignore_errors=True)
d.brief_bg.setCurrentIndex(d.brief_bg.findData("canvas")); d.export(); h4 = open(os.path.join(out, seen["default"])).read()
print("  current map, nothing visible -> OSM tiles", len(urls), "tile.openstreetmap.org" in (urls or [""])[0], "| attribution", "OpenStreetMap contributors" in h4)
iface.mapCanvas().setLayers(vis)
d.brief_bg.setCurrentIndex(d.brief_bg.findData("none")); d.export(); h2 = open(os.path.join(out, seen["default"])).read()
print("  untick background -> no image:", "data:image" not in h2, "| %d KB" % (len(h2) // 1024))
d.brief_bg.setCurrentIndex(d.brief_bg.findData("canvas")); d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief_pdf")); d.export()
pdf = os.path.join(out, seen["default"]); pd = open(pdf, "rb").read()
print("  PDF report:", seen["default"], "| %d KB" % (len(pd) // 1024), "| valid:", pd[:5] == b"%PDF-" and pd.rstrip().endswith(b"%%EOF"), "| images:", pd.count(b"/Subtype /Image"))
assert seen["default"].endswith(".pdf") and pd.count(b"/Subtype /Image") >= 1
d.wind.setValue(15); d.wind_unit_cb.setCurrentIndex(d.wind_unit_cb.findData("m/s")); w_ms = d.wind.value(); d.wind_unit_cb.setCurrentIndex(d.wind_unit_cb.findData("kn"))
print("  wind unit switch kn -> m/s -> kn:", w_ms, "->", d.wind.value(), "| report:", [l for l in d.report.toPlainText().splitlines() if l.startswith("Wind")])
# flight date, time zone, early arrival -> daylight section and warnings
from qgis.PyQt.QtCore import QDate, QTime
d.fdate.setDate(QDate(2026, 12, 21)); d.tz.setValue(8); d.start_t.setTime(QTime(4, 0)); pump(1500)
rt = d.report.toPlainText()
print("  daylight in report:", "Flight date & light" in rt, "| warnings:", [w[:60] for w in d.mission.warnings if w.startswith(("DAYLIGHT", "LIGHT"))][:2])
print("  name hint with flight date:", d.name_le.placeholderText() or d.mission_name())
assert "Flight date & light" in rt and any(w.startswith("DAYLIGHT") for w in d.mission.warnings)
d.tz.setValue(d.tz.minimum()); pump(800); print("  computer zone ->", d.tz.text(), "|", [l for l in d.report.toPlainText().splitlines() if "Date" in l][:1])
d.start_t.setTime(QTime(8, 0)); pump(800)
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("dji_kmz")); d.export(); print("  kmz default:", seen["default"], "->", sorted(x for x in os.listdir(out) if x.endswith(".kmz"))[:2])
pl.unload()

print("\n== ArcGIS")
sys.path.insert(0, REPO + "/tests/fake_arcpy"); import arcpy
loader = importlib.machinery.SourceFileLoader("dmp_b", REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt")
spec = importlib.util.spec_from_loader("dmp_b", loader); T = importlib.util.module_from_spec(spec); loader.exec_module(T)
Wsr = arcpy.SpatialReference(4326)
arcpy.INPUTS["C:/data/site_boundaries.gdb/Lot_7"] = {"shapeType": "Polygon", "sr": Wsr, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring + [ring[0]]]), Wsr)]}
def run(**vals):
    arcpy.MSGS.clear(); tool = T.AreaMission(); ps = tool.getParameterInfo(); p = T._by_name(ps)
    for k, v in vals.items(): p[k].value = v; p[k].altered = True
    tool.updateParameters(ps); tool.updateMessages(ps); tool.execute(ps, None); return p
ex = tempfile.mkdtemp()
p = run(in_features="C:/data/site_boundaries.gdb/Lot_7", out_gdb=tempfile.mkdtemp(), export_folder=ex, formats=["Field brief & battery plan (.html)"])
print("  automatic:", [m for l, m in arcpy.MSGS if m.startswith("Mission  ")], os.listdir(ex))
ex2 = tempfile.mkdtemp()
p = run(in_features="C:/data/site_boundaries.gdb/Lot_7", mission_name="Lot 7: Quarry Face", out_gdb=tempfile.mkdtemp(), export_folder=ex2, formats=["Field brief & battery plan (.html)"])
h = open(os.path.join(ex2, os.listdir(ex2)[0])).read()
print("  named: out_name ->", p["out_name"].value, "| files", os.listdir(ex2), "| title:", h[h.index("<h1>") + 4:h.index("</h1>")], "| map:", "<svg" in h)
