import os, sys, json, zipfile, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, REPO + "/qgis")
from qgis.core import *
from qgis.PyQt.QtCore import QCoreApplication
from qgis.PyQt.QtWidgets import QApplication
app = QgsApplication([], True); app.initQgis()
from qgis.PyQt.QtCore import QSettings as _QS  # CLEAN_DMP_SETTINGS
_QS().remove('dmp')
from qgis.testing.mocked import get_iface
iface = get_iface()
canvas = iface.mapCanvas()
canvas.setDestinationCrs(QgsCoordinateReferenceSystem("EPSG:3857"))
QgsProject.instance().setCrs(QgsCoordinateReferenceSystem("EPSG:4326"))

import drone_mission_planner as pkg
plugin = pkg.classFactory(iface)
plugin.initGui()
plugin.toggle(True)
d = plugin.dock
from drone_mission_planner import layers as L
def pump(ms=1000):
    import time; t=time.time()
    while time.time()-t<ms/1000: app.processEvents()

# 1. "draw" an L-shaped polygon (as the capture tool would emit, in canvas CRS 3857)
from qgis.core import QgsPointXY as P
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), QgsCoordinateReferenceSystem("EPSG:3857"), QgsProject.instance())
ll = [(115.735,-32.300),(115.741,-32.300),(115.741,-32.2975),(115.738,-32.2975),(115.738,-32.295),(115.735,-32.295)]
pts = [xf.transform(P(*p)) for p in ll]
canvas.setExtent(QgsGeometry.fromPolygonXY([pts]).boundingBox()); canvas.refresh()
# emulate a CaptureTool session
from drone_mission_planner.maptools import CaptureTool
d.start_draw(); tool = canvas.mapTool(); assert isinstance(tool, CaptureTool), tool
tool.points = pts; tool._finish()
assert d.mission is not None, d.report.toPlainText()
print("CRS:", d.work_crs.authid()); print(d.report.toPlainText()); print()
out = {n: L.find(n).featureCount() for n in (L.BOUNDARY, L.LEGS, L.ROUTES, L.WAYPOINTS, L.PHOTOS, L.SITE)}
print("layers:", out)

# 2. pick take-off & a separate landing point
d._picked("takeoff", xf.transform(P(115.7345,-32.3005)))
d.same_ld.setChecked(False)
d._picked("landing", xf.transform(P(115.7415,-32.3005)))
pump(1200)
print("takeoff/landing:", d.mission.takeoff, d.mission.landing, "| label:", d.site_lbl.text().splitlines()[0])

# 3. live parameter changes -> replan
t0 = d.mission.total_time_s
d.gsd.setValue(1.5); pump(1200)
print("GSD 1.5 -> height %.1f, flights %d, time %.1f->%.1f min" % (d.height.value(), len(d.mission.flights), t0/60, d.mission.total_time_s/60))
d.wind.setValue(9); d.temp.setValue(2); d.payload_tbl.add_item("Sensor", 0.15, 0); d.elev.setValue(400); pump(1200)
print("harsh conditions:", d.mission.endurance, d.mission.warnings)
d.head_mode.setCurrentIndex(1); d.head.setValue(45); pump(1200); print("manual heading", d.mission.heading_deg)

# 4. edit boundary vertex (as the vertex tool would) -> replan
lyr = L.find(L.BOUNDARY); a0 = d.mission.area_m2
lyr.startEditing(); f = next(lyr.getFeatures()); g = f.geometry(); g.moveVertex(g.vertexAt(1).x()+200, g.vertexAt(1).y(), 1)
lyr.changeGeometry(f.id(), g); lyr.commitChanges(); pump(1200)
print("edited boundary area %.2f -> %.2f ha" % (a0/1e4, d.mission.area_m2/1e4))

# 5. user-chosen CRS (GDA2020 / MGA zone 50)
d.auto_utm.setChecked(False); d.crs_w.setCrs(QgsCoordinateReferenceSystem("EPSG:7850")); pump(1200)
print("manual CRS:", d.work_crs.authid(), "legs layer CRS:", L.find(L.LEGS).crs().authid())

# 6. CAD DXF import (closed polyline, no CRS) + GeoPackage
from osgeo import ogr, osr, gdal
dxf = REPO + "/tests/qgis_data/site.dxf"
drv = ogr.GetDriverByName("DXF"); 
if os.path.exists(dxf): os.remove(dxf)
ds = drv.CreateDataSource(dxf); lay = ds.CreateLayer("entities")
xf2 = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), QgsCoordinateReferenceSystem("EPSG:7850"), QgsProject.instance())
ring = [xf2.transform(P(*p)) for p in [(115.75,-32.31),(115.756,-32.31),(115.756,-32.305),(115.75,-32.305),(115.75,-32.31)]]
feat = ogr.Feature(lay.GetLayerDefn()); feat.SetGeometry(ogr.CreateGeometryFromWkt("LINESTRING(%s)" % ",".join("%f %f"%(p.x(),p.y()) for p in ring))); lay.CreateFeature(feat); ds=None
QgsProject.instance().setCrs(QgsCoordinateReferenceSystem("EPSG:7850"))
from qgis.PyQt.QtWidgets import QFileDialog, QMessageBox
QFileDialog.getOpenFileName = staticmethod(lambda *a, **k: (dxf, ""))
QMessageBox.information = staticmethod(lambda *a, **k: None)
QMessageBox.warning = staticmethod(lambda *a, **k: print("WARN DIALOG", a[2]))
d.import_file()
print("DXF import area %.2f ha (expected ~%.2f)" % (d.mission.area_m2/1e4, 0.006*94400*0.005*111000/1e4))

# 7. DEM terrain following
dem = REPO + "/tests/qgis_data/dem.tif"
r = gdal.GetDriverByName("GTiff").Create(dem, 200, 200, 1, gdal.GDT_Float32)
r.SetGeoTransform((115.745, 0.0001, 0, -32.300, 0, -0.0001)); s = osr.SpatialReference(); s.ImportFromEPSG(4326); r.SetProjection(s.ExportToWkt())
import numpy as np; arr = np.fromfunction(lambda i,j: 20 + j*0.8, (200,200)); r.GetRasterBand(1).WriteArray(arr); r=None
rl = QgsRasterLayer(dem, "dem"); rl.setCrs(QgsCoordinateReferenceSystem("EPSG:4326")); QgsProject.instance().addMapLayer(rl)
d.dem_cb.setLayer(rl); d._picked("takeoff", xf.transform(P(115.7495,-32.3105)))
d.terrain.setChecked(True); pump(1200)
alts=[w.alt for w in d.mission.waypoints if w.kind.startswith("line")]
smp=d._dem_sampler(d.work_crs); tk=d.mission.takeoff; print("DBG rl crs", rl.crs().authid(), rl.crs().isValid(), rl.extent().toString(), rl.isValid()); print("DBG takeoff",tk, d.work_crs.authid())
xfd=L.transform(d.work_crs, rl.crs()); print("DBG xf", xfd.transform(QgsPointXY(*tk)))
try: print("DBG sample", smp(*tk))
except Exception as e: print("DBG exc", repr(e))
print("terrain follow alts %.1f..%.1f (n=%d)" % (min(alts), max(alts), len(alts)), d.mission.warnings)
d._picked("takeoff", xf.transform(P(115.70,-32.3105))); pump(1200); print("takeoff off-DEM:", d.mission.warnings[:1] if d.mission else d.report.toPlainText()[:400])
d._picked("takeoff", xf.transform(P(115.7495,-32.3105)))

# 8. route mode
d.type_cb.setCurrentIndex(1)
line = [xf.transform(P(*p)) for p in [(115.73,-32.31),(115.75,-32.305),(115.77,-32.307)]]
d.terrain.setChecked(False)
d.set_boundary(QgsGeometry.fromPolylineXY(line), canvas.mapSettings().destinationCrs())
print(); print(d.report.toPlainText())

# 9. exports via the dock
outdir = tempfile.mkdtemp()
for i in range(d.fmt_cb.count()):
    d.fmt_cb.setCurrentIndex(i); key = d.fmt_cb.currentData()
    QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(outdir, "route_"+key+(".csv" if key=="csv" else "")), ""))
    d.export()
print("exported:", sorted(os.listdir(outdir)))

# 10. upload dialog (device copy path)
from drone_mission_planner.dock import UploadDialog
dlg = UploadDialog(d); dlg.r_dev.setChecked(True)
dev = tempfile.mkdtemp(); dlg.dest.setCurrentText(dev); dlg.dev_fmt.setCurrentIndex(dlg.dev_fmt.findData("dji_kmz")); dlg.run()
print("device copy:", os.listdir(dev), "|", dlg.log.toPlainText().strip()[:120])
dlg.r_mav.setChecked(True); dlg.conn.setCurrentText("udpin:127.0.0.1:14599")
import dronemapper_core.uploader as U
orig = U.upload_mavlink
U.upload_mavlink = lambda *a, **k: (_ for _ in ()).throw(U.UploadError("No heartbeat (test)"))
dlg.run(); print("mavlink error path:", dlg.log.toPlainText().splitlines()[-1])
plugin.unload(); print("unloaded OK")
app.exitQgis()
