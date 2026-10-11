# SPDX-License-Identifier: GPL-2.0-only
"""Screenshots of the QGIS plugin, driven headless with sample data.

    QT_QPA_PLATFORM=offscreen <QGIS python> tools/make_app_screenshots.py

Run tools/build.py first. Writes docs/screenshots/qgis_*.png. The QGIS main
window isn't available headless, so each image shows the planner panel and the
mission layers rendered by QGIS, composed side by side.
"""
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(ROOT, "qgis"))
sys.path.insert(0, os.path.join(ROOT, "tests"))
from qgis.core import *                                            # noqa: E402,F401,F403
from qgis.core import QgsPointXY as P                               # noqa: E402
from qgis.PyQt.QtCore import QSize, QPoint, QRect, Qt, QSettings     # noqa: E402
from qgis.PyQt.QtGui import QColor, QImage, QPainter, QFont, QPen    # noqa: E402

app = QgsApplication([], True)
app.initQgis()
QSettings().remove("dmp")
from fake_iface import FakeIface                                   # noqa: E402
iface = FakeIface()
canvas = iface.mapCanvas()
canvas.resize(1000, 760)
c3857 = QgsCoordinateReferenceSystem("EPSG:3857")
canvas.setDestinationCrs(c3857)
import drone_mission_planner as pkg                                # noqa: E402
from drone_mission_planner import layers as L                      # noqa: E402
from drone_mission_planner.dock import UploadDialog                # noqa: E402

OUT = os.path.join(ROOT, "docs", "screenshots")
os.makedirs(OUT, exist_ok=True)
pl = pkg.classFactory(iface)
pl.initGui()
pl.toggle(True)
d = pl.dock
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
PAPER = QColor(244, 242, 236)


def pump(ms=1200):
    t = time.time()
    while time.time() - t < ms / 1000:
        app.processEvents()


def render(extra, extent, size):
    order = [L.TERRAIN, L.SITE, L.CHAINAGE, L.WAYPOINTS, L.ROUTES, L.LEGS, L.PHOTOS, L.BOUNDARY, L.CENTRELINE]
    lyrs = [L.find(n) for n in order if L.find(n) and n != L.PHOTOS] + extra
    ms = QgsMapSettings()
    ms.setLayers(lyrs)
    ms.setDestinationCrs(c3857)
    ms.setExtent(extent)
    ms.setOutputSize(size)
    ms.setOutputDpi(110)
    ms.setBackgroundColor(PAPER)
    job = QgsMapRendererParallelJob(ms)
    job.start()
    job.waitForFinished()
    return job.renderedImage()


def panel_image():
    root = d.widget().widget()
    root.setFixedWidth(520)
    root.resize(520, root.sizeHint().height())
    pump(300)
    return root.grab().toImage(), root


def compose(map_img, panel_crop, title, path):
    W = map_img.width() + panel_crop.width() + 1
    H = max(map_img.height(), panel_crop.height()) + 34
    img = QImage(W, H, QImage.Format.Format_RGB32)
    img.fill(QColor(255, 255, 255))
    p = QPainter(img)
    p.fillRect(0, 0, W, 34, QColor(46, 52, 64))
    f = QFont()
    f.setPointSize(11)
    f.setBold(True)
    p.setFont(f)
    p.setPen(QColor(255, 255, 255))
    p.drawText(QRect(12, 0, W, 34), int(Qt.AlignmentFlag.AlignVCenter), title)
    p.drawImage(0, 34, map_img)
    p.setPen(QPen(QColor(200, 200, 200)))
    p.drawLine(map_img.width(), 34, map_img.width(), H)
    p.drawImage(map_img.width() + 1, 34, panel_crop)
    p.end()
    img.save(path)
    print("wrote", os.path.relpath(path, ROOT))


# ------------------------------------------------------------- area mission
pw = QgsVectorLayer("LineString?crs=EPSG:4326&field=name:string&field=HEIGHT:double", "Powerlines", "memory")
f = QgsFeature(pw.fields())
f.setGeometry(QgsGeometry.fromPolylineXY([P(115.731, -32.2963), P(115.751, -32.2958)]))
f.setAttributes(["33 kV", 28.0])
pw.dataProvider().addFeatures([f])
pw.renderer().setSymbol(QgsLineSymbol.createSimple({"line_color": "60,60,60", "line_width": "0.7",
                                                     "line_style": "dot"}))
bl = QgsVectorLayer("Polygon?crs=EPSG:4326&field=HEIGHT:double", "Buildings & trees", "memory")
fs = []
for lo, la, w, h, ht in [(115.737, -32.2988, 0.0006, 0.0004, 12.0), (115.7465, -32.2940, 0.0012, 0.0008, 22.0),
                         (115.7435, -32.2992, 0.0004, 0.0003, 9.0)]:
    ft = QgsFeature(bl.fields())
    ft.setGeometry(QgsGeometry.fromPolygonXY([[P(lo, la), P(lo + w, la), P(lo + w, la + h), P(lo, la + h)]]))
    ft.setAttributes([ht])
    fs.append(ft)
bl.dataProvider().addFeatures(fs)
bl.renderer().setSymbol(QgsFillSymbol.createSimple({"color": "150,140,120,110", "outline_color": "90,80,70",
                                                     "style": "b_diagonal"}))
QgsProject.instance().addMapLayers([pw, bl])
canvas.setLayers([pw, bl])

ring = [(115.733, -32.3005), (115.749, -32.3005), (115.749, -32.2950), (115.744, -32.2925), (115.733, -32.2925)]
d.name_le.setText("Lot 12 Stockpile & Paddock Survey")
d.drone_cb.setCurrentText("DJI Mavic 3 Enterprise")
d.gsd.setValue(2.0)
d.wind.setValue(6)
d.wdir.setValue(220)
d.temp.setValue(31)
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857)
pump()
d._picked("takeoff", xf.transform(P(115.7335, -32.3012)))
d._picked("station", xf.transform(P(115.7445, -32.2972)))
d.obs_cb.setLayer(pw)
pump()
d.min_agl.setValue(40)
pump(1500)
ext = xf.transformBoundingBox(QgsRectangle(115.7310, -32.3020, 115.7510, -32.2915))
m_img = render([pw, bl], ext, QSize(1000, 760))
p_img, root = panel_image()
compose(m_img, p_img.copy(0, 0, p_img.width(), 760),
        "Drone Mission Planner - area mission with two launch stations (QGIS)",
        os.path.join(OUT, "qgis_area_mission.png"))
p_img.save(os.path.join(OUT, "qgis_panel_full.png"))
d.report.setFixedHeight(690)                      # show the whole text report
p2, root = panel_image()
y = d.report.mapTo(root, QPoint(0, 0)).y()
compose(m_img, p2.copy(0, y - 52, p2.width(), 760),
        "Mission report in the panel - heights, GSD, flights, batteries, field schedule, checks",
        os.path.join(OUT, "qgis_area_report.png"))
d.report.setMinimumHeight(0); d.report.setMaximumHeight(16777215)
print("wrote docs/screenshots/qgis_panel_full.png")

dlg = UploadDialog(d)
w = max(dlg.sizeHint().width(), 760)
dlg.layout().activate()
dlg.resize(w, dlg.layout().totalHeightForWidth(w))
pump(300)
dlg.grab().save(os.path.join(OUT, "qgis_upload_dialog.png"))
print("wrote docs/screenshots/qgis_upload_dialog.png")

# ------------------------------------------------------------- route mission
pipe = QgsVectorLayer("LineString?crs=EPSG:4326&field=name:string", "Pipeline easement", "memory")
pts = [(115.70 + i * 0.0015, -32.40 + 0.01 * math.sin(i / 9.0)) for i in range(71)]
f = QgsFeature(pipe.fields())
f.setGeometry(QgsGeometry.fromPolylineXY([P(*q) for q in pts]))
pipe.dataProvider().addFeatures([f])
pipe.renderer().setSymbol(QgsLineSymbol.createSimple({"line_color": "120,120,120", "line_width": "0.3"}))
QgsProject.instance().addMapLayer(pipe)
d.obs_cb.setLayer(None)
d.min_agl.setValue(0)
d.type_cb.setCurrentIndex(1)
d.name_le.setText("Gas pipeline KP 0-7 corridor survey")
d.left_w.setValue(40)
d.right_w.setValue(30)
d.kp_int.setValue(1000)
d.range_cb.setChecked(True)
d.range_m.setValue(900)
d.set_boundary(next(pipe.getFeatures()).geometry(), pipe.crs())
pump()
d.auto_stations()
pump(1500)
e = xf.transformBoundingBox(pipe.extent())
e.grow(450)
r_img = render([pipe], e, QSize(1000, 760))
p_img, root = panel_image()
compose(r_img, p_img.copy(0, 0, p_img.width(), 760),
        "Route / corridor mapping - auto-placed launch stations and chainage (QGIS)",
        os.path.join(OUT, "qgis_route_mission.png"))
print(d.report.toPlainText().splitlines()[:3])
pl.unload()
