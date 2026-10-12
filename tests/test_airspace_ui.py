import os, sys, time, tempfile, math, importlib.util, importlib.machinery, json
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
core = sys.modules["dronemapper_core"]
from qgis.core import QgsPointXY as P
def pump(ms=1200):
    t = time.time()
    while time.time() - t < ms / 1000: app.processEvents()
xf = QgsCoordinateTransform(QgsCoordinateReferenceSystem("EPSG:4326"), c3857, QgsProject.instance())
ring = [(115.735, -32.300), (115.745, -32.300), (115.745, -32.293), (115.735, -32.293)]
out = tempfile.mkdtemp()
def dm(v, pos, neg):
    h = pos if v >= 0 else neg; v = abs(v); dd = int(v); mm = (v - dd) * 60
    return "%d:%06.3f %s" % (dd, mm, h)
Pt = lambda lo, la: "%s %s" % (dm(la, "N", "S"), dm(lo, "E", "W"))
oa = os.path.join(out, "test_airspace.txt")
open(oa, "w").write("""AC D
AN TEST CTR
AL SFC
AH 1500ft
DP %s
DP %s
DP %s
DP %s
AC R
AN R999 TEST RANGE
AL SFC
AH 3000ft
V X=%s
DC 0.5
""" % (Pt(115.70, -32.32), Pt(115.740, -32.32), Pt(115.740, -32.27), Pt(115.70, -32.27), Pt(115.760, -32.296)))
print("== QGIS: file source")
d.set_boundary(QgsGeometry.fromPolygonXY([[xf.transform(P(*q)) for q in ring]]), c3857); pump()
d.asp_src.setCurrentIndex(d.asp_src.findData("file")); d.asp_file.setText(oa); d.load_airspace(); pump(1500)
print("  ", d.asp_lbl.text())
rep = d.mission.airspace
print("  ", [(h.airspace.name, h.status) for h in rep.hits])
st = {h.airspace.name: h.status for h in rep.hits}
assert st.get("TEST CTR") == "inside" and st.get("R999 TEST RANGE") == "near"
assert any(w.startswith("AIRSPACE") for w in d.mission.warnings) and "Airspace check" in d.report.toPlainText()
lyr = L.find(L.AIRSPACE); print("   Airspace layer features:", lyr.featureCount())
assert lyr.featureCount() == 2
print("== QGIS: layer source (shapefile-like fields)")
al = QgsVectorLayer("Polygon?crs=EPSG:4326&field=NAME:string&field=CLASS:string&field=LOWER_VAL:double&field=LOWER_UOM:string"
                    "&field=LOWER_CODE:string&field=UPPER_VAL:double&field=UPPER_UOM:string&field=UPPER_CODE:string", "Class airspace", "memory")
f = QgsFeature(al.fields()); f.setGeometry(QgsGeometry.fromPolygonXY([[P(115.72, -32.31), P(115.76, -32.31), P(115.76, -32.28), P(115.72, -32.28)]]))
f.setAttributes(["LAYER CLASS C", "C", 700, "FT", "MSL", 8500, "FT", "MSL"]); al.dataProvider().addFeatures([f])
QgsProject.instance().addMapLayer(al)
d.asp_src.setCurrentIndex(d.asp_src.findData("layer")); d.asp_layer.setLayer(al); d.load_airspace(); pump(1500)
h = d.mission.airspace.hits[0]
print("  ", d.asp_lbl.text(), "|", h.airspace.name, h.status, round(h.floor_amsl_m - h.max_amsl_m))
assert h.status == "below"
d.height.setValue(250); pump(1500)
print("   at 250 m:", [(x.airspace.name, x.status) for x in d.mission.airspace.hits])
assert d.mission.airspace.hits[0].status == "inside"
print("== QGIS: download (fake transport) + PDF")
core.airspace._http_json = lambda url, headers=None, timeout=30, fetch=None: {"type": "FeatureCollection", "features": [
    {"type": "Feature", "properties": {"NAME": "FAA TEST D", "CLASS": "D", "UPPER_VAL": 2500, "UPPER_UOM": "FT", "UPPER_CODE": "MSL",
                                       "LOWER_VAL": 0, "LOWER_UOM": "FT", "LOWER_CODE": "SFC"},
     "geometry": {"type": "Polygon", "coordinates": [[[115.73, -32.31], [115.75, -32.31], [115.75, -32.28], [115.73, -32.28], [115.73, -32.31]]]}}]}
d.asp_src.setCurrentIndex(d.asp_src.findData("faa")); d.load_airspace(); pump(1500)
print("  ", d.asp_lbl.text())
assert "FAA" in d.asp_lbl.text() and d.mission.airspace.hits
from qgis.PyQt.QtWidgets import QFileDialog, QMessageBox
QMessageBox.information = staticmethod(lambda *a, **k: None)
QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (os.path.join(out, "asp.pdf"), ""))
d.brief_bg.setCurrentIndex(d.brief_bg.findData("none"))
d.fmt_cb.setCurrentIndex(d.fmt_cb.findData("brief_pdf")); d.export()
print("   PDF:", os.path.getsize(os.path.join(out, "asp.pdf")) // 1024, "KB")
d.asp_src.setCurrentIndex(d.asp_src.findData("none")); pump(1200)
assert d.mission.airspace is None
print("== ArcGIS: file + layer")
sys.path.insert(0, REPO + "/tests/fake_arcpy"); import arcpy
loader = importlib.machinery.SourceFileLoader("dmp_a", REPO + "/arcgis/DroneMissionPlanner/DroneMissionPlanner.pyt")
spec = importlib.util.spec_from_loader("dmp_a", loader); T = importlib.util.module_from_spec(spec); loader.exec_module(T)
T.core.airspace._http_json = core.airspace._http_json
sr = arcpy.SpatialReference(4326)
arcpy.INPUTS["blk"] = {"shapeType": "Polygon", "sr": sr, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in ring + [ring[0]]]), sr)]}
arcpy.INPUTS["asp"] = {"shapeType": "Polygon", "sr": sr, "geoms": [arcpy.Polygon(arcpy.Array([arcpy.Point(*q) for q in
                       [(115.72, -32.31), (115.76, -32.31), (115.76, -32.28), (115.72, -32.28), (115.72, -32.31)]]), sr)],
                       "attrs": [{"NAME": "ARC CTR", "TYPE": "CTR", "CLASS": "C", "LOWER": "SFC", "UPPER": "2500ft"}]}
gdb = tempfile.mkdtemp()
for src, extra in (("File (OpenAir / GeoJSON / ED-269)", {"asp_file": oa}), ("Airspace layer", {"asp_layer": "asp"}),
                   ("Download - FAA (USA)", {})):
    arcpy.MSGS.clear()
    ps = T.AreaMission().getParameterInfo(); dd = T._by_name(ps)
    for k, v in dict(in_features="blk", out_gdb=gdb, gsd=2.5, asp_source=src, **extra).items():
        dd[k].value = v; dd[k].altered = True
    T.AreaMission().updateParameters(ps); T.AreaMission().updateMessages(ps); T.AreaMission().execute(ps, None)
    msgs = [m for l, m in arcpy.MSGS]
    print("  %-34s" % src, [m for m in msgs if m.startswith(("Airspace:", "  Result"))], "| fc rows",
          len(arcpy.FCS[dd["out_airspace"].value]["rows"]) if dd["out_airspace"].value else 0)
    assert any("Result" in m and ("Controlled" in m) for m in msgs)
pl.unload(); print("airspace UI OK")
