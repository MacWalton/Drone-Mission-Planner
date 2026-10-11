import sys, os, re, zipfile, csv, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters
from pyproj import Transformer
import xml.dom.minidom as md
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
blk = [[(x0, y0), (x0 + 300, y0), (x0 + 300, y0 + 200), (x0, y0 + 200)]]
print("== profiles")
for n in ("DJI Mini 3 Pro", "DJI Mini 4 Pro", "DJI Mini 4 Pro (Plus battery)", "DJI Mini 5 Pro", "DJI Mini 5 Pro (Plus battery)", "DJI Mini 4 Pro (Litchi/WPML)"):
    d = get_drone(n); m = EnduranceModel(d, Conditions(), 60)
    print("  %-30s %s %s GSD@60m %.2f cm  usable %.1f min  default export %s, photo %s" % (n, d.platform, d.camera.image_width_px, photogrammetry.gsd_from_height(d.camera, 60), m.usable_minutes, exporters.DEFAULT_FORMAT_FOR_PLATFORM.get(d.platform), d.photo_trigger))
out = tempfile.mkdtemp()
# fake DJI Fly template KMZ (as created by DJI Fly on the controller)
tpl = os.path.join(out, "9A3F2C1E-DJIFLY.kmz")
with zipfile.ZipFile(tpl, "w") as z:
    z.writestr("wpmz/template.kml", '<kml xmlns:wpml="x"><Document><wpml:missionConfig><wpml:droneInfo><wpml:droneEnumValue>99</wpml:droneEnumValue><wpml:droneSubEnumValue>3</wpml:droneSubEnumValue></wpml:droneInfo><wpml:payloadInfo><wpml:payloadEnumValue>77</wpml:payloadEnumValue></wpml:payloadInfo></wpml:missionConfig></Document></kml>')
d = get_drone("DJI Mini 5 Pro")
for mode in ("timed", "stop", "distance"):
    m = plan_mission(MissionParams(height_m=60, photo_trigger=mode), d, Conditions(), blk, (x0 - 20, y0 - 20))
    f = os.path.join(out, "m5_%s.kmz" % mode)
    exporters.write_dji_kmz(m, f, to84, 0, template=tpl)
    z = zipfile.ZipFile(f); tk = z.read("wpmz/template.kml").decode(); md.parseString(tk); md.parseString(z.read("wpmz/waylines.wpml"))
    enum = re.findall(r"<wpml:(droneEnumValue|droneSubEnumValue|payloadEnumValue)>(\d+)<", tk)
    print("  %-8s %d flights %.1f min | placemarks %d | takePhoto %d hover %d multipleDistance %d | enums %s" % (
        mode, len(m.flights), m.total_time_s / 60, tk.count("<Placemark>"), tk.count(">takePhoto<"), tk.count(">hover<"), tk.count("multipleDistance"), enum))
    print("           report:", m.trigger_text())
# Litchi for Mini 4 Pro
d4 = get_drone("DJI Mini 4 Pro")
for mode in ("distance", "timed", "stop"):
    m = plan_mission(MissionParams(height_m=60, photo_trigger=mode), d4, Conditions(), blk, (x0 - 20, y0 - 20))
    files = exporters.write_litchi_csv(m, os.path.join(out, "l_%s.csv" % mode), to84, 0)
    rows = list(csv.DictReader(open(files[0])))
    print("  litchi %-8s files %d | rows %d | distint %s | timeint %s | stay+photo rows %d" % (
        mode, len(files), len(rows), sorted({r["photo_distinterval"] for r in rows})[:3], sorted({r["photo_timeinterval"] for r in rows})[:3],
        sum(1 for r in rows if r["actiontype1"] == "0" and r["actiontype2"] == "1")))
try:
    exporters.read_dji_template(os.path.join(out, "l_distance.csv"))
except Exception as e: print("  bad template ->", type(e).__name__)
