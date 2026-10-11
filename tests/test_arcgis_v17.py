import sys, os, re, zipfile, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
exec(open(_os.path.join(REPO, "tests", "test_arcgis.py")).read().split('print("\\n== Area (drawn')[0])
exec(open(_os.path.join(REPO, "tests", "test_basemap.py")).read().split("x0, y0 = 381000")[0].split("from pyproj")[0])  # png(), fake()
import zlib, struct
def png(r, g, b):
    raw = b"".join(b"\x00" + bytes([r, g, b]) * 256 for _ in range(256))
    ch = lambda tag, d: struct.pack(">I", len(d)) + tag + d + struct.pack(">I", zlib.crc32(tag + d) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", 256, 256, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(raw)) + ch(b"IEND", b"")
urls = []
T.core.basemap.TileBasemap._http = lambda self, u: (urls.append(u), png(90, 160, 90))[1]
tpl = os.path.join(out, "DJIFLY-1234.kmz")
with zipfile.ZipFile(tpl, "w") as z:
    z.writestr("wpmz/template.kml", '<kml xmlns:wpml="x"><Document><wpml:missionConfig><wpml:droneInfo><wpml:droneEnumValue>99</wpml:droneEnumValue><wpml:droneSubEnumValue>3</wpml:droneSubEnumValue></wpml:droneInfo><wpml:payloadInfo><wpml:payloadEnumValue>77</wpml:payloadEnumValue></wpml:payloadInfo></wpml:missionConfig></Document></kml>')
print("== Mini 5 Pro: aircraft change sets timed trigger, notes shown")
ps = T.AreaMission().getParameterInfo(); dd = T._by_name(ps)
dd["aircraft"].value = "DJI Mini 5 Pro"; dd["aircraft"].altered = True
T.AreaMission().updateParameters(ps); T.AreaMission().updateMessages(ps)
print("  trigger:", dd["photo_trigger"].value, "| aircraft msg:", (dd["aircraft"].msg or ("", ""))[1][:70])
print("== Mini 5 Pro stop & shoot, DJI KMZ with template, brief basemap (Esri imagery)")
def run2(tool, first, **vals):
    arcpy.MSGS.clear(); ps = tool.getParameterInfo(); d = T._by_name(ps)
    for k, v in first.items(): d[k].value = v; d[k].altered = True
    tool.updateParameters(ps)
    for q in ps: q.hasBeenValidated = True
    for k, v in vals.items(): d[k].value = v; d[k].altered = True; d[k].hasBeenValidated = False
    tool.updateParameters(ps); tool.updateMessages(ps); tool.execute(ps, None); return d
d = run2(T.AreaMission(), {"aircraft": "DJI Mini 5 Pro"}, in_features="boundary", takeoff="tk", out_gdb=gdb, gsd=2.0,
        photo_trigger=T.TRIGGER_LABELS["stop"], stop_hover=1.5, export_folder=out,
        formats=[T.exporters.FORMATS["dji_kmz"][0], T.exporters.FORMATS["brief"][0]], dji_template=tpl,
        brief_basemap="Esri World Imagery", mission_name="Mini test")
print(" ", [m for l, m in arcpy.MSGS if "trigger" in m.lower() or "Flights/" in m])
k = [f for f in os.listdir(out) if f.startswith("Mini") and f.endswith(".kmz")]
tk = zipfile.ZipFile(os.path.join(out, k[0])).read("wpmz/template.kml").decode()
print("  kmz:", k[0], "enums", re.findall(r"EnumValue>(\d+)<", tk), "hover", tk.count(">hover<"), "takePhoto", tk.count(">takePhoto<"))
h = open(os.path.join(out, [f for f in os.listdir(out) if f.startswith("Mini") and f.endswith(".html")][0])).read()
print("  brief: tiles fetched", len(urls), "World_Imagery" in urls[0], "| images", h.count("<image"), "| attribution", "Esri" in h)
assert "99" in re.findall(r"EnumValue>(\d+)<", tk) and tk.count(">hover<") > 10 and h.count("<image") > 0
print("== Upload: replace DJI Fly mission")
ctrl = tempfile.mkdtemp(); target = os.path.join(ctrl, "ABC.kmz")
import shutil; shutil.copy(tpl, target)
src = os.path.join(out, k[0])
run(T.UploadMission(), in_file=src, method=[v for v in T._by_name(T.UploadMission().getParameterInfo())["method"].filter.list if v.startswith("Replace")][0], replace_file=target)
for l, m in arcpy.MSGS: print(" ", l, m)
print("  files on controller:", sorted(os.listdir(ctrl)), "| replaced has hover:", ">hover<" in zipfile.ZipFile(target).read("wpmz/template.kml").decode())
