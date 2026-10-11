import sys, os, re, zipfile, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
blk = [[(x0, y0), (x0 + 500, y0), (x0 + 500, y0 + 400), (x0, y0 + 400)]]
out = tempfile.mkdtemp()
EXPECT = {"DJI Matrice 4E": ("99", "0", "88"), "DJI Matrice 4T": ("99", "1", "89"),
          "DJI Matrice 4D (Dock 3)": ("100", "0", "98"), "DJI Matrice 4TD (Dock 3)": ("100", "1", "99"),
          "DJI Matrice 3D (Dock 2)": ("91", "0", "80"), "DJI Matrice 3TD (Dock 2)": ("91", "1", "81"),
          "DJI Mavic 3 Enterprise": ("77", "0", "66"), "DJI Mavic 3 Thermal (3T)": ("77", "1", "67"),
          "DJI Mavic 3 Multispectral (3M)": ("77", "2", "68"), "DJI Matrice 30": ("67", "0", "52"),
          "DJI Matrice 30T": ("67", "1", "53"), "DJI Matrice 350 RTK + P1": ("89", "0", "50"),
          "DJI Matrice 300 RTK + P1": ("60", "0", "50"), "DJI Matrice 350 RTK + L1": ("89", "0", "90742"), "DJI Matrice 300 RTK + L1": ("60", "0", "90742")}
tpl = os.path.join(out, "pilot2.kmz")
with zipfile.ZipFile(tpl, "w") as z:
    z.writestr("wpmz/template.kml", '<kml><wpml:droneEnumValue>103</wpml:droneEnumValue><wpml:droneSubEnumValue>0</wpml:droneSubEnumValue><wpml:payloadEnumValue>84</wpml:payloadEnumValue></kml>')
n = 0
for name in drone_names():
    d = get_drone(name)
    if d.platform != "dji_wpml" or name == "DJI Phantom 4 RTK":
        continue
    m = plan_mission(MissionParams(gsd_cm=2.5), d, Conditions(), blk, (x0 - 30, y0 - 30))
    f = os.path.join(out, "%d.kmz" % n); n += 1
    exporters.write_dji_kmz(m, f, to84, 0, template=tpl if d.needs_dji_template else None)
    tk = zipfile.ZipFile(f).read("wpmz/template.kml").decode()
    got = (re.search(r"droneEnumValue>(\d+)<", tk).group(1), re.search(r"droneSubEnumValue>(\d+)<", tk).group(1),
           re.search(r"payloadEnumValue>(\d+)<", tk).group(1))
    if name in EXPECT:
        assert got == EXPECT[name], (name, got)
    else:
        assert d.needs_dji_template and got == ("103", "0", "84"), (name, got)
    print("  %-30s h %5.1f m  GSD %.2f  %d flight(s) %4.1f min  codes %s%s" % (
        name, m.height_m, m.gsd_cm, len(m.flights), m.total_time_s / 60, ",".join(got),
        "  (from template)" if d.needs_dji_template else ""))
assert not get_drone("DJI Matrice 4E").needs_dji_template and get_drone("DJI Matrice 400 + P1").needs_dji_template
assert get_drone("DJI Mini 5 Pro").needs_dji_template and not get_drone("DJI Mini 4 Pro").needs_dji_template
print("enterprise profiles OK:", n)
