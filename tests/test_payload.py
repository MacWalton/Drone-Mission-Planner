import sys, os, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, units as U
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
d = get_drone("DJI Matrice 350 RTK + P1")
base = EnduranceModel(d, Conditions(), 100)
print("M350 empty: %.1f min usable, flight power %.0f W" % (base.usable_minutes, base.flight_power_w))
items = [PayloadItem("Zenmuse L2 LiDAR", 0.905, 28), ("Spotlight", 0.4, 30), {"name": "Methane sensor", "mass_kg": U.kg(1.1, "lb"), "power_w": 6}]
c = Conditions(payload_items=items)
m = EnduranceModel(d, c, 100)
print("with 3 items: payload %.3f kg, %.0f W -> %.1f min (%.1f min weight only)" % (c.total_payload_kg, c.accessory_power_w, m.usable_minutes, m.flight_minutes))
w_only = EnduranceModel(d, Conditions(payload_kg=c.total_payload_kg), 100)
assert abs(w_only.usable_minutes - m.flight_minutes) < 1e-9
# energy check: hovering for usable minutes consumes exactly the usable battery
import math
frac = m._fraction(m.usable_minutes * 60, 1.0); print("fraction used over usable minutes at best speed: %.4f" % frac)
over = Conditions(payload_items=[("Heavy rig", 3.0, 50)]); print("over limit warning:", EnduranceModel(d, over, 100).warnings[:1])
x0, y0 = 381000, 6426000
blk = [[(x0, y0), (x0 + 800, y0), (x0 + 800, y0 + 800), (x0, y0 + 800)]]
mp = MissionParams(height_m=80, mass_unit="lb")
m0 = plan_mission(mp, d, Conditions(), blk, (x0, y0))
m1 = plan_mission(mp, d, c, blk, (x0, y0))
print("mission: %d flights (no payload) vs %d flights (payload list)" % (len(m0.flights), len(m1.flights)))
print("\n".join(l for l in m1.report().splitlines() if l.startswith(("Payload", "  -", "  Take-off"))))
out = tempfile.mkdtemp(); exporters.export(m1, "brief", os.path.join(out, "b.html"), to84)
h = open(os.path.join(out, "b.html")).read(); print("brief payload table:", "<h2>Payload</h2>" in h and "Spotlight" in h and "Methane sensor" in h)
import shutil; shutil.copy(os.path.join(out, "b.html"), REPO + "/tests/out/payload_brief.html")
