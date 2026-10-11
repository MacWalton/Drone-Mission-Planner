import sys, os, datetime as dt, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import daylight as D, exporters
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000            # near Perth, WA (UTM 50S)
# 1. sun events vs published values (Geoscience Australia / NOAA, +-2 min)
r = D.day_report(-31.95, 115.86, dt.date(2026, 10, 15), 8)
print("Perth 15 Oct 2026: sunrise %s sunset %s civil %s-%s noon %s max %.1f best %s"
      % (D.clock(r.sunrise), D.clock(r.sunset), D.clock(r.dawn), D.clock(r.dusk), D.clock(r.noon), r.max_elev, r.best_text))
assert abs(r.sunrise - (5 * 60 + 37)) <= 3 and abs(r.sunset - (18 * 60 + 28)) <= 3
assert abs(r.noon - (12 * 60 + 2)) <= 3 and 65 < r.max_elev < 68
# 2. mission: big block, arrive 04:30 -> first flight in twilight/dark, flags + best light
blk = [[(x0, y0), (x0 + 4000, y0), (x0 + 4000, y0 + 2500), (x0, y0 + 2500)]]
lp = LogisticsParams(start_time_min=4 * 60 + 30, flight_date="2026-10-15", utc_offset_h=8, battery_sets=8)
m = plan_mission(MissionParams(gsd_cm=2.5), get_drone("DJI Matrice 4E"), Conditions(), blk, (x0 - 30, y0 - 30),
                 logistics=lp)
rep = D.analyse(m, to84)
st = [(f.flight, f.status) for f in rep.flights]
print("flights:", [(f.flight, D.clock(f.start_min), f.sun_text) for f in rep.flights][:6])
assert rep.flights[0].status in ("night", "twilight") and any(s == "ok" for _, s in st)
assert any(w.startswith("DAYLIGHT") for w in m.warnings) and any(w.startswith("LIGHT") for w in m.warnings)
assert "Flight date & light" in m.report()
# 3. no date -> nothing
m2 = plan_mission(MissionParams(gsd_cm=2.5), get_drone("DJI Matrice 4E"), Conditions(), blk, (x0 - 30, y0 - 30))
assert D.analyse(m2, to84) is None and m2.daylight is None
# 4. wrong time zone flagged
m3 = plan_mission(MissionParams(gsd_cm=2.5), get_drone("DJI Matrice 4E"), Conditions(), blk, (x0 - 30, y0 - 30),
                  logistics=LogisticsParams(flight_date="2026-10-15", utc_offset_h=-5))
D.analyse(m3, to84)
assert any(w.startswith("TIME ZONE") for w in m3.warnings)
# 5. brief + PDF carry date, daylight, sun column
out = tempfile.mkdtemp()
exporters.export(m, "brief", os.path.join(out, "b.html"), to84)
h = open(os.path.join(out, "b.html"), encoding="utf-8").read()
assert "Flight date" in h and "Thu 15 Oct 2026" in h and "Best light" in h and "<th>Sun</th>" in h
exporters.export(m, "brief_pdf", os.path.join(out, "b.pdf"), to84)
try:
    import pypdf
    txt = "\n".join(p.extract_text() for p in pypdf.PdfReader(os.path.join(out, "b.pdf")).pages)
    assert "Flight date" in txt and "Best light" in txt and ("NIGHT" in txt or "TWILIGHT" in txt)
except ImportError:
    pass
print("default name:", default_mission_name(AREA, "Lot 7", "15 Oct 2026"))
print("daylight OK")
