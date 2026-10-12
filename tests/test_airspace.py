import sys, os, json, math, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import airspace as A, exporters
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
ti = Transformer.from_crs(4326, 32750, always_xy=True); from84 = lambda lo, la: ti.transform(lo, la)
x0, y0 = 381000, 6426000
lon0, lat0 = to84(x0 + 500, y0 + 300)
def dm(v, pos, neg):
    h = pos if v >= 0 else neg; v = abs(v); d = int(v); m = (v - d) * 60
    return "%d:%06.3f %s" % (d, m, h)
P = lambda lo, la: "%s %s" % (dm(la, "N", "S"), dm(lo, "E", "W"))
# OpenAir: a class C CTR over the west half (SFC-2500ft), a class C CTA step over everything with base 1500 ft,
# a restricted area circle 3 km east (near), a class G box, and an FIR (ignored)
oa = """* test airspace
AC C
AN TEST CTR
AL SFC
AH 2500ft AMSL
DP %s
DP %s
DP %s
DP %s

AC C
AN TEST CTA STEP
AL 1500 FT AMSL
AH FL125
DP %s
DP %s
DP %s
DP %s

AC R
AN R123 RANGE
AL SFC
AH 5000
V X=%s
DC 0.8

AC G
AN CLASS G AREA
AL SFC
AH 8500ft
DP %s
DP %s
DP %s

AC UKN
AY FIR
AN SOME FIR
AL SFC
AH UNL
DP %s
DP %s
DP %s
""" % (P(lon0 - 0.05, lat0 - 0.05), P(lon0 - 0.0005, lat0 - 0.05), P(lon0 - 0.0005, lat0 + 0.05), P(lon0 - 0.05, lat0 + 0.05),
       P(lon0 - 0.1, lat0 - 0.1), P(lon0 + 0.1, lat0 - 0.1), P(lon0 + 0.1, lat0 + 0.1), P(lon0 - 0.1, lat0 + 0.1),
       P(lon0 + 0.040, lat0),
       P(lon0 - 0.2, lat0 - 0.2), P(lon0 + 0.2, lat0 - 0.2), P(lon0, lat0 + 0.2),
       P(lon0 - 1, lat0 - 1), P(lon0 + 1, lat0 - 1), P(lon0, lat0 + 1))
asp = A.parse_openair(oa, "test.txt")
print("parsed:", [(a.name, a.label, a.limits_text, a.category) for a in asp])
assert len(asp) == 5 and asp[2].rings and len(asp[2].rings[0]) > 60
blk = [[(x0, y0), (x0 + 1000, y0), (x0 + 1000, y0 + 600), (x0, y0 + 600)]]
m = plan_mission(MissionParams(height_m=100), get_drone("DJI Matrice 4E"), Conditions(site_elevation_m=20), blk,
                 (x0 - 30, y0 - 30))
rep = A.check(m, asp, to84, from84, near_m=3000)
print(rep.text())
st = {h.airspace.name: h.status for h in rep.hits}
assert st["TEST CTR"] == "inside" and st["TEST CTA STEP"] == "below" and st["R123 RANGE"] == "near"
assert st["CLASS G AREA"] == "inside" and "SOME FIR" not in st
assert any(w.startswith("AIRSPACE") and "TEST CTR" in w for w in m.warnings)
assert rep.summary.startswith("Controlled")
# CTA floor: fly at 500 m -> now inside the CTA
m2 = plan_mission(MissionParams(height_m=450), get_drone("DJI Matrice 4E"), Conditions(site_elevation_m=20), blk,
                  (x0 - 30, y0 - 30))
r2 = A.check(m2, asp, to84)
assert {h.airspace.name: h.status for h in r2.hits}["TEST CTA STEP"] == "inside"
# limits parsing
for s, exp in [("SFC", "SFC"), ("1500ft AMSL", "1500 ft AMSL"), ("FL125", "FL125"), ("400 ft AGL", "400 ft AGL"),
               ("UNL", "UNL"), ("300m", "984 ft AMSL"), ("GND", "SFC")]:
    assert A.parse_limit(s).text() == exp, (s, A.parse_limit(s).text())
# FAA GeoJSON (class airspace fields) + UAS facility map + openAIP item + ED-269
faa = {"type": "FeatureCollection", "features": [
    {"type": "Feature", "properties": {"NAME": "TEST CLASS D", "CLASS": "D", "LOCAL_TYPE": "CLASS_D", "UPPER_VAL": 2600,
                                       "UPPER_UOM": "FT", "UPPER_CODE": "MSL", "LOWER_VAL": 0, "LOWER_UOM": "FT", "LOWER_CODE": "SFC"},
     "geometry": {"type": "Polygon", "coordinates": [[[lon0 - .02, lat0 - .02], [lon0 + .02, lat0 - .02], [lon0 + .02, lat0 + .02], [lon0 - .02, lat0 + .02], [lon0 - .02, lat0 - .02]]]}},
    {"type": "Feature", "properties": {"CEILING": 200, "UNIT": "Feet", "APT1_NAME": "TEST FIELD", "ARPT_COUNT": 1, "AIRSPACE_1": "D"},
     "geometry": {"type": "Polygon", "coordinates": [[[lon0 - .01, lat0 - .01], [lon0 + .01, lat0 - .01], [lon0 + .01, lat0 + .01], [lon0 - .01, lat0 + .01], [lon0 - .01, lat0 - .01]]]}}]}
oaip = {"items": [{"name": "OAIP TMA", "type": 7, "icaoClass": 2, "upperLimit": {"value": 85, "unit": 6, "referenceDatum": 2},
                   "lowerLimit": {"value": 0, "unit": 1, "referenceDatum": 0},
                   "geometry": {"type": "Polygon", "coordinates": [[[lon0 - .03, lat0 - .03], [lon0 + .03, lat0 - .03], [lon0, lat0 + .03], [lon0 - .03, lat0 - .03]]]}}],
        "totalPages": 1}
ed = {"title": "zones", "features": [{"identifier": "Z1", "name": "PARK ZONE", "restriction": "PROHIBITED", "geometry": [
    {"uomDimensions": "M", "lowerLimit": 0, "lowerVerticalReference": "AGL", "upperLimit": 120, "upperVerticalReference": "AGL",
     "horizontalProjection": {"type": "Circle", "center": [lon0, lat0], "radius": 300}}]}]}
more = A.parse_geojson(faa, "faa") + A.parse_geojson(oaip, "openAIP") + A.parse_geojson(ed, "ed269")
print("other formats:", [(a.name, a.label, a.limits_text, a.category) for a in more])
r3 = A.check(m, more, to84, near_m=1000)
s3 = {h.airspace.name: h for h in r3.hits}
assert s3["TEST CLASS D"].status == "inside" and s3["OAIP TMA"].status == "inside"
assert s3["PARK ZONE"].status == "inside" and s3["PARK ZONE"].airspace.category == "restricted"
uas = [h for h in r3.hits if h.airspace.uas_ceiling_m is not None][0]
assert uas.status == "inside" and "above the grid ceiling" in uas.message, uas.message
print(r3.text())
# online fetchers with a fake transport
calls = []
def fake(url, headers):
    calls.append((url, headers)); return json.dumps(faa if "arcgis" in url else oaip)
got = A.fetch_faa((lon0 - .1, lat0 - .1, lon0 + .1, lat0 + .1), fetch=fake)
got2 = A.fetch_openaip((lon0 - .1, lat0 - .1, lon0 + .1, lat0 + .1), "KEY", fetch=fake)
assert len(got) == 8 and len(got2) == 1 and calls[-1][1].get("x-openaip-api-key") == "KEY" and "f=geojson" in calls[0][0]
# brief / PDF carry the airspace section
out = tempfile.mkdtemp()
exporters.export(m, "brief", os.path.join(out, "b.html"), to84)
h = open(os.path.join(out, "b.html"), encoding="utf-8").read()
assert "<h2>Airspace</h2>" in h and "TEST CTR" in h
exporters.export(m, "brief_pdf", os.path.join(out, "b.pdf"), to84)
print("airspace OK")
