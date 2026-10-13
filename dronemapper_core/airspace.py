# SPDX-License-Identifier: GPL-2.0-only
"""Airspace check: does the planned mission enter controlled, restricted or
other notable airspace?

Sources (all end up as Airspace objects in WGS84):

* OpenAir files (.txt / .air) - the common worldwide airspace text format;
* GeoJSON / ED-269 (EU UAS geographical zones) files;
* any GIS layer (shapefile, GeoPackage, Airservices / FAA downloads ...) whose
  attributes are recognised by common field names (class, type, name, upper /
  lower limits);
* online: FAA ArcGIS services (USA, no key) and openAIP (worldwide, free API
  key - check openAIP's data licence for commercial use).

The check follows every flight path (positions every ~25 m, height above
ground / sea level from the plan and the DEM), and reports each airspace the
mission is inside (horizontally and vertically), flies beneath (inside the
footprint but below the floor) or passes near.

This is a planning aid, not an authoritative airspace service: always confirm
with your aviation authority's approved tools (CASA-verified apps, FAA
B4UFLY / LAANC, NOTAMs) before flight.
"""
import json
import math
import re
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import List, Optional

FT = 0.3048
NM = 1852.0
UA = "DroneMissionPlanner/1.12 (airspace check)"

CONTROLLED_KINDS = {"CTR", "CTA", "TMA", "MCTR", "CLASS B", "CLASS C", "CLASS D", "CLASS E", "ATZ", "MATZ",
                    "TIZ", "TIA", "HTZ"}
RESTRICTED_KINDS = {"R", "RESTRICTED", "P", "PROHIBITED", "TFR", "NATIONAL DEFENSE", "NSUFR", "PROTECTED",
                    "OVERFLIGHT RESTRICTION", "UAS PROHIBITED", "UAS RESTRICTED"}
DANGER_KINDS = {"D", "DANGER", "ALERT", "WARNING", "MOA", "TRA", "TSA", "MILITARY", "W", "A", "MTA"}
IGNORE_KINDS = {"FIR", "UIR", "ACC SECTOR", "FIS SECTOR", "VFR SECTOR", "AIRWAY", "LTA", "UTA", "ADIZ", "TRP"}


@dataclass
class Limit:
    value: float = 0.0          # metres
    ref: str = "GND"            # GND | AGL | AMSL | FL | UNL

    def amsl(self, ground_m):
        """Altitude AMSL (metres) of this limit over ground at ground_m."""
        if self.ref == "UNL":
            return 1e6
        if self.ref == "GND":
            return ground_m
        if self.ref == "AGL":
            return ground_m + self.value
        return self.value                       # AMSL, FL (pressure altitude ~ AMSL)

    def text(self):
        if self.ref == "UNL":
            return "UNL"
        if self.ref == "GND" or (self.ref in ("AGL",) and self.value <= 0):
            return "SFC"
        if self.ref == "FL":
            return "FL%03d" % round(self.value / FT / 100)
        return "%.0f ft %s" % (self.value / FT, "AGL" if self.ref == "AGL" else "AMSL")


@dataclass
class Airspace:
    name: str
    cls: str = ""               # ICAO class A..G ('' if not classed)
    kind: str = ""              # CTR, CTA, R, P, D, ATZ, UAS zone, TFR, ...
    lower: Limit = field(default_factory=Limit)
    upper: Limit = field(default_factory=lambda: Limit(0, "UNL"))
    rings: list = field(default_factory=list)   # [[(lon, lat), ...], ...] outer ring(s)
    source: str = ""
    remarks: str = ""
    uas_ceiling_m: Optional[float] = None       # FAA UAS facility map: max AGL for automatic approval
    _bbox: tuple = None

    @property
    def category(self):
        k = self.kind.upper()
        if k in IGNORE_KINDS:
            return "ignore"
        if self.uas_ceiling_m is not None:
            return "controlled"
        if k in RESTRICTED_KINDS or k.startswith(("RESTRICT", "PROHIB")):
            return "restricted"
        if self.cls in ("A", "B", "C", "D", "E") or k in CONTROLLED_KINDS:
            return "controlled"
        if k in DANGER_KINDS:
            return "danger"
        if self.cls in ("F", "G"):
            return "uncontrolled"
        return "other"

    @property
    def label(self):
        bits = []
        if self.cls:
            bits.append("Class %s" % self.cls)
        if self.kind and self.kind.upper() not in ("CLASS %s" % self.cls,):
            bits.append(self.kind)
        return " ".join(bits) or "Airspace"

    @property
    def limits_text(self):
        return "%s – %s" % (self.lower.text(), self.upper.text())

    def bbox(self):
        if self._bbox is None:
            pts = [p for r in self.rings for p in r]
            self._bbox = (min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts),
                          max(p[1] for p in pts)) if pts else (0, 0, 0, 0)
        return self._bbox

    def contains(self, lon, lat):
        inside = False
        for r in self.rings:
            if _pip(lon, lat, r):
                inside = not inside
        return inside

    def distance_m(self, lon, lat):
        """Approximate distance (m) from a point to the airspace boundary."""
        kx, ky = 111320.0 * math.cos(math.radians(lat)), 110540.0
        best = 1e18
        for r in self.rings:
            for a, b in zip(r[:-1], r[1:]):
                ax, ay = (a[0] - lon) * kx, (a[1] - lat) * ky
                bx, by = (b[0] - lon) * kx, (b[1] - lat) * ky
                dx, dy = bx - ax, by - ay
                L2 = dx * dx + dy * dy
                t = 0 if L2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / L2))
                best = min(best, math.hypot(ax + dx * t, ay + dy * t))
        return best


def _pip(x, y, ring):
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-15) + xi:
            inside = not inside
        j = i
    return inside


# ------------------------------------------------------------------ limits
def parse_limit(s):
    """'SFC', 'GND', '1500ft', '1500 FT AMSL', '1000 AGL', 'FL65', 'UNL', '300m', '2500 MSL' -> Limit."""
    if s is None:
        return Limit(0, "GND")
    t = str(s).strip().upper().replace(",", "")
    if t in ("", "SFC", "GND", "SURFACE", "0", "0 FT", "0FT", "0 AGL", "0FT AGL", "0 FT AGL"):
        return Limit(0, "GND")
    if t.startswith("UNL") or "UNLIM" in t:
        return Limit(0, "UNL")
    m = re.match(r"^FL\s*(\d+)", t)
    if m:
        return Limit(int(m.group(1)) * 100 * FT, "FL")
    m = re.search(r"(-?\d+(?:\.\d+)?)\s*(FT|F|M|METRES?|METERS?)?", t)
    if not m:
        return Limit(0, "GND")
    v = float(m.group(1))
    unit = m.group(2) or "FT"
    v = v if unit.startswith("M") else v * FT
    ref = "AGL" if ("AGL" in t or "AGND" in t or "ASFC" in t or "SFC" in t[m.end():]) else "AMSL"
    return Limit(v, ref)


def _limit(value, unit, ref):
    """From separate value / unit / reference fields (FAA, openAIP, ED-269)."""
    if value is None or value == "":
        return Limit(0, "GND")
    try:
        v = float(value)
    except (TypeError, ValueError):
        return parse_limit(value)
    u = str(unit or "FT").upper()
    r = str(ref or "MSL").upper()
    if v < -9000:                                  # FAA -9998: up to (not incl.) class A
        return Limit(18000 * FT, "AMSL")
    if u in ("FL", "6"):
        return Limit(v * 100 * FT, "FL")
    vm = v if u in ("M", "METER", "METRE", "0") else v * FT
    if r in ("SFC", "GND", "0", "AGL", "AGND") or "SFC" in r or "AGL" in r:
        return Limit(vm, "AGL") if vm > 0 else Limit(0, "GND")
    return Limit(vm, "AMSL")


# ------------------------------------------------------------------ OpenAir
_COORD = re.compile(r"(\d+(?:\.\d+)?)(?::(\d+(?:\.\d+)?))?(?::(\d+(?:\.\d+)?))?\s*([NS])\s*[, ]?\s*"
                    r"(\d+(?:\.\d+)?)(?::(\d+(?:\.\d+)?))?(?::(\d+(?:\.\d+)?))?\s*([EW])", re.I)


def _dms(d, m, s, h):
    v = float(d) + float(m or 0) / 60 + float(s or 0) / 3600
    return -v if h.upper() in ("S", "W") else v


def _coord(text):
    m = _COORD.search(text)
    if not m:
        raise ValueError("bad coordinate: %s" % text)
    g = m.groups()
    return _dms(g[4], g[5], g[6], g[7]), _dms(g[0], g[1], g[2], g[3])      # (lon, lat)


def _dest(lon, lat, brg_deg, dist_m):
    R = 6371008.8
    br, la, lo = math.radians(brg_deg), math.radians(lat), math.radians(lon)
    d = dist_m / R
    la2 = math.asin(math.sin(la) * math.cos(d) + math.cos(la) * math.sin(d) * math.cos(br))
    lo2 = lo + math.atan2(math.sin(br) * math.sin(d) * math.cos(la), math.cos(d) - math.sin(la) * math.sin(la2))
    return math.degrees(lo2), math.degrees(la2)


def _bearing_dist(c, p):
    kx, ky = 111320.0 * math.cos(math.radians(c[1])), 110540.0
    dx, dy = (p[0] - c[0]) * kx, (p[1] - c[1]) * ky
    return math.degrees(math.atan2(dx, dy)) % 360, math.hypot(dx, dy)


def _arc(c, r_m, a0, a1, cw, step=5.0):
    if cw:
        span = (a1 - a0) % 360 or 360
        angs = [a0 + k * step for k in range(int(span / step) + 1)] + [a0 + span]
    else:
        span = (a0 - a1) % 360 or 360
        angs = [a0 - k * step for k in range(int(span / step) + 1)] + [a0 - span]
    return [_dest(c[0], c[1], a, r_m) for a in angs]


def parse_openair(text, source="OpenAir"):
    out, cur, center, cw = [], None, None, True

    def close():
        if cur and cur["pts"] and len(cur["pts"]) >= 3:
            pts = cur["pts"]
            if pts[0] != pts[-1]:
                pts = pts + [pts[0]]
            out.append(Airspace(cur["an"] or "(unnamed)", cur["cls"], cur["kind"], cur["al"], cur["ah"], [pts],
                                source))
    for raw in text.splitlines():
        line = raw.split("*", 1)[0].strip() if not raw.lstrip().startswith("*") else ""
        if not line:
            continue
        key, _, val = line.partition(" ")
        key = key.upper()
        val = val.strip()
        if key == "AC":
            close()
            c = val.upper()
            cls = c if c in ("A", "B", "C", "D", "E", "F", "G") else ""
            kind = {"R": "R", "Q": "D", "P": "P", "CTR": "CTR", "GP": "Glider prohibited", "W": "Wave",
                    "TMZ": "TMZ", "RMZ": "RMZ", "UKN": "", "GSEC": "Gliding sector"}.get(c, "" if cls else c)
            cur = {"an": "", "cls": cls, "kind": kind, "al": Limit(0, "GND"), "ah": Limit(0, "UNL"), "pts": []}
            center, cw = None, True
        elif cur is None:
            continue
        elif key == "AN":
            cur["an"] = val
        elif key == "AY":
            cur["kind"] = val.upper()
        elif key == "AL":
            cur["al"] = parse_limit(val)
        elif key == "AH":
            cur["ah"] = parse_limit(val)
        elif key == "V":
            k, _, v = val.partition("=")
            if k.strip().upper() == "X":
                center = _coord(v)
            elif k.strip().upper() == "D":
                cw = v.strip() != "-"
        elif key == "DP":
            cur["pts"].append(_coord(val))
        elif key == "DC" and center:
            r = float(val.split()[0]) * NM
            cur["pts"] = _arc(center, r, 0, 360, True)
        elif key == "DA" and center:
            r, a0, a1 = [float(x) for x in val.replace(" ", "").split(",")[:3]]
            cur["pts"] += _arc(center, r * NM, a0, a1, cw)
        elif key == "DB" and center:
            p0, p1 = val.split(",", 1)[0], val.split(",", 1)[1] if "," in val else val
            c0, c1 = _coord(p0), _coord(p1)
            b0, r = _bearing_dist(center, c0)
            b1, _ = _bearing_dist(center, c1)
            cur["pts"] += _arc(center, r, b0, b1, cw)
    close()
    return out


# ------------------------------------------------------------------ GeoJSON / attributes
OPENAIP_TYPE = {0: "", 1: "R", 2: "D", 3: "P", 4: "CTR", 5: "TMZ", 6: "RMZ", 7: "TMA", 8: "TRA", 9: "TSA",
                10: "FIR", 11: "UIR", 12: "ADIZ", 13: "ATZ", 14: "MATZ", 15: "AIRWAY", 16: "MTR", 17: "ALERT",
                18: "WARNING", 19: "PROTECTED", 20: "HTZ", 21: "Gliding sector", 22: "TRP", 23: "TIZ", 24: "TIA",
                25: "MTA", 26: "CTA", 27: "ACC SECTOR", 28: "Aerial sporting", 29: "OVERFLIGHT RESTRICTION",
                30: "MRT", 31: "TFR", 32: "VFR SECTOR", 33: "FIS SECTOR", 34: "LTA", 35: "UTA", 36: "MCTR"}
OPENAIP_CLASS = {0: "A", 1: "B", 2: "C", 3: "D", 4: "E", 5: "F", 6: "G", 8: ""}
OPENAIP_UNIT = {0: "M", 1: "FT", 6: "FL"}
OPENAIP_REF = {0: "GND", 1: "MSL", 2: "STD"}


def _get(props, *names):
    low = {str(k).lower(): v for k, v in props.items()}
    for n in names:
        v = low.get(n.lower())
        if v not in (None, ""):
            return v
    return None


def _rings_from_geometry(g):
    if not g:
        return []
    t = g.get("type")
    c = g.get("coordinates") or []
    if t == "Polygon":
        return [[tuple(p[:2]) for p in r] for r in c]
    if t == "MultiPolygon":
        return [[tuple(p[:2]) for p in r] for poly in c for r in poly]
    return []


def from_attributes(props, rings, source=""):
    """Airspace from a feature's attributes (openAIP, FAA ADDS / SUA / UAS facility map,
    Airservices / generic shapefiles) and its WGS84 rings. None if not an airspace."""
    p = props or {}
    if "icaoClass" in p or ("type" in p and isinstance(p.get("type"), int) and "upperLimit" in p):  # openAIP
        ul, ll = p.get("upperLimit") or {}, p.get("lowerLimit") or {}
        cls = OPENAIP_CLASS.get(p.get("icaoClass"), "") if isinstance(p.get("icaoClass"), int) else str(p.get("icaoClass") or "")
        kind = OPENAIP_TYPE.get(p.get("type"), "") if isinstance(p.get("type"), int) else str(p.get("type") or "")

        def lim(d):
            u = OPENAIP_UNIT.get(d.get("unit"), d.get("unit"))
            r = OPENAIP_REF.get(d.get("referenceDatum"), d.get("referenceDatum"))
            if r == "GND" and not d.get("value"):
                return Limit(0, "GND")
            return _limit(d.get("value"), u, "AGL" if r == "GND" else "MSL")
        return Airspace(str(p.get("name") or "(unnamed)"), cls, kind, lim(ll), lim(ul), rings, source or "openAIP",
                        " ".join(x for x in ("on request" if p.get("onRequest") else "",
                                             "by NOTAM" if p.get("byNotam") else "") if x))
    if _get(p, "CEILING") is not None and _get(p, "APT1_NAME", "APT1_FAAID", "ARPT_COUNT") is not None:   # FAA UASFM
        ceil_ft = float(_get(p, "CEILING") or 0)
        apt = _get(p, "APT1_NAME", "APT1_ICAO", "APT1_FAAID") or "airport"
        return Airspace("UAS facility map grid (%s)" % apt, _get(p, "AIRSPACE_1") or "", "UAS facility map",
                        Limit(0, "GND"), Limit(400 * FT, "AGL"), rings, source or "FAA UAS facility map",
                        "LAANC / authorization ceiling %.0f ft AGL" % ceil_ft, uas_ceiling_m=ceil_ft * FT)
    name = _get(p, "NAME", "name", "AN", "ARSP_NAME", "airspace_name", "title", "designator", "IDENT")
    cls = str(_get(p, "CLASS", "icao_class", "class_", "AIRSPACE_CLASS", "ARSP_CLASS", "ac") or "").strip().upper()
    kind = str(_get(p, "LOCAL_TYPE", "TYPE_CODE", "TYPE", "type", "ARSP_TYPE", "category", "AY", "airspace_type")
               or "").strip().upper()
    if cls.startswith("CLASS "):
        cls = cls[6:]
    if len(cls) > 1 and cls not in ("A", "B", "C", "D", "E", "F", "G"):
        kind, cls = kind or cls, ""
    kind = {"CLASS_B": "CTA", "CLASS_C": "CTA", "CLASS_D": "CTR", "CLASS_E2": "CTR", "CLASS_E3": "CTA",
            "CLASS_E4": "CTA", "CLASS_E5": "CTA", "MOA": "MOA", "RA": "R", "PA": "P", "WA": "W", "AA": "ALERT",
            "NSA": "R"}.get(kind, kind)
    if not (name or cls or kind):
        return None
    lo = _get(p, "LOWER_VAL", "lower_val", "lower", "AL", "floor", "lowerlimit", "LOWER_LIMIT", "BASE", "LWR_LIMIT")
    hi = _get(p, "UPPER_VAL", "upper_val", "upper", "AH", "ceiling_ft", "upperlimit", "UPPER_LIMIT", "TOP", "UPR_LIMIT")
    lower = _limit(lo, _get(p, "LOWER_UOM", "lower_uom", "lower_unit"), _get(p, "LOWER_CODE", "lower_code", "lower_ref")) \
        if _get(p, "LOWER_UOM", "LOWER_CODE", "lower_unit") is not None else parse_limit(lo)
    upper = _limit(hi, _get(p, "UPPER_UOM", "upper_uom", "upper_unit"), _get(p, "UPPER_CODE", "upper_code", "upper_ref")) \
        if _get(p, "UPPER_UOM", "UPPER_CODE", "upper_unit") is not None else (parse_limit(hi) if hi is not None
                                                                                  else Limit(0, "UNL"))
    return Airspace(str(name or kind or "Airspace"), cls if cls in ("A", "B", "C", "D", "E", "F", "G") else "", kind,
                    lower, upper, rings, source, str(_get(p, "REMARKS", "remarks", "WKHR_RMK", "message") or "")[:200])


def parse_ed269(obj, source="ED-269"):
    out = []
    feats = obj.get("features") or []
    for f in feats:
        restr = str(f.get("restriction") or "").upper()
        kind = {"PROHIBITED": "UAS prohibited", "REQ_AUTHORISATION": "UAS authorisation", "CONDITIONAL":
                "UAS conditional", "NO_RESTRICTION": "UAS zone"}.get(restr, "UAS zone")
        for g in f.get("geometry") or []:
            hp = g.get("horizontalProjection") or {}
            if hp.get("type") == "Circle":
                c = hp.get("center") or [0, 0]
                rings = [_arc((c[0], c[1]), float(hp.get("radius", 0)), 0, 360, True)]
            else:
                rings = _rings_from_geometry(hp)
            u = (g.get("uomDimensions") or "M").upper()
            lo = _limit(g.get("lowerLimit") or 0, "M" if u == "M" else "FT", g.get("lowerVerticalReference") or "AGL")
            hi = _limit(g.get("upperLimit") if g.get("upperLimit") is not None else 120,
                        "M" if u == "M" else "FT", g.get("upperVerticalReference") or "AGL")
            a = Airspace(str(f.get("name") or f.get("identifier") or "UAS zone"), "", kind, lo, hi, rings, source,
                         str(f.get("message") or "")[:200])
            if restr == "PROHIBITED":
                a.kind = "UAS prohibited"
            out.append(a)
    return out


def parse_geojson(obj, source="GeoJSON"):
    if isinstance(obj, dict) and obj.get("features") and isinstance(obj["features"], list) and \
            obj["features"] and "horizontalProjection" in json.dumps(obj["features"][0])[:4000]:
        return parse_ed269(obj, source)
    if isinstance(obj, dict) and isinstance(obj.get("items"), list):          # openAIP API page
        return [a for a in (from_attributes(it, _rings_from_geometry(it.get("geometry")), source)
                            for it in obj["items"]) if a and a.rings]
    out = []
    for f in obj.get("features", []) if isinstance(obj, dict) else []:
        a = from_attributes(f.get("properties") or {}, _rings_from_geometry(f.get("geometry")), source)
        if a and a.rings:
            out.append(a)
    return out


def load_file(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    text = raw.decode("utf-8", "replace")
    name = path.replace("\\", "/").split("/")[-1]
    if text.lstrip().startswith(("{", "[")):
        return parse_geojson(json.loads(text), name)
    return parse_openair(text, name)


# ------------------------------------------------------------------ online
FAA_SERVICES = [
    ("Class_Airspace", "FAA class airspace"),
    ("Special_Use_Airspace", "FAA special use airspace"),
    ("FAA_UAS_FacilityMap_Data", "FAA UAS facility map"),
    ("Prohibited_Areas", "FAA prohibited areas"),
]
FAA_BASE = "https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/%s/FeatureServer/0/query"
OPENAIP_URL = "https://api.core.openaip.net/api/airspaces"


def _http_json(url, headers=None, timeout=30, fetch=None):
    if fetch:
        return json.loads(fetch(url, headers or {}))
    req = urllib.request.Request(url, headers=dict({"User-Agent": UA}, **(headers or {})))
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def fetch_faa(bbox, fetch=None, log=None):
    """USA: FAA class airspace, special use airspace, prohibited areas and UAS facility
    map grids intersecting bbox (lon0, lat0, lon1, lat1)."""
    out = []
    for svc, label in FAA_SERVICES:
        q = {"where": "1=1", "geometry": "%f,%f,%f,%f" % bbox, "geometryType": "esriGeometryEnvelope",
             "inSR": "4326", "spatialRel": "esriSpatialRelIntersects", "outFields": "*", "outSR": "4326",
             "f": "geojson", "resultRecordCount": "2000"}
        try:
            obj = _http_json((FAA_BASE % svc) + "?" + urllib.parse.urlencode(q), fetch=fetch)
            got = parse_geojson(obj, label)
            out += got
            if log:
                log("%s: %d feature(s)" % (label, len(got)))
        except Exception as e:
            if log:
                log("%s not available: %s" % (label, e))
    return out


def fetch_openaip(bbox, api_key, fetch=None, log=None, max_pages=10):
    """Worldwide: openAIP airspaces in bbox. Needs a free openAIP API key."""
    if not api_key:
        raise ValueError("openAIP needs an API key (free account at openaip.net).")
    out = []
    for page in range(1, max_pages + 1):
        q = {"bbox": "%f,%f,%f,%f" % bbox, "limit": 1000, "page": page, "apiKey": api_key}
        obj = _http_json(OPENAIP_URL + "?" + urllib.parse.urlencode(q),
                         {"x-openaip-api-key": api_key, "x-openaip-client-id": api_key}, fetch=fetch)
        out += parse_geojson(obj, "openAIP")
        if page >= int(obj.get("totalPages") or 1):
            break
    if log:
        log("openAIP: %d airspace(s)" % len(out))
    return out


# ------------------------------------------------------------------ the check
@dataclass
class AirspaceHit:
    airspace: Airspace
    status: str                 # inside | below | above | near
    flights: list
    min_dist_m: float
    max_agl_m: float
    max_amsl_m: float
    floor_amsl_m: float         # airspace floor where the mission is (inside/below)
    ring_xy: list = None        # rings in the planning CRS (for maps)

    @property
    def message(self):
        a = self.airspace
        cat = a.category
        fl = ("flights " if len(self.flights) > 1 else "flight ") + ", ".join(str(f) for f in self.flights[:10]) + \
            (" ..." if len(self.flights) > 10 else "")
        if self.status == "inside":
            if a.uas_ceiling_m is not None:
                over = self.max_agl_m > a.uas_ceiling_m + 0.5
                return ("in %s - controlled airspace near an airport: needs authorization (LAANC up to %.0f ft AGL)%s"
                        % (a.name, a.uas_ceiling_m / FT,
                           "; mission flies %.0f ft AGL - above the grid ceiling" % (self.max_agl_m / FT) if over else ""))
            what = {"controlled": "controlled airspace - ATC approval / authorization required",
                    "restricted": "restricted/prohibited - do not enter without the controlling authority's approval",
                    "danger": "danger / military area - check activity times and NOTAMs",
                    "uncontrolled": "uncontrolled airspace - standard drone rules apply",
                    "other": "check what applies"}[cat]
            return "%s enter%s %s %s (%s): %s" % (fl, "" if len(self.flights) > 1 else "s", a.label, a.name,
                                                  a.limits_text, what)
        if self.status == "below":
            return ("%s fl%s under %s %s (floor %s) - %.0f m below it; stay below"
                    % (fl, "y" if len(self.flights) > 1 else "ies", a.label, a.name, a.lower.text(),
                       self.floor_amsl_m - self.max_amsl_m))
        return "%s %s (%s) is %.0f m from the mission" % (a.label, a.name, a.limits_text, self.min_dist_m)


@dataclass
class AirspaceReport:
    hits: List[AirspaceHit]
    sources: list
    checked: int
    near_m: float

    def by_status(self, *st):
        return [h for h in self.hits if h.status in st]

    @property
    def summary(self):
        ins = self.by_status("inside")
        cats = {h.airspace.category for h in ins}
        if "restricted" in cats:
            return "RESTRICTED / PROHIBITED airspace"
        if "controlled" in cats:
            cl = sorted({h.airspace.cls for h in ins if h.airspace.category == "controlled" and h.airspace.cls})
            return "Controlled%s - approval needed" % ((" (Class %s)" % "/".join(cl)) if cl else "")
        if "danger" in cats:
            return "Danger / military area"
        if "uncontrolled" in cats:
            return "Uncontrolled (Class G/F)"
        below = [h for h in self.by_status("below") if h.airspace.category in ("controlled", "restricted", "danger")]
        if below:
            h = min(below, key=lambda q: q.floor_amsl_m - q.max_amsl_m)
            return "Below %s %s (floor %s) - stay below" % (h.airspace.label, h.airspace.name, h.airspace.lower.text())
        if self.by_status("near"):
            return "Clear of the loaded airspace (some nearby)"
        return "Clear of the loaded airspace"

    def text(self):
        out = ["Airspace check (%d airspace(s) from %s):" % (self.checked, ", ".join(self.sources) or "-"),
               "  Result            : %s" % self.summary]
        for h in sorted(self.hits, key=lambda h: ("inside", "below", "above", "near").index(h.status)):
            out.append("  %-6s %s" % (h.status.upper(), h.message))
        out.append("  Not authoritative - confirm with an approved airspace app / NOTAMs before flight.")
        return "\n".join(out)


def check(mission, airspaces, to_wgs84, from_wgs84=None, near_m=1000.0, step_m=25.0, sources=None):
    """Attach mission.airspace (AirspaceReport) and add warnings."""
    from . import geometry as G
    airspaces = [a for a in airspaces if a.rings and a.category != "ignore"]
    ground = getattr(mission, "ground", None)
    site_elev = mission.conditions.site_elevation_m or 0.0
    pts = []                    # (flight, lon, lat, amsl, agl)
    for f in mission.flights:
        z_st = f.z_ref if ground is not None else site_elev
        wps = f.waypoints
        for a, b in zip(wps[:-1], wps[1:]):
            L = math.hypot(b.x - a.x, b.y - a.y)
            n = max(1, int(L / step_m))
            for k in range(n + 1):
                t = k / float(n)
                x, y = a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t
                alt = a.alt + (b.alt - a.alt) * t
                g = z_st
                if ground is not None:
                    try:
                        gv = ground(x, y)
                        g = gv if gv is not None else z_st
                    except Exception:
                        pass
                lon, lat = to_wgs84(x, y)
                pts.append((f.index + 1, lon, lat, z_st + alt, z_st + alt - g, g))
    if not pts:
        mission.airspace = AirspaceReport([], sources or [], len(airspaces), near_m)
        return mission.airspace
    lons, lats = [p[1] for p in pts], [p[2] for p in pts]
    pad = near_m / 111000.0 * 1.5
    box = (min(lons) - pad / max(0.2, math.cos(math.radians(lats[0]))), min(lats) - pad,
           max(lons) + pad / max(0.2, math.cos(math.radians(lats[0]))), max(lats) + pad)
    hits = []
    for a in airspaces:
        b = a.bbox()
        if b[2] < box[0] or b[0] > box[2] or b[3] < box[1] or b[1] > box[3]:
            continue
        inside_fl, below_fl, above_fl = set(), set(), set()
        max_agl = max_amsl = -1e9
        floor = 1e9
        for fl, lon, lat, amsl, agl, g in pts:
            if b[0] <= lon <= b[2] and b[1] <= lat <= b[3] and a.contains(lon, lat):
                lo, hi = a.lower.amsl(g), a.upper.amsl(g)
                if amsl < lo:
                    below_fl.add(fl)
                    floor = min(floor, lo)
                    max_amsl = max(max_amsl, amsl)
                elif amsl > hi:
                    above_fl.add(fl)
                else:
                    inside_fl.add(fl)
                    max_agl, max_amsl = max(max_agl, agl), max(max_amsl, amsl)
        if inside_fl:
            st, fls = "inside", inside_fl
        elif below_fl:
            st, fls = "below", below_fl
        elif above_fl:
            st, fls = "above", above_fl
        else:
            d = min(a.distance_m(lon, lat) for _, lon, lat, _, _, _ in pts[::max(1, len(pts) // 400)])
            if d > near_m:
                continue
            st, fls = "near", set()
        dmin = 0.0 if st != "near" else d
        h = AirspaceHit(a, st, sorted(fls), dmin, max_agl, max_amsl, floor)
        if from_wgs84:
            try:
                h.ring_xy = [[from_wgs84(*p) for p in r] for r in a.rings]
            except Exception:
                h.ring_xy = None
        hits.append(h)
    rep = AirspaceReport(hits, sources or sorted({a.source for a in airspaces if a.source}), len(airspaces), near_m)
    mission.airspace = rep
    w = mission.warnings
    for h in rep.by_status("inside"):
        cat = h.airspace.category
        if cat in ("controlled", "restricted", "danger"):
            w.insert(0, "AIRSPACE: " + h.message)
        elif cat == "other":
            w.append("AIRSPACE: " + h.message)
    for h in rep.by_status("below"):
        if h.airspace.category in ("controlled", "restricted", "danger") and h.floor_amsl_m - h.max_amsl_m < 60:
            w.append("AIRSPACE: " + h.message + " (less than 60 m margin)")
    for h in rep.by_status("near"):
        if h.airspace.category in ("controlled", "restricted"):
            w.append("AIRSPACE: " + h.message + " - stay clear")
    return rep


# ------------------------------------------------------------------ automatic download
US_BOXES = [(-125.0, 24.0, -66.5, 49.5),     # contiguous USA
            (-170.0, 51.0, -129.0, 71.5),    # Alaska
            (-161.0, 18.5, -154.0, 22.5),    # Hawaii
            (-67.5, 17.5, -65.0, 18.6)]      # Puerto Rico


def in_usa(lon, lat):
    return any(a <= lon <= c and b <= lat <= d for a, b, c, d in US_BOXES)


def project_bbox(lonlats, pad_km=10.0):
    lat = sum(p[1] for p in lonlats) / len(lonlats)
    dlat = pad_km / 111.0
    dlon = pad_km / (111.0 * max(0.2, math.cos(math.radians(lat))))
    return (min(p[0] for p in lonlats) - dlon, min(p[1] for p in lonlats) - dlat,
            max(p[0] for p in lonlats) + dlon, max(p[1] for p in lonlats) + dlat)


class AutoAirspace(object):
    """Downloads airspace for the project area automatically - FAA in the USA,
    openAIP elsewhere (free API key) - and re-uses it while the mission stays
    inside the downloaded area. Failures are remembered so live re-planning
    doesn't retry on every change."""

    def __init__(self, fetch=None, log=None):
        self.fetch, self.log = fetch, log or (lambda *_: None)
        self.box = None
        self.airspaces, self.sources = [], []
        self.status = "Not downloaded yet."
        self._failed = {}

    @staticmethod
    def _inside(inner, outer):
        return outer and inner[0] >= outer[0] and inner[1] >= outer[1] and inner[2] <= outer[2] and inner[3] <= outer[3]

    def get(self, lonlats, openaip_key=""):
        """(airspaces, sources) for an area given by WGS84 points; downloads when needed."""
        tight = project_bbox(lonlats, 2.0)
        if self._inside(tight, self.box):
            return self.airspaces, self.sources
        c = (sum(p[0] for p in lonlats) / len(lonlats), sum(p[1] for p in lonlats) / len(lonlats))
        usa = in_usa(*c)
        key = (round(c[0], 2), round(c[1], 2), usa, bool(openaip_key))
        if key in self._failed:
            self.status = self._failed[key]
            return [], []
        box = project_bbox(lonlats, 10.0)
        try:
            if usa:
                asp, src = fetch_faa(box, fetch=self.fetch, log=self.log), ["FAA"]
            elif openaip_key:
                asp, src = fetch_openaip(box, openaip_key, fetch=self.fetch, log=self.log), ["openAIP"]
            else:
                self._failed[key] = ("Outside the USA: enter a free openAIP API key (openaip.net) to download "
                                     "airspace automatically, or use an airspace layer / file.")
                self.status = self._failed[key]
                return [], []
            if not asp and usa:
                raise IOError("no data returned (offline?)")
        except Exception as e:
            self._failed[key] = "Airspace download failed: %s" % e
            self.status = self._failed[key]
            return [], []
        self.box, self.airspaces, self.sources = box, asp, src
        self.status = "%d airspace(s) downloaded from %s for the project area." % (
            len([a for a in asp if a.category != "ignore"]), ", ".join(src))
        return asp, src
