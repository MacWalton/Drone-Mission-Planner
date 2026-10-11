# SPDX-License-Identifier: GPL-2.0-only
# -*- coding: utf-8 -*-
"""Drone Mission Planner - ArcGIS Pro Python toolbox.

Tools
  1. Create Area Mapping Mission   - grid / double grid over a polygon
  2. Create Route Mapping Mission  - linear features: roads, rail, pipelines, powerlines,
                                     rivers; pieces are chained, mapped by chainage (KP)
  Both plan battery changes across one or more launch stations (manual or
  auto-placed), with an optional VLOS range limit and a field schedule.
  3. Flight Time Calculator        - endurance for wind, temperature, altitude, payload
  4. Upload Mission to Drone       - send an exported mission to an aircraft / controller
  5. Download DEM                  - free elevation for an area (AWS Terrain Tiles, or
                                     Copernicus GLO-30 when GDAL is available)
  The mission tools can also download a DEM themselves, check terrain clearance on
  every flight path and (routes) produce an elevation profile by chainage.

Input boundaries can be drawn interactively (pencil button on the input), taken
from any feature layer, or from CAD (DWG/DXF/DGN) feature classes. Requires
ArcGIS Pro 2.9+ (Python 3). Live MAVLink upload needs pymavlink:
  ArcGIS Pro > Settings > Package Manager > clone env > add 'pymavlink'
  or, in a cloned env's Python Command Prompt: python -m pip install pymavlink
"""
import math
import os
import sys

import arcpy

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import dronemapper_core as core
from dronemapper_core import exporters, uploader, photogrammetry as P

WGS84 = arcpy.SpatialReference(4326)
FORMAT_LABELS = {v[0]: k for k, v in exporters.FORMATS.items()}
TRIGGERS = {"Distance - camera fires every N m (autopilots, Pilot 2, Litchi)": "distance",
            "Timed interval - DJI Fly / Mini series": "timed",
            "Stop & shoot - hover at every photo point": "stop"}
TRIGGER_LABELS = {v: k for k, v in TRIGGERS.items()}
WIND_UNITS = {"Knots": "kn", "m/s": "m/s", "km/h": "km/h"}
BASEMAPS = ["Current map basemap", "OpenStreetMap", "Esri World Imagery", "Esri World Topographic", "None"]
SHUTTERS = ["1/500", "1/800", "1/1000", "1/1600", "1/2000", "1/3200"]


# ======================================================================
# helpers
# ======================================================================
def _param(name, label, dtype, ptype="Required", direction="Input", value=None,
           category=None, values=None, multi=False, enabled=True):
    p = arcpy.Parameter(name=name, displayName=label, datatype=dtype, parameterType=ptype,
                        direction=direction, multiValue=multi, category=category)
    if values is not None:
        p.filter.type = "ValueList"
        p.filter.list = values
    if value is not None:
        p.value = value
    p.enabled = enabled
    return p


def _by_name(parameters):
    return {p.name: p for p in parameters}


def _val(params, name, default=None):
    p = params.get(name)
    if p is None or p.value is None or p.valueAsText in (None, ""):
        return default
    return p.value


def _msg(s):
    arcpy.AddMessage(s)


def _warn(s):
    arcpy.AddWarning(s)


def _transformation(src, dst, extent=None):
    try:
        lst = arcpy.ListTransformations(src, dst, extent) if extent else arcpy.ListTransformations(src, dst)
        return lst[0] if lst else ""
    except Exception:
        return ""


def _project(geom, sr):
    tr = _transformation(geom.spatialReference, sr, geom.extent)
    return geom.projectAs(sr, tr) if tr else geom.projectAs(sr)


def _input_sr(fc):
    sr = arcpy.Describe(fc).spatialReference
    if sr is None or sr.name in ("Unknown", "") or sr.factoryCode == 0 and sr.type == "Unknown":
        try:
            sr = arcpy.mp.ArcGISProject("CURRENT").activeMap.spatialReference
            _warn("Input has no coordinate system (common for CAD). Assuming the map's: %s" % sr.name)
        except Exception:
            raise arcpy.ExecuteError("Input has no coordinate system - define one first "
                                     "(Define Projection tool).")
    return sr


def _read_geoms(fc):
    sr = _input_sr(fc)
    geoms = []
    with arcpy.da.SearchCursor(fc, ["SHAPE@"]) as cur:
        for (g,) in cur:
            if g is not None and not g.isMultipart and g.pointCount == 0:
                continue
            if g is not None:
                if g.spatialReference is None or g.spatialReference.name == "Unknown":
                    g = arcpy.FromWKT(g.WKT, sr) if hasattr(arcpy, "FromWKT") else g
                geoms.append(g)
    if not geoms:
        raise arcpy.ExecuteError("No features found in %s." % fc)
    return geoms, sr


def _rings(poly):
    """Polygon geometry -> list of parts, each part = list of rings."""
    parts = []
    for part in poly:
        rings, ring = [], []
        for pt in part:
            if pt is None:
                if ring:
                    rings.append(ring)
                ring = []
            else:
                ring.append((pt.X, pt.Y))
        if ring:
            rings.append(ring)
        if rings:
            parts.append(rings)
    return parts


def _lines(pl):
    return [[(pt.X, pt.Y) for pt in part if pt] for part in pl]


def _closed_lines_to_polygons(geoms, sr):
    polys = []
    for g in geoms:
        for line in _lines(g):
            if len(line) >= 4 and math.hypot(line[0][0] - line[-1][0], line[0][1] - line[-1][1]) < 1e-6 * max(
                    1.0, abs(line[0][0])):
                polys.append(arcpy.Polygon(arcpy.Array([arcpy.Point(*p) for p in line]), sr))
    return polys


def _union(geoms):
    g = geoms[0]
    for h in geoms[1:]:
        g = g.union(h)
    return g


def _utm_sr(geom):
    c = _project(arcpy.PointGeometry(geom.centroid, geom.spatialReference), WGS84).firstPoint
    return arcpy.SpatialReference(core.utm_epsg(c.X, c.Y))


def _point_from_fs(fs, work_sr):
    if not fs:
        return None
    try:
        sr = _input_sr(fs)
        with arcpy.da.SearchCursor(fs, ["SHAPE@"]) as cur:
            for (g,) in cur:
                if g is not None:
                    pg = arcpy.PointGeometry(g.firstPoint, sr)
                    p = _project(pg, work_sr).firstPoint
                    return (p.X, p.Y)
    except Exception as e:
        _warn("Could not read point input: %s" % e)
    return None


def _points_from_fs(fs, work_sr):
    """All points of a feature set, in input order, projected to work_sr."""
    out = []
    if not fs:
        return out
    try:
        sr = _input_sr(fs)
        with arcpy.da.SearchCursor(fs, ["SHAPE@"]) as cur:
            for (g,) in cur:
                if g is not None:
                    p = _project(arcpy.PointGeometry(g.firstPoint, sr), work_sr).firstPoint
                    out.append((p.X, p.Y))
    except Exception as e:
        _warn("Could not read point input: %s" % e)
    return out


def _logistics_params():
    cat = "Battery changes & field schedule"
    return [
        _param("split", "Split into flights per battery", "GPBoolean", value=True, category=cat),
        _param("midline", "Change battery mid-line (resume where it stopped)", "GPBoolean",
               value=True, category=cat),
        _param("sets", "Charged battery sets on site", "GPLong", value=4, category=cat),
        _param("chargers", "Chargers (sets charging at once)", "GPLong", value=2, category=cat),
        _param("charge_min", "Charge time per set (min)", "GPDouble", value=60.0, category=cat),
        _param("swap_min", "Battery swap + pre-flight checks (min)", "GPDouble", value=4.0, category=cat),
        _param("setup_min", "Site setup before first flight (min)", "GPDouble", value=15.0, category=cat),
        _param("drive_kmh", "Travel speed between stations (km/h)", "GPDouble", value=30.0, category=cat),
        _param("reloc_min", "Pack-up / set-up per station move (min)", "GPDouble", value=8.0, category=cat),
        _param("start_time", "Arrive on site (HH:MM)", "GPString", value="08:00", category=cat),
    ]


def _station_params(route):
    cat = "Launch stations & range"
    return [
        _param("max_range", "Max distance from pilot / VLOS (m, blank = no limit)", "GPDouble",
               "Optional", category=cat),
        _param("auto_stations", "Auto-place launch stations (%s)" %
               ("one per route section" if route else "grid sized to the range limit"),
               "GPBoolean", value=False, category=cat),
        _param("station_offset", "Auto stations: offset from the line (m, + = left)", "GPDouble",
               value=0.0, category=cat, enabled=False),
        _param("leapfrog", "Leapfrog - flights may land at the next station (crew moves)",
               "GPBoolean", value=False, category=cat),
    ]


def _logistics_from_params(p):
    hhmm = (p["start_time"].valueAsText or "08:00").strip()
    try:
        h, m = hhmm.split(":")
        start = int(h) * 60 + int(m)
    except Exception:
        start = 8 * 60
    return core.LogisticsParams(
        battery_sets=int(_val(p, "sets", 4)), chargers=int(_val(p, "chargers", 2)),
        charge_min=float(_val(p, "charge_min", 60)), swap_min=float(_val(p, "swap_min", 4)),
        setup_min=float(_val(p, "setup_min", 15)), drive_kmh=float(_val(p, "drive_kmh", 30)),
        relocate_overhead_min=float(_val(p, "reloc_min", 8)), start_time_min=start)


def _to_wgs84_fn(work_sr):
    tr = _transformation(work_sr, WGS84)
    cache = {}

    def f(x, y):
        k = (round(x, 3), round(y, 3))
        if k not in cache:
            pg = arcpy.PointGeometry(arcpy.Point(x, y), work_sr)
            p = (pg.projectAs(WGS84, tr) if tr else pg.projectAs(WGS84)).firstPoint
            cache[k] = (p.X, p.Y)
        return cache[k]
    return f


def _from_wgs84_fn(work_sr):
    tr = _transformation(WGS84, work_sr)

    def f(lon, lat):
        pg = arcpy.PointGeometry(arcpy.Point(lon, lat), WGS84)
        p = (pg.projectAs(work_sr, tr) if tr else pg.projectAs(work_sr)).firstPoint
        return (p.X, p.Y)
    return f


def _current_basemap():
    """(source key, layer name) of the basemap in the active ArcGIS Pro map."""
    try:
        mp = arcpy.mp.ArcGISProject("CURRENT").activeMap
        names = [l.name for l in mp.listLayers()
                 if getattr(l, "isBasemapLayer", False) and getattr(l, "visible", True)]
    except Exception:
        return None, ""
    skip = ("hillshade", "reference", "label", "boundar", "place")
    for n in names:
        if not any(k in n.lower() for k in skip):
            key = core.basemap.source_for_name(n, None)
            if key:
                return key, n
    return None, ", ".join(names)


def _brief_basemap(choice, work_sr):
    """Tile basemap for the field-brief maps, or None."""
    choice = choice or "None"
    if choice == "None":
        return None
    if choice.startswith("Current"):
        key, name = _current_basemap()
        if key:
            _msg("Field-brief basemap: %s (matches '%s' in the active map)." % (core.basemap.SOURCES[key]["name"], name))
        else:
            key = "osm"
            _msg("Field-brief basemap: no matching basemap in the active map%s - using OpenStreetMap."
                 % ((" (%s)" % name) if name else ""))
    else:
        key = core.basemap.source_for_name(choice)
    try:
        cache = os.path.join(arcpy.env.scratchFolder, "dmp_tiles")
    except Exception:
        cache = None
    return core.basemap.TileBasemap(key, _to_wgs84_fn(work_sr), _from_wgs84_fn(work_sr),
                                    cache_dir=cache, log=_warn)


def _dem_sampler(dem, work_sr, extent_pts, buffer_m=500.0):
    """Read the DEM once into memory over the mission extent and sample it."""
    ras = arcpy.Raster(dem)
    rsr = ras.spatialReference
    xs = [p[0] for p in extent_pts]
    ys = [p[1] for p in extent_pts]
    corners = [(min(xs) - buffer_m, min(ys) - buffer_m), (max(xs) + buffer_m, max(ys) + buffer_m),
               (min(xs) - buffer_m, max(ys) + buffer_m), (max(xs) + buffer_m, min(ys) - buffer_m)]
    rc = []
    for x, y in corners:
        p = _project(arcpy.PointGeometry(arcpy.Point(x, y), work_sr), rsr).firstPoint
        rc.append((p.X, p.Y))
    cw, ch = ras.meanCellWidth, ras.meanCellHeight
    xmin = max(min(p[0] for p in rc), ras.extent.XMin)
    ymin = max(min(p[1] for p in rc), ras.extent.YMin)
    xmax = min(max(p[0] for p in rc), ras.extent.XMax)
    ymax = min(max(p[1] for p in rc), ras.extent.YMax)
    if xmax <= xmin or ymax <= ymin:
        raise arcpy.ExecuteError("The DEM does not cover the mission area.")
    ncols = int(math.ceil((xmax - xmin) / cw))
    nrows = int(math.ceil((ymax - ymin) / ch))
    arr = arcpy.RasterToNumPyArray(ras, arcpy.Point(xmin, ymin), ncols, nrows, nodata_to_value=-99999)
    top = ymin + nrows * ch
    tr = _transformation(work_sr, rsr)

    def f(x, y):
        pg = arcpy.PointGeometry(arcpy.Point(x, y), work_sr)
        p = (pg.projectAs(rsr, tr) if tr else pg.projectAs(rsr)).firstPoint
        c = int((p.X - xmin) / cw)
        r = int((top - p.Y) / ch)
        if not (0 <= r < nrows and 0 <= c < ncols):
            raise ValueError("outside DEM")
        v = float(arr[r, c])
        if v <= -99998:
            raise ValueError("nodata")
        return v
    return f


DEM_SOURCES = ["AWS Terrain Tiles", "Copernicus GLO-30 (needs GDAL)"]


def _download_dem(source, lonlat_pts, buffer_m, out_path):
    """Download a DEM for the points' area into out_path (GeoTIFF or GDB raster)."""
    bbox = core.dem.bbox_lonlat(lonlat_pts, buffer_m)
    arcpy.SetProgressor("step", "Downloading elevation data...", 0, 100, 1)

    def prog(done, total):
        arcpy.SetProgressorPosition(int(100 * done / max(total, 1)))
    try:
        if source.startswith("Copernicus"):
            tif = out_path if out_path.lower().endswith(".tif") else \
                os.path.join(arcpy.env.scratchFolder, "dmp_copernicus.tif")
            info = core.dem.fetch_copernicus(bbox, tif, prog)
            if tif != out_path:
                arcpy.management.CopyRaster(tif, out_path)
        else:
            grid, gt, epsg, info = core.dem.fetch_terrarium(bbox, progress=prog)
            arr, gt2 = core.dem.crop_grid(grid, gt, core.dem.webmerc_bbox(bbox))
            ll = arcpy.Point(gt2[0], gt2[3] + gt2[5] * arr.shape[0])
            ras = arcpy.NumPyArrayToRaster(arr, ll, gt2[1], -gt2[5])
            ras.save(out_path)
            arcpy.management.DefineProjection(out_path, arcpy.SpatialReference(3857))
    except core.dem.DEMError as e:
        raise arcpy.ExecuteError("%s  (Check the internet connection / proxy, or try the other "
                                 "source.)" % e)
    finally:
        arcpy.ResetProgressor()
    _msg("DEM saved: %s" % out_path)
    if info.get("missing_tiles"):
        _warn("%d tile(s) had no data (sea or not released) and were set to 0 m." % info["missing_tiles"])
    _msg("Elevation data: " + info["attribution"])
    return out_path


def _read_obstacles(p, work_sr, near_pts, pad_m):
    """Obstacles from the obstacle layer, in the planning CRS, near the mission."""
    fs = p["obstacles"].valueAsText if p["obstacles"].value is not None else None
    if not fs:
        return []
    sr = _input_sr(fs)
    field = p["obstacle_field"].valueAsText if p["obstacle_field"].value is not None else None
    default_h = _hm(p, "obstacle_default", 25)
    xs, ys = [q[0] for q in near_pts], [q[1] for q in near_pts]
    box = (min(xs) - pad_m, min(ys) - pad_m, max(xs) + pad_m, max(ys) + pad_m)
    out, missing = [], 0
    cols = ["SHAPE@"] + ([field] if field else [])
    with arcpy.da.SearchCursor(fs, cols) as cur:
        for row in cur:
            g = row[0]
            if g is None:
                continue
            h = None
            if field:
                try:
                    h = core.units.to_m(float(row[1]), _hunit(p)) if row[1] not in (None, "") else None
                except (TypeError, ValueError):
                    h = None
                if h is None:
                    missing += 1
            h = h if h is not None else default_h
            if not h or h <= 0:
                continue
            if g.spatialReference is None or getattr(g.spatialReference, "name", "") in ("", "Unknown"):
                g = arcpy.PointGeometry(g.firstPoint, sr) if isinstance(g, arcpy.PointGeometry) else g
            gw = _project(g, work_sr)
            e = gw.extent
            if e.XMax < box[0] or e.XMin > box[2] or e.YMax < box[1] or e.YMin > box[3]:
                continue
            if isinstance(gw, arcpy.PointGeometry):
                for part in gw:
                    for q in part:
                        if q:
                            out.append(core.Obstacle("point", [(q.X, q.Y)], h))
            elif isinstance(gw, arcpy.Polyline):
                for line in _lines(gw):
                    out.append(core.Obstacle("line", line, h))
            else:
                for rings in _rings(gw):
                    out.append(core.Obstacle("polygon", rings, h))
    msg = "Obstacles: %d feature(s) near the mission" % len(out)
    if missing:
        msg += "; %d had no height and used the default" % missing
    _msg(msg + ".")
    return out


def _new_fc(gdb, name, shape, sr, fields):
    name = arcpy.ValidateTableName(name, gdb)
    path = os.path.join(gdb, name)
    if arcpy.Exists(path):
        arcpy.management.Delete(path)
    arcpy.management.CreateFeatureclass(gdb, name, shape, spatial_reference=sr)
    for fname, ftype in fields:
        arcpy.management.AddField(path, fname, ftype)
    return path


def _drone_from_params(params):
    d = core.get_drone(params["aircraft"].valueAsText)
    c = d.camera
    ov = {"sensor_w": ("camera", "sensor_width_mm"), "sensor_h": ("camera", "sensor_height_mm"),
          "focal": ("camera", "focal_length_mm"), "img_w": ("camera", "image_width_px"),
          "img_h": ("camera", "image_height_px"), "interval": ("camera", "min_trigger_interval_s"),
          "mass": ("drone", "empty_mass_kg"), "max_payload": ("drone", "max_payload_kg"),
          "rated_time": ("drone", "rated_flight_time_min"), "max_speed": ("drone", "max_speed_ms"),
          "max_wind": ("drone", "max_wind_ms"), "battery_wh": ("drone", "battery_wh")}
    for pname, (obj, attr) in ov.items():
        v = _val(params, pname)
        if v is not None and pname == "max_wind":
            v = core.units.wind_to_ms(v, _wunit(params))
        if v is not None:
            setattr(c if obj == "camera" else d, attr,
                    int(v) if attr.endswith("_px") else float(v))
    e = _val(params, "dji_enum")
    if e:
        d.drone_enum = str(e)
    return d


def _hunit(params):
    """'m' or 'ft' from the Height units parameter (absent = metres)."""
    p = params.get("height_units")
    return "ft" if (p is not None and (p.valueAsText or "").startswith("Feet")) else "m"


def _hm(params, name, default=None):
    """A height parameter in metres, whatever unit it was entered in."""
    v = _val(params, name, default)
    return None if v is None else core.units.to_m(float(v), _hunit(params))


def _wunit(params):
    """'kn', 'm/s' or 'km/h' from the Wind speed unit parameter (absent = knots)."""
    p = params.get("wind_unit")
    return WIND_UNITS.get(p.valueAsText if p is not None else "Knots", "kn")


def _convert_wind_on_switch(tool, p):
    """When the wind unit is changed, convert the wind numbers already entered."""
    try:
        u = _wunit(p)
        prev = getattr(tool, "_last_wunit", None)
        if prev and prev != u and p["wind_unit"].altered:
            for n in ("wind", "max_wind"):
                if n in p and p[n].value is not None:
                    p[n].value = round(core.units.wind_from_ms(core.units.wind_to_ms(float(p[n].value), prev), u), 1)
        tool._last_wunit = u
    except Exception:
        pass


def _conditions_from_params(params):
    return core.Conditions(
        wind_speed_ms=core.units.wind_to_ms(_val(params, "wind", 0), _wunit(params)),
        wind_unit=_wunit(params), wind_from_deg=float(_val(params, "wind_dir", 0)),
        temperature_c=float(_val(params, "temp", 15)), site_elevation_m=_hm(params, "elev", 0),
        payload_kg=_payload_from_params(params)[1], payload_items=_payload_from_params(params)[0],
        reserve_pct=float(_val(params, "reserve", 25)),
        battery_health_pct=float(_val(params, "health", 100)))


def _aircraft_params():
    cat = "Aircraft overrides (leave blank for profile values)"
    return [
        _param("aircraft", "Aircraft", "GPString", values=core.drone_names(),
               value=core.drone_names()[0]),
        _param("sensor_w", "Sensor width (mm)", "GPDouble", "Optional", category=cat),
        _param("sensor_h", "Sensor height (mm)", "GPDouble", "Optional", category=cat),
        _param("focal", "Focal length (mm)", "GPDouble", "Optional", category=cat),
        _param("img_w", "Image width (px)", "GPLong", "Optional", category=cat),
        _param("img_h", "Image height (px)", "GPLong", "Optional", category=cat),
        _param("interval", "Minimum photo interval (s)", "GPDouble", "Optional", category=cat),
        _param("mass", "Take-off mass without payload (kg)", "GPDouble", "Optional", category=cat),
        _param("max_payload", "Maximum payload (kg)", "GPDouble", "Optional", category=cat),
        _param("rated_time", "Rated flight time (min)", "GPDouble", "Optional", category=cat),
        _param("max_speed", "Maximum speed (m/s)", "GPDouble", "Optional", category=cat),
        _param("max_wind", "Maximum wind (wind speed unit)", "GPDouble", "Optional", category=cat),
        _param("battery_wh", "Battery capacity per flight (Wh, all packs)", "GPDouble", "Optional",
               category=cat),
        _param("dji_enum", "DJI WPML enum 'drone,payload' (e.g. 77,66)", "GPString", "Optional",
               category=cat),
    ]


def _payload_table(cat):
    p = arcpy.Parameter(name="payload_items", displayName="Payload carried: item, weight, power drawn "
                        "from the flight battery (W, 0 if self-powered)", datatype="GPValueTable",
                        parameterType="Optional", direction="Input", category=cat)
    p.columns = [["GPString", "Item"], ["GPDouble", "Weight"], ["GPDouble", "Power (W)"]]
    return p


def _payload_from_params(params):
    """(items in kg, other payload kg) from the value table."""
    unit = (params["payload_unit"].valueAsText or "kg") if "payload_unit" in params else "kg"
    items = []
    p = params.get("payload_items")
    rows = (p.values or []) if p is not None and p.value is not None else []
    for r in rows:
        if not r:
            continue
        name = str(r[0] or "Item").strip()
        try:
            w = float(r[1] or 0) if len(r) > 1 else 0.0
            pw = float(r[2] or 0) if len(r) > 2 else 0.0
        except (TypeError, ValueError):
            raise arcpy.ExecuteError("Payload item '%s': weight and power must be numbers." % name)
        items.append(core.PayloadItem(name, core.units.kg(max(w, 0.0), unit), max(pw, 0.0)))
    other = core.units.kg(float(_val(params, "payload", 0) or 0), unit)
    return items, other


def _payload_messages(p):
    """Show the payload total in the dialog; warn when over the aircraft limit."""
    if "payload_items" not in p:
        return
    try:
        items, other = _payload_from_params(p)
    except Exception as e:
        p["payload_items"].setErrorMessage(str(e))
        return
    total = other + sum(i.mass_kg for i in items)
    watts = sum(i.power_w for i in items)
    try:
        limit = float(_val(p, "max_payload")) if _val(p, "max_payload") is not None else \
            core.get_drone(p["aircraft"].valueAsText).max_payload_kg
    except Exception:
        limit = None
    unit = p["payload_unit"].valueAsText or "kg"
    msg = "Total payload %s, accessory power %.0f W." % (core.units.mass_fmt(total, unit), watts)
    if limit is not None and total > limit + 1e-9:
        p["payload_items"].setWarningMessage(msg + " Over this aircraft's %s payload limit."
                                             % core.units.mass_fmt(limit, unit))


def _dji_template_param(cat):
    t = _param("dji_template", "DJI Fly / Pilot 2 mission made on the controller (.kmz) - copies its "
               "aircraft codes (needed for DJI Fly, e.g. Mini 5 Pro)", "DEFile", "Optional", category=cat)
    t.filter.list = ["kmz"]
    return t


def _conditions_params():
    cat = "Conditions (flight-time optimisation)"
    return [
        _param("wind_unit", "Wind speed unit", "GPString", values=list(WIND_UNITS), value="Knots",
               category=cat),
        _param("wind", "Wind speed (wind speed unit)", "GPDouble", value=0.0, category=cat),
        _param("wind_dir", "Wind direction - blowing FROM (deg)", "GPDouble", value=0.0, category=cat),
        _param("temp", "Air temperature (C)", "GPDouble", value=20.0, category=cat),
        _param("elev", "Take-off elevation above sea level (height units)", "GPDouble", value=0.0,
               category=cat),
        _payload_table(cat),
        _param("payload_unit", "Payload weight unit", "GPString", values=["kg", "lb"], value="kg",
               category=cat),
        _param("payload", "Other payload not listed (weight unit)", "GPDouble", value=0.0, category=cat),
        _param("reserve", "Battery reserve (%)", "GPDouble", value=25.0, category=cat),
        _param("health", "Battery health (%)", "GPDouble", value=100.0, category=cat),
    ]


def _update_aircraft_defaults(params):
    p = params["aircraft"]
    if p.altered and not p.hasBeenValidated and p.valueAsText in core.DRONES:
        d = core.get_drone(p.valueAsText)
        c = d.camera
        for name, v in (("sensor_w", c.sensor_width_mm), ("sensor_h", c.sensor_height_mm),
                        ("focal", c.focal_length_mm), ("img_w", c.image_width_px),
                        ("img_h", c.image_height_px), ("interval", c.min_trigger_interval_s),
                        ("mass", d.empty_mass_kg), ("max_payload", d.max_payload_kg),
                        ("rated_time", d.rated_flight_time_min), ("max_speed", d.max_speed_ms),
                        ("max_wind", round(core.units.wind_from_ms(d.max_wind_ms, _wunit(params)), 1)),
                        ("battery_wh", d.battery_wh),
                        ("dji_enum", d.drone_enum or None)):
            if name in params:
                params[name].value = v
        if "photo_trigger" in params:
            params["photo_trigger"].value = TRIGGER_LABELS.get(d.photo_trigger, TRIGGER_LABELS["distance"])


# ======================================================================
# mission tools
# ======================================================================
class _MissionTool(object):
    route = False

    def __init__(self):
        self.canRunInBackground = False

    def getParameterInfo(self):
        r = self.route
        fl = ["Polyline"] if r else ["Polygon", "Polyline"]
        inp = _param("in_features",
                     "Route centreline (draw, layer or CAD)" if r else
                     "Mission boundary (draw, layer or CAD)", "GPFeatureRecordSetLayer")
        inp.filter.list = fl
        tk = _param("takeoff", "Take-off & launch stations (first point = take-off S1; add more "
                    "points for battery changes)", "GPFeatureRecordSetLayer", "Optional")
        tk.filter.list = ["Point"]
        ld = _param("landing", "Final landing point (blank = last flight's launch station)",
                    "GPFeatureRecordSetLayer", "Optional")
        ld.filter.list = ["Point"]
        nm = _param("mission_name", "Mission name (blank = automatic)", "GPString", "Optional")
        ps = [nm, inp, tk, ld] + _aircraft_params()
        cat = "Flight parameters"
        ps += [
            _param("height_units", "Height units (all heights & elevations; files are always "
                   "written in metres)", "GPString", values=["Metres", "Feet"], value="Metres",
                   category=cat),
            _param("size_mode", "Set flight by", "GPString",
                   values=["Flying height", "Ground sample distance (cm/px)"],
                   value="Ground sample distance (cm/px)", category=cat),
            _param("height", "Flying height above take-off/ground (height units)", "GPDouble", value=100.0,
                   category=cat),
            _param("rth_height", "Return-to-take-off height (height units, blank = flying height)",
                   "GPDouble", "Optional", category=cat),
            _param("batt_height", "Battery-change return / back-out height (height units, blank = "
                   "return height)", "GPDouble", "Optional", category=cat),
            _param("min_agl", "Minimum height above ground - trees, powerlines, buildings (height units, "
                   "blank = off)", "GPDouble", "Optional", category=cat),
            _param("gsd", "Ground sample distance (cm/px)", "GPDouble", value=2.5, category=cat),
            _param("front", "Front overlap (%)", "GPDouble", value=75.0 if not r else 80.0, category=cat),
            _param("side", "Side overlap (%)", "GPDouble", value=65.0 if not r else 70.0, category=cat),
        ]
        if r:
            lc = "Linear feature & corridor"
            ps += [_param("left_width", "Width left of the line (m, direction of KP)", "GPDouble",
                          value=30.0, category=lc),
                   _param("right_width", "Width right of the line (m)", "GPDouble", value=30.0,
                          category=lc),
                   _param("corridor_lines", "Number of lines (blank = automatic)", "GPLong",
                          "Optional", category=lc),
                   _param("kp_from", "Map from chainage KP (km, blank = start)", "GPDouble",
                          "Optional", category=lc),
                   _param("kp_to", "Map to chainage KP (km, blank = end)", "GPDouble", "Optional",
                          category=lc),
                   _param("reverse_kp", "Reverse KP direction", "GPBoolean", value=False, category=lc),
                   _param("simplify", "Simplify line (m)", "GPDouble", value=2.0, category=lc),
                   _param("sections", "Sections", "GPString",
                          values=["Auto (battery & range)", "Fixed length", "None - full length"],
                          value="Auto (battery & range)", category=lc),
                   _param("section_km", "Fixed section length (km)", "GPDouble", value=2.0,
                          category=lc, enabled=False),
                   _param("kp_interval", "Chainage markers every (m, 0 = none)", "GPDouble",
                          value=100.0, category=lc)]
        else:
            ps += [_param("heading_mode", "Line heading", "GPString",
                          values=["Automatic (fewest lines)", "Manual", "Align with wind"],
                          value="Automatic (fewest lines)", category=cat),
                   _param("heading", "Manual heading (deg)", "GPDouble", value=0.0, category=cat,
                          enabled=False),
                   _param("crosshatch", "Double grid (crosshatch) for 3D models", "GPBoolean",
                          value=False, category=cat),
                   _param("margin", "Extend coverage beyond boundary (m)", "GPDouble", value=0.0,
                          category=cat)]
        ps += [
            _param("overshoot", "Line run-in / run-out (m)", "GPDouble", value=10.0, category=cat),
            _param("speed", "Speed (m/s, blank = optimise)", "GPDouble", "Optional", category=cat),
            _param("pitch", "Gimbal pitch (deg)", "GPDouble", value=-90.0, category=cat),
            _param("shutter", "Shutter speed (motion-blur limit)", "GPString", values=SHUTTERS,
                   value="1/1000", category=cat),
            _param("portrait", "Camera portrait (short side across track)", "GPBoolean", value=False,
                   category=cat),
            _param("photo_trigger", "Photo trigger", "GPString", values=list(TRIGGERS),
                   value=TRIGGER_LABELS["distance"], category=cat),
            _param("stop_hover", "Stop & shoot: hover at each photo (s)", "GPDouble", value=1.0,
                   category=cat, enabled=False),
        ]
        ps += _station_params(r) + _logistics_params()
        ps += _conditions_params()
        cat = "Terrain (DEM)"
        ps += [_param("dem", "Elevation model (DEM)", "GPRasterLayer", "Optional", category=cat),
               _param("auto_dem", "If no DEM is given, download one", "GPString", category=cat,
                      values=["No"] + DEM_SOURCES, value="No"),
               _param("terrain", "Terrain follow (constant height above ground)", "GPBoolean",
                      value=False, category=cat),
               _param("tcheck", "Check terrain clearance on every flight path", "GPBoolean",
                      value=True, category=cat),
               _param("tclear", "Minimum clearance above ground (height units)", "GPDouble", value=30.0,
                      category=cat),
               _param("tagl", "Maximum height above ground (height units)", "GPDouble", value=120.0,
                      category=cat)]
        oc = "Obstacles (powerlines, buildings, trees)"
        ol = _param("obstacles", "Obstacle layer (points, lines or polygons)", "GPFeatureLayer", "Optional",
                    category=oc)
        of = _param("obstacle_field", "Height field (height units; blank = default height)", "Field",
                    "Optional", category=oc)
        of.parameterDependencies = ["obstacles"]
        of.filter.list = ["Short", "Long", "Float", "Double"]
        ps += [ol, of,
               _param("obstacle_default", "Default obstacle height (height units)", "GPDouble", value=25.0,
                      category=oc),
               _param("obstacle_margin", "Vertical margin above obstacle tops (height units)", "GPDouble",
                      value=10.0, category=oc),
               _param("obstacle_buffer", "Horizontal buffer around obstacles (height units)", "GPDouble",
                      value=15.0, category=oc)]
        cat = "Output"
        ps += [_param("out_crs", "Coordinate system (blank = automatic UTM)", "GPCoordinateSystem",
                      "Optional", category=cat),
               _param("out_gdb", "Output geodatabase", "DEWorkspace", category=cat),
               _param("out_name", "Output name prefix", "GPString",
                      value="route_mission" if r else "area_mission", category=cat)]
        cat = "Export & upload"
        fm = _param("formats", "Export formats", "GPString", "Optional", category=cat,
                    values=[v[0] for v in exporters.FORMATS.values()], multi=True)
        ps += [_param("export_folder", "Export folder", "DEFolder", "Optional", category=cat), fm,
               _param("firmware", "QGroundControl firmware", "GPString", values=["PX4", "ArduPilot"],
                      value="PX4", category=cat),
               _dji_template_param(cat),
               _param("brief_basemap", "Field-brief map background", "GPString", values=BASEMAPS,
                      value=BASEMAPS[0], category=cat),
               _param("mavlink", "Upload now over MAVLink - connection (e.g. COM5, udpin:0.0.0.0:14550)",
                      "GPString", "Optional", category=cat),
               _param("baud", "Serial baud rate", "GPLong", value=57600, category=cat),
               _param("upload_flight", "Flight number to upload", "GPLong", value=1, category=cat),
               _param("device_folder", "Copy files to controller / SD card folder", "DEFolder",
                      "Optional", category=cat)]
        for n, label, dtype in (("out_boundary", "Boundary", "DEFeatureClass"),
                                ("out_legs", "Survey legs", "DEFeatureClass"),
                                ("out_routes", "Flight routes", "DEFeatureClass"),
                                ("out_waypoints", "Waypoints", "DEFeatureClass"),
                                ("out_photos", "Photo centres", "DEFeatureClass"),
                                ("out_footprints", "Image footprints", "DEFeatureClass"),
                                ("out_site", "Launch stations and landing", "DEFeatureClass"),
                                ("out_chainage", "Chainage markers", "DEFeatureClass"),
                                ("out_schedule", "Flight & battery schedule", "DETable"),
                                ("out_terrain", "Terrain warnings", "DEFeatureClass"),
                                ("out_profile", "Route elevation profile", "DETable"),
                                ("out_dem", "Downloaded DEM", "DERasterDataset")):
            ps.append(_param(n, label, dtype, "Derived", "Output"))
        try:
            ps_gdb = _by_name(ps)["out_gdb"]
            ps_gdb.value = arcpy.env.workspace or arcpy.env.scratchGDB
        except Exception:
            pass
        return ps

    def isLicensed(self):
        return True

    def updateParameters(self, parameters):
        p = _by_name(parameters)
        _update_aircraft_defaults(p)
        # mission name -> output name prefix (unless the user typed their own prefix)
        if p["mission_name"].altered and p["mission_name"].value and not p["out_name"].altered:
            p["out_name"].value = core.safe_filename(p["mission_name"].valueAsText)
        # switching units converts the numbers already entered
        try:
            u = _hunit(p)
            prev = getattr(self, "_last_unit", None)
            if prev and prev != u and p["height_units"].altered:
                for n in ("height", "rth_height", "batt_height", "elev", "tclear", "tagl", "min_agl",
                          "obstacle_default", "obstacle_margin", "obstacle_buffer"):
                    if n in p and p[n].value is not None:
                        p[n].value = round(core.units.from_m(core.units.to_m(float(p[n].value), prev), u), 1)
            self._last_unit = u
        except Exception:
            pass
        _convert_wind_on_switch(self, p)
        by_gsd = p["size_mode"].valueAsText.startswith("Ground")
        p["gsd"].enabled, p["height"].enabled = by_gsd, not by_gsd
        try:
            cam = _drone_from_params(p).camera
            if by_gsd and p["gsd"].value:
                p["height"].value = round(core.units.from_m(
                    P.height_from_gsd(cam, float(p["gsd"].value)), _hunit(p)), 1)
            elif not by_gsd and p["height"].value:
                p["gsd"].value = round(P.gsd_from_height(cam, _hm(p, "height")), 2)
        except Exception:
            pass
        if "heading_mode" in p:
            p["heading"].enabled = p["heading_mode"].valueAsText == "Manual"
        has_dem = p["dem"].value is not None or (p["auto_dem"].valueAsText or "No") != "No"
        p["terrain"].enabled = has_dem
        p["auto_dem"].enabled = p["dem"].value is None
        p["tclear"].enabled = p["tagl"].enabled = bool(p["tcheck"].value) and has_dem
        p["station_offset"].enabled = bool(p["auto_stations"].value) and self.route
        if "sections" in p:
            p["section_km"].enabled = p["sections"].valueAsText == "Fixed length"
        p["stop_hover"].enabled = TRIGGERS.get(p["photo_trigger"].valueAsText) == "stop"
        up = p["mavlink"].value is not None
        p["baud"].enabled = p["upload_flight"].enabled = up or p["device_folder"].value is not None
        return

    def updateMessages(self, parameters):
        p = _by_name(parameters)
        h = _hm(p, "height") if p["height"].value else None
        if h and float(h) > 120:
            p["height"].setWarningMessage("Above 120 m (400 ft) AGL needs an approval in most "
                                          "jurisdictions (CASA, FAA, EASA).")
        oc = p["out_crs"].value
        if oc is not None and getattr(oc, "type", "") == "Geographic":
            p["out_crs"].setErrorMessage("Choose a projected coordinate system (metres).")
        for n in ("front", "side"):
            v = p[n].value
            if v is not None and not 10 <= float(v) <= 95:
                p[n].setErrorMessage("Overlap must be between 10 and 95 %.")
        if p["device_folder"].value is not None and not p["formats"].value:
            p["formats"].setWarningMessage("Choose at least one format to copy to the device "
                                           "(DJI WPML for Pilot 2, Litchi CSV, KML for Autel).")
        if p["auto_stations"].value and not self.route and not p["max_range"].value:
            p["auto_stations"].setWarningMessage("No range limit set - stations will be placed for "
                                                 "a 1000 m range.")
        _payload_messages(p)
        try:
            d = core.get_drone(p["aircraft"].valueAsText)
            if d.platform == "dji_fly" or d.needs_dji_template:
                p["aircraft"].setWarningMessage(d.notes or "No SDK - routes load in DJI Fly only.")
                if not p["dji_template"].value and "dji_kmz" in [FORMAT_LABELS.get(v, "") for v in (p["formats"].values or [])]:
                    p["dji_template"].setWarningMessage("Create and save any waypoint route on this aircraft "
                                                        "(DJI Fly or Pilot 2), copy its .kmz here so the "
                                                        "export carries the right aircraft codes.")
        except Exception:
            pass
        if p["mavlink"].value is not None and not uploader.pymavlink_available():
            p["mavlink"].setErrorMessage("pymavlink is not installed in this Python environment. "
                                         "Clone the ArcGIS Pro environment and add pymavlink.")
        return

    def execute(self, parameters, messages):
        p = _by_name(parameters)
        arcpy.env.overwriteOutput = True
        fs = p["in_features"].valueAsText
        geoms, in_sr = _read_geoms(fs)
        shape = arcpy.Describe(fs).shapeType

        # ---- boundary geometry
        if self.route:
            g = geoms[0]
        else:
            if shape == "Polyline":
                polys = _closed_lines_to_polygons(geoms, in_sr)
                if not polys:
                    raise arcpy.ExecuteError("No closed polylines found to build a boundary from.")
                _msg("Built %d polygon(s) from closed CAD polylines." % len(polys))
                geoms = polys
            g = _union(geoms) if len(geoms) > 1 else geoms[0]

        oc = p["out_crs"].value
        work_sr = oc if (oc is not None and getattr(oc, "type", "") == "Projected") else _utm_sr(g)
        _msg("Planning coordinate system: %s" % work_sr.name)
        if self.route:
            parts = [l for gg in geoms for l in _lines(_project(gg, work_sr))]
            gdata, gap = core.geometry.chain_lines(parts)
            if len(parts) > 1:
                _msg("Joined %d line pieces into one route (%.3f km)."
                     % (len(parts), core.geometry.line_length(gdata) / 1000))
                if gap > 1.0:
                    _warn("Largest gap bridged between pieces: %.0f m - check the route there." % gap)
            verts = gdata
        else:
            parts = _rings(_project(g, work_sr))
            if len(parts) > 1:
                _warn("%d separate polygons - using the largest. Plan the others separately." % len(parts))
            gdata = max(parts, key=lambda rr: abs(core.geometry.ring_area(rr[0])))
            verts = gdata[0]

        site_pts = _points_from_fs(p["takeoff"].valueAsText, work_sr)
        tk = site_pts[0] if site_pts else None
        extra = site_pts[1:]
        ld = _point_from_fs(p["landing"].valueAsText, work_sr)

        drone = _drone_from_params(p)
        cond = _conditions_from_params(p)
        max_range = float(_val(p, "max_range")) if _val(p, "max_range") else None
        by_gsd = p["size_mode"].valueAsText.startswith("Ground")
        sh = p["shutter"].valueAsText
        mp = core.MissionParams(
            mission_type=core.ROUTE if self.route else core.AREA,
            height_m=_hm(p, "height", 100), gsd_cm=float(_val(p, "gsd")) if by_gsd else None,
            rth_height_m=_hm(p, "rth_height"), battery_height_m=_hm(p, "batt_height"),
            min_agl_m=_hm(p, "min_agl"),
            obstacle_margin_m=_hm(p, "obstacle_margin", 10), obstacle_buffer_m=_hm(p, "obstacle_buffer", 15),
            height_unit=_hunit(p), mass_unit=(p["payload_unit"].valueAsText or "kg"),
            mission_name=(p["mission_name"].valueAsText or "").strip() or core.default_mission_name(
                core.ROUTE if self.route else core.AREA,
                "" if fs.startswith("in_memory") or "FeatureSet" in fs else os.path.splitext(os.path.basename(fs))[0]),
            front_overlap_pct=float(_val(p, "front", 75)), side_overlap_pct=float(_val(p, "side", 65)),
            overshoot_m=float(_val(p, "overshoot", 10)),
            speed_ms=float(_val(p, "speed")) if _val(p, "speed") else None,
            gimbal_pitch_deg=float(_val(p, "pitch", -90)), shutter_s=1.0 / float(sh.split("/")[1]),
            portrait=bool(_val(p, "portrait", False)), split_batteries=bool(_val(p, "split", True)),
            terrain_follow=bool(_val(p, "terrain", False)),
            split_midline=bool(_val(p, "midline", True)), max_range_m=max_range,
            leapfrog=bool(_val(p, "leapfrog", False)),
            photo_trigger=TRIGGERS.get(p["photo_trigger"].valueAsText, "distance"),
            stop_hover_s=float(_val(p, "stop_hover", 1.0)))
        if self.route:
            mp.left_width_m = float(_val(p, "left_width", 30))
            mp.right_width_m = float(_val(p, "right_width", 30))
            mp.corridor_lines = int(_val(p, "corridor_lines")) if _val(p, "corridor_lines") else None
            mp.chainage_start_m = float(_val(p, "kp_from")) * 1000 if _val(p, "kp_from") else None
            mp.chainage_end_m = float(_val(p, "kp_to")) * 1000 if _val(p, "kp_to") else None
            mp.reverse_route = bool(_val(p, "reverse_kp", False))
            mp.simplify_m = float(_val(p, "simplify", 0))
            sec = p["sections"].valueAsText
            mp.section_length_m = (float(_val(p, "section_km", 2)) * 1000 if sec == "Fixed length"
                                   else 0 if sec.startswith("None") else None)
        else:
            hm = p["heading_mode"].valueAsText
            mp.heading_mode = "manual" if hm == "Manual" else "wind" if hm.startswith("Align") else "auto"
            mp.heading_deg = float(_val(p, "heading", 0))
            mp.crosshatch = bool(_val(p, "crosshatch", False))
            mp.margin_m = float(_val(p, "margin", 0))

        # ---- launch stations
        if _val(p, "auto_stations", False):
            if self.route:
                auto = core.suggest_stations(mp, drone, cond, gdata, float(_val(p, "station_offset", 0)))
            else:
                if not mp.max_range_m:
                    mp.max_range_m = 1000.0
                    _warn("Range limit set to 1000 m to size the station grid.")
                auto = core.suggest_area_stations(gdata, mp.max_range_m)
            _msg("Auto-placed %d launch stations (edit the _site output and use it as the "
                 "launch-station input to move them to access points)." % len(auto))
            tk, extra = auto[0], auto[1:]
        if tk is None:
            tk = min(verts, key=lambda v: v[1])
            _msg("No take-off point given - using the southernmost boundary vertex.")

        dem = None
        dem_path = p["dem"].valueAsText if p["dem"].value is not None else None
        site_pts = verts + [tk] + extra + ([ld] if ld else [])
        auto_src = p["auto_dem"].valueAsText or "No"
        if dem_path is None and auto_src != "No":
            gdb = p["out_gdb"].valueAsText
            prefix = p["out_name"].valueAsText or "mission"
            to84_ = _to_wgs84_fn(work_sr)
            dem_path = _download_dem(auto_src, [to84_(*q) for q in site_pts],
                                     max(1500.0, mp.max_range_m or 0.0),
                                     os.path.join(gdb, arcpy.ValidateTableName(prefix + "_dem", gdb))
                                     if gdb.lower().endswith(".gdb") else
                                     os.path.join(gdb, prefix + "_dem.tif"))
            p["out_dem"].value = dem_path
        if dem_path:
            dem = _dem_sampler(dem_path, work_sr, site_pts, buffer_m=1500.0)
        elif mp.terrain_follow:
            _warn("Terrain follow needs a DEM - planning at constant height.")

        try:
            obstacles = _read_obstacles(p, work_sr, site_pts, mp.max_range_m or 1500.0)
            mission = core.plan_mission(mp, drone, cond, gdata, tk, ld, ground_elevation=dem,
                                        stations=extra, logistics=_logistics_from_params(p),
                                        obstacles=obstacles)
        except ValueError as e:
            raise arcpy.ExecuteError(str(e))
        if (dem is not None or mission.obstacles) and _val(p, "tcheck", True):
            limit = (_hm(p, "min_agl") - 0.5) if _hm(p, "min_agl") else _hm(p, "tclear", 30)
            core.terrain.analyse(mission, dem, limit, _hm(p, "tagl", 120))
        for line in mission.report().splitlines():
            (_warn if line.startswith("WARNING") else _msg)(line)

        # ---- outputs
        gdb = p["out_gdb"].valueAsText
        prefix = p["out_name"].valueAsText or "mission"
        if prefix in ("area_mission", "route_mission"):        # default prefix -> follow the name
            prefix = core.safe_filename(mission.params.mission_name, prefix)
        out = self._write_outputs(mission, gdb, prefix, work_sr,
                                  float(_val(p, "kp_interval", 0)) if self.route else 0)
        out.update(self._write_terrain(mission, gdb, prefix, work_sr))
        for key, path in out.items():
            p[key].value = path

        # ---- exports / upload
        to84 = _to_wgs84_fn(work_sr)
        files = []
        folder = p["export_folder"].valueAsText
        fmts = [FORMAT_LABELS[f] for f in (p["formats"].values or [])] if p["formats"].value else []
        if fmts and not folder and p["device_folder"].value is None:
            folder = arcpy.env.scratchFolder
            _warn("No export folder given - writing files to %s" % folder)
        if fmts:
            folder = folder or arcpy.env.scratchFolder
            for k in fmts:
                ext = exporters.FORMATS[k][1]
                tag = "" if k not in ("csv",) else "_waypoints"
                path = os.path.join(folder, prefix + tag + ext)
                kw = {"firmware": p["firmware"].valueAsText.lower()} if k == "qgc_plan" else {}
                if k in ("brief", "brief_pdf") and self.route:
                    kw["kp_interval"] = float(_val(p, "kp_interval", 0)) or None
                if k in ("brief", "brief_pdf"):
                    kw["background"] = _brief_basemap(p["brief_basemap"].valueAsText, work_sr)
                if k == "dji_kmz" and p["dji_template"].value is not None:
                    kw["template"] = p["dji_template"].valueAsText
                fw = exporters.export(mission, k, path, to84, per_flight=True, **kw)
                files += fw
                for f in fw:
                    _msg("Exported %s" % f)
        if p["device_folder"].value is not None and files:
            for f in uploader.copy_to_device(files, p["device_folder"].valueAsText):
                _msg("Copied to device: %s" % f)
            _msg(uploader.DJI_PILOT2_HINT)
        conn = p["mavlink"].valueAsText
        if conn:
            k = max(1, int(_val(p, "upload_flight", 1))) - 1
            if k >= len(mission.flights):
                raise arcpy.ExecuteError("Flight %d does not exist (mission has %d)."
                                         % (k + 1, len(mission.flights)))
            try:
                n = uploader.upload_mavlink(mission, to84, conn, int(_val(p, "baud", 57600)), k, log=_msg)
                _msg("Uploaded flight %d (%d items). Verify in your ground station before flight."
                     % (k + 1, n))
            except uploader.UploadError as e:
                raise arcpy.ExecuteError(str(e))
        return

    @staticmethod
    def _write_outputs(m, gdb, prefix, sr, kp_interval=0):
        out = {}
        A = arcpy.Array
        Pt = arcpy.Point

        path = _new_fc(gdb, prefix + "_boundary", "POLYGON", sr, [("area_ha", "DOUBLE")])
        with arcpy.da.InsertCursor(path, ["SHAPE@", "area_ha"]) as cur:
            arr = A()
            for i, ring in enumerate(m.boundary):
                ring = list(ring)
                if ring[0] != ring[-1]:
                    ring.append(ring[0])
                arr.add(A([Pt(*q) for q in ring]))
            cur.insertRow([arcpy.Polygon(arr, sr), round(m.area_m2 / 1e4, 3)])
        out["out_boundary"] = path

        path = _new_fc(gdb, prefix + "_legs", "POLYLINE", sr, [("leg", "LONG"), ("length_m", "DOUBLE")])
        with arcpy.da.InsertCursor(path, ["SHAPE@", "leg", "length_m"]) as cur:
            for i, (a, b) in enumerate(m.lines):
                cur.insertRow([arcpy.Polyline(A([Pt(*a), Pt(*b)]), sr), i + 1,
                               round(math.hypot(b[0] - a[0], b[1] - a[1]), 1)])
        out["out_legs"] = path

        sched = {r.flight: r for r in (m.schedule.rows if m.schedule else [])}
        t0 = m.schedule.params.start_time_min if m.schedule else 0
        path = _new_fc(gdb, prefix + "_routes", "POLYLINE", sr,
                       [("flight", "LONG"), ("minutes", "DOUBLE"), ("battery_pct", "DOUBLE"),
                        ("photos", "LONG"), ("length_m", "DOUBLE"), ("launch", "TEXT"),
                        ("land", "TEXT"), ("battery_set", "LONG"), ("start_time", "TEXT"),
                        ("end_time", "TEXT"), ("kp_from", "DOUBLE"), ("kp_to", "DOUBLE"),
                        ("ends_by", "TEXT")])
        cols = ["SHAPE@", "flight", "minutes", "battery_pct", "photos", "length_m", "launch", "land",
                "battery_set", "start_time", "end_time", "kp_from", "kp_to", "ends_by"]
        with arcpy.da.InsertCursor(path, cols) as cur:
            for f in m.flights:
                r = sched.get(f.index + 1)
                cur.insertRow([arcpy.Polyline(A([Pt(w.x, w.y) for w in f.waypoints]), sr), f.index + 1,
                               round(f.duration_s / 60, 1), round(f.battery_fraction * 100),
                               f.photos, round(f.distance_m), m.label(f.station_start),
                               m.label(f.station_end), r.battery_set if r else None,
                               core.clock(t0 + r.start_min) if r else None,
                               core.clock(t0 + r.end_min) if r else None,
                               round(f.chainage[0] / 1000, 3) if f.chainage else None,
                               round(f.chainage[1] / 1000, 3) if f.chainage else None,
                               f.end_reason or None])
        out["out_routes"] = path

        path = _new_fc(gdb, prefix + "_waypoints", "POINT", sr,
                       [("flight", "LONG"), ("seq", "LONG"), ("kind", "TEXT"), ("alt_m", "DOUBLE"),
                        ("speed_ms", "DOUBLE"), ("photo_m", "DOUBLE")])
        with arcpy.da.InsertCursor(path, ["SHAPE@XY", "flight", "seq", "kind", "alt_m", "speed_ms",
                                          "photo_m"]) as cur:
            for f in m.flights:
                for i, w in enumerate(f.waypoints):
                    cur.insertRow([(w.x, w.y), f.index + 1, i, w.kind, round(w.alt, 1),
                                   round(w.speed, 1), round(w.photo_dist, 1)])
        out["out_waypoints"] = path

        fps = m.photo_footprints()
        path = _new_fc(gdb, prefix + "_photos", "POINT", sr, [("n", "LONG"), ("flight", "LONG")])
        with arcpy.da.InsertCursor(path, ["SHAPE@XY", "n", "flight"]) as cur:
            for f in fps:
                cur.insertRow([(f["x"], f["y"]), f["n"], f["flight"]])
        out["out_photos"] = path

        path = _new_fc(gdb, prefix + "_footprints", "POLYGON", sr,
                       [("flight", "LONG"), ("photo", "LONG"), ("agl_m", "DOUBLE"), ("gsd_cm", "DOUBLE"),
                        ("across_m", "DOUBLE"), ("along_m", "DOUBLE"), ("heading", "DOUBLE")])
        with arcpy.da.InsertCursor(path, ["SHAPE@", "flight", "photo", "agl_m", "gsd_cm", "across_m",
                                          "along_m", "heading"]) as cur:
            for f in fps:
                cur.insertRow([arcpy.Polygon(A([Pt(*q) for q in f["ring"]]), sr), f["flight"], f["n"],
                               round(f["agl_m"], 1), round(f["gsd_cm"], 2), round(f["across_m"], 1),
                               round(f["along_m"], 1), round(f["heading_deg"], 1)])
        out["out_footprints"] = path

        path = _new_fc(gdb, prefix + "_site", "POINT", sr,
                       [("role", "TEXT"), ("name", "TEXT"), ("flights", "LONG")])
        with arcpy.da.InsertCursor(path, ["SHAPE@XY", "role", "name", "flights"]) as cur:
            for k, st in enumerate(m.stations):
                n = sum(1 for f in m.flights if tuple(f.station_start) == tuple(st))
                cur.insertRow([tuple(st), "takeoff" if k == 0 else "station", m.label(st), n])
            if tuple(m.landing) not in [tuple(s_) for s_ in m.stations]:
                cur.insertRow([tuple(m.landing), "landing", m.label(m.landing), 0])
        out["out_site"] = path

        path = _new_fc(gdb, prefix + "_chainage", "POINT", sr, [("kp_km", "DOUBLE")])
        if m.centreline and kp_interval > 0:
            with arcpy.da.InsertCursor(path, ["SHAPE@XY", "kp_km"]) as cur:
                for x, y, c in m.chainage_markers(kp_interval):
                    cur.insertRow([(x, y), round(c / 1000, 3)])
        out["out_chainage"] = path

        name = arcpy.ValidateTableName(prefix + "_schedule", gdb)
        tpath = os.path.join(gdb, name)
        if arcpy.Exists(tpath):
            arcpy.management.Delete(tpath)
        arcpy.management.CreateTable(gdb, name)
        tf = [("flight", "LONG"), ("launch", "TEXT"), ("land", "TEXT"), ("battery_set", "LONG"),
              ("start_time", "TEXT"), ("end_time", "TEXT"), ("minutes", "DOUBLE"),
              ("battery_pct", "DOUBLE"), ("wait_min", "DOUBLE"), ("drive_min", "DOUBLE"),
              ("kp_from", "DOUBLE"), ("kp_to", "DOUBLE"), ("notes", "TEXT")]
        for fname, ftype in tf:
            arcpy.management.AddField(tpath, fname, ftype)
        with arcpy.da.InsertCursor(tpath, [f_[0] for f_ in tf]) as cur:
            for f in m.flights:
                r = sched.get(f.index + 1)
                cur.insertRow([f.index + 1, m.label(f.station_start), m.label(f.station_end),
                               r.battery_set if r else None,
                               core.clock(t0 + r.start_min) if r else None,
                               core.clock(t0 + r.end_min) if r else None,
                               round(f.duration_s / 60, 1), round(f.battery_fraction * 100),
                               round(r.wait_min, 1) if r else 0, round(r.relocate_min, 1) if r else 0,
                               round(f.chainage[0] / 1000, 3) if f.chainage else None,
                               round(f.chainage[1] / 1000, 3) if f.chainage else None,
                               (r.note if r else "")[:250]])
        out["out_schedule"] = tpath
        return out

    @staticmethod
    def _write_terrain(m, gdb, prefix, sr):
        out = {}
        rep = m.terrain
        if rep is None:
            return out
        path = _new_fc(gdb, prefix + "_terrain", "POINT", sr,
                       [("kind", "TEXT"), ("flight", "LONG"), ("value_m", "DOUBLE"),
                        ("ground_m", "DOUBLE"), ("alt_amsl", "DOUBLE"), ("kp_km", "DOUBLE")])
        with arcpy.da.InsertCursor(path, ["SHAPE@XY", "kind", "flight", "value_m", "ground_m",
                                          "alt_amsl", "kp_km"]) as cur:
            for i in rep.issues:
                cur.insertRow([(i.x, i.y), i.kind, i.flight, round(i.value_m, 1), round(i.ground_m, 1),
                               round(i.alt_amsl, 1), round(i.kp / 1000, 3) if i.kp is not None else None])
        out["out_terrain"] = path
        if m.profile:
            name = arcpy.ValidateTableName(prefix + "_profile", gdb)
            tpath = os.path.join(gdb, name)
            if arcpy.Exists(tpath):
                arcpy.management.Delete(tpath)
            arcpy.management.CreateTable(gdb, name)
            cols = [("kp_km", "DOUBLE"), ("ground_m", "DOUBLE"), ("flight_min_m", "DOUBLE"),
                    ("flight_max_m", "DOUBLE"), ("clearance_min_m", "DOUBLE")]
            for c, t in cols:
                arcpy.management.AddField(tpath, c, t)
            rnd = lambda v: round(v, 1) if v is not None else None
            with arcpy.da.InsertCursor(tpath, [c for c, _ in cols]) as cur:
                for r in m.profile:
                    cur.insertRow([round(r["kp"] / 1000, 3), rnd(r["ground"]), rnd(r["flight_min"]),
                                   rnd(r["flight_max"]), rnd(r["clearance_min"])])
            out["out_profile"] = tpath
            _msg("Elevation profile table: %s (chart it in Pro: right-click > Create Chart > Line)" % tpath)
        return out


class AreaMission(_MissionTool):
    route = False

    def __init__(self):
        super(AreaMission, self).__init__()
        self.label = "Create Area Mapping Mission"
        self.description = ("Plans a lawn-mower (or crosshatch) photogrammetry grid over a "
                            "drawn, layer or CAD polygon with battery-aware flight splitting.")


class RouteMission(_MissionTool):
    route = True

    def __init__(self):
        super(RouteMission, self).__init__()
        self.label = "Create Route Mapping Mission"
        self.description = ("Plans parallel corridor lines along a road, rail, pipeline, "
                            "powerline or river centreline.")


# ======================================================================
class FlightTimeCalculator(object):
    def __init__(self):
        self.label = "Flight Time Calculator"
        self.description = ("Estimates usable flight time and area per battery for given wind, "
                            "temperature, altitude and payload, and the speed that gets the most "
                            "out of each battery.")
        self.canRunInBackground = False

    def getParameterInfo(self):
        ps = _aircraft_params() + _conditions_params()
        ps += [_param("gsd", "Ground sample distance (cm/px)", "GPDouble", value=2.5),
               _param("front", "Front overlap (%)", "GPDouble", value=75.0),
               _param("side", "Side overlap (%)", "GPDouble", value=65.0),
               _param("heading", "Typical line heading (deg)", "GPDouble", value=0.0)]
        for c in ps:                    # show conditions up front for this tool
            if c.category and c.category.startswith("Conditions"):
                c.category = None
        return ps

    def updateParameters(self, parameters):
        p = _by_name(parameters)
        _convert_wind_on_switch(self, p)
        _update_aircraft_defaults(p)

    def updateMessages(self, parameters):
        _payload_messages(_by_name(parameters))

    def execute(self, parameters, messages):
        p = _by_name(parameters)
        d = _drone_from_params(p)
        c = _conditions_from_params(p)
        cam = d.camera
        gsd = float(_val(p, "gsd", 2.5))
        h = P.height_from_gsd(cam, gsd)
        model = core.EnduranceModel(d, c, h)
        spacing = P.line_spacing(cam, h, float(_val(p, "side", 65)))
        v_cam, why = P.max_speed_for_camera(cam, h, float(_val(p, "front", 75)))
        v = model.best_speed([float(_val(p, "heading", 0))], max(2.0, min(v_cam, d.max_speed_ms)))
        usable = model.usable_minutes
        s = model.summary()
        _msg("Aircraft: %s   height for %.2f cm/px: %.1f m" % (d.name, gsd, h))
        if c.items or c.payload_kg:
            pu = p["payload_unit"].valueAsText or "kg"
            _msg("Payload:")
            for it in c.items:
                _msg("  %-24s %s%s" % (it.name, core.units.mass_fmt(it.mass_kg, pu),
                                       ("   %.0f W" % it.power_w) if it.power_w else ""))
            if c.payload_kg:
                _msg("  %-24s %s" % ("other (unlisted)", core.units.mass_fmt(c.payload_kg, pu)))
            _msg("  Total %s, %.0f W accessory power (aircraft uses ~%.0f W to fly)"
                 % (core.units.mass_fmt(c.total_payload_kg, pu), c.accessory_power_w, model.flight_power_w))
            _msg("  Endurance %.1f min with accessories; %.1f min if they drew no power"
                 % (model.usable_minutes, model.flight_minutes))
        _msg("Take-off mass %.2f kg | air density %.3f kg/m3 | battery temp factor %.2f"
             % (s["take_off_mass_kg"], s["air_density_kg_m3"], s["battery_temp_factor"]))
        _msg("Rated %.0f min -> usable %.1f min per battery (still air, %.0f%% reserve kept)"
             % (d.rated_flight_time_min, usable, c.reserve_pct))
        _msg("Best speed %.1f m/s (camera limit %.1f m/s - %s)" % (v, v_cam, why))
        _msg("Approx. area per battery: %.1f ha  (~85%% time on lines)"
             % (spacing * v * usable * 60 * 0.85 / 1e4))
        _msg("")
        _msg("Wind sensitivity (same heading, wind along the lines):")
        wu = _wunit(p)
        steps = {"kn": [0, 5, 10, 15, 20, 25], "m/s": [0, 3, 6, 9, 12], "km/h": [0, 10, 20, 30, 40, 50]}[wu]
        for wv in steps:
            w = core.units.wind_to_ms(wv, wu)
            cc = core.Conditions(**dict(c.__dict__, wind_speed_ms=w,
                                        wind_from_deg=float(_val(p, "heading", 0))))
            mm = core.EnduranceModel(d, cc, h)
            r = math.radians(float(_val(p, "heading", 0)))
            f1 = mm.leg((0, 0), (math.sin(r) * 1000, math.cos(r) * 1000), v)[1]
            f2 = mm.leg((0, 0), (-math.sin(r) * 1000, -math.cos(r) * 1000), v)[1]
            km = 2.0 / (f1 + f2) if f1 + f2 > 0 else 0
            flag = "  (above aircraft limit)" if w > d.max_wind_ms else ""
            _msg("  wind %2d %-4s: %.1f km of survey line per battery%s" % (wv, wu, km, flag))
        for wmsg in model.warnings:
            _warn(wmsg)


def _replace_param():
    r = _param("replace_file", "Mission file to replace on the controller (DJI Fly: Android/data/dji.go.v5/"
               "files/waypoint/<id>/<id>.kmz)", "DEFile", "Optional", enabled=False)
    r.filter.list = ["kmz"]
    return r


class UploadMission(object):
    def __init__(self):
        self.label = "Upload Mission to Drone"
        self.description = ("Uploads an exported .plan / .waypoints file live over MAVLink "
                            "(ArduPilot / PX4), or copies any mission file to a controller, SD "
                            "card or USB stick (DJI Pilot 2, Litchi, Autel).")
        self.canRunInBackground = False

    def getParameterInfo(self):
        f = _param("in_file", "Mission file", "DEFile")
        f.filter.list = ["plan", "waypoints", "kmz", "kml", "csv", "geojson"]
        return [f,
                _param("method", "Method", "GPString",
                       values=["Live MAVLink upload (ArduPilot / PX4)",
                               "Copy to controller / SD card folder",
                               "Replace a mission file made in DJI Fly / Pilot 2 (Mini 4 Pro, Mini 5 Pro ...)"],
                       value="Live MAVLink upload (ArduPilot / PX4)"),
                _param("mavlink", "Connection (COM5, /dev/ttyUSB0, udpin:0.0.0.0:14550, tcp:127.0.0.1:5760)",
                       "GPString", "Optional", value="udpin:0.0.0.0:14550"),
                _param("baud", "Serial baud rate", "GPLong", value=57600),
                _param("device_folder", "Destination folder", "DEFolder", "Optional", enabled=False),
                _replace_param()]

    def updateParameters(self, parameters):
        p = _by_name(parameters)
        live = p["method"].valueAsText.startswith("Live")
        repl = p["method"].valueAsText.startswith("Replace")
        p["mavlink"].enabled = p["baud"].enabled = live
        p["device_folder"].enabled = not live and not repl
        p["replace_file"].enabled = repl

    def updateMessages(self, parameters):
        p = _by_name(parameters)
        live = p["method"].valueAsText.startswith("Live")
        f = p["in_file"].valueAsText or ""
        if live and f and not f.lower().endswith((".plan", ".waypoints")):
            p["in_file"].setErrorMessage("Live upload needs a .plan or .waypoints file. Use 'Copy "
                                         "to controller' for DJI .kmz, Litchi .csv or .kml.")
        if live and not uploader.pymavlink_available():
            p["method"].setWarningMessage("pymavlink is not installed in this Python environment.")
        repl = p["method"].valueAsText.startswith("Replace")
        if repl:
            if f and not f.lower().endswith(".kmz"):
                p["in_file"].setErrorMessage("Replacing a DJI mission needs a DJI WPML .kmz export.")
            if not p["replace_file"].value:
                p["replace_file"].setErrorMessage("Choose the .kmz that DJI Fly / Pilot 2 created.")
        elif not live and not p["device_folder"].value:
            p["device_folder"].setErrorMessage("Choose the controller / SD card folder.")

    def execute(self, parameters, messages):
        p = _by_name(parameters)
        f = p["in_file"].valueAsText
        if p["method"].valueAsText.startswith("Live"):
            try:
                n = uploader.upload_file(f, p["mavlink"].valueAsText, int(_val(p, "baud", 57600)), log=_msg)
                _msg("Mission accepted: %d items. Verify in your ground station before flight." % n)
            except uploader.UploadError as e:
                raise arcpy.ExecuteError(str(e))
        elif p["method"].valueAsText.startswith("Replace"):
            try:
                uploader.replace_dji_mission(f, p["replace_file"].valueAsText, log=_msg)
            except Exception as e:
                raise arcpy.ExecuteError("Could not replace the mission: %s" % e)
            _msg("Restart DJI Fly / Pilot 2 and open that route - check every waypoint before flight.")
        else:
            for t in uploader.copy_to_device([f], p["device_folder"].valueAsText):
                _msg("Copied to %s" % t)
            _msg(uploader.DJI_PILOT2_HINT)


class DownloadDEM(object):
    def __init__(self):
        self.label = "Download DEM"
        self.description = ("Downloads free elevation data around features (or a drawn area): AWS "
                            "Terrain Tiles (works everywhere) or Copernicus GLO-30 30 m surface model "
                            "(needs GDAL in the Python environment).")
        self.canRunInBackground = False

    def getParameterInfo(self):
        a = _param("in_features", "Area (draw, or any layer / CAD)", "GPFeatureRecordSetLayer")
        a.filter.list = ["Polygon", "Polyline", "Point"]
        src = _param("source", "Source", "GPString", values=DEM_SOURCES, value=DEM_SOURCES[0])
        buf = _param("buffer", "Margin around the area (m)", "GPDouble", value=1500.0)
        out = _param("out_raster", "Output DEM", "DERasterDataset", direction="Output")
        try:
            out.value = os.path.join(arcpy.env.workspace or arcpy.env.scratchGDB, "dem")
        except Exception:
            pass
        return [a, src, buf, out]

    def updateParameters(self, parameters):
        return

    def updateMessages(self, parameters):
        p = _by_name(parameters)
        if (p["source"].valueAsText or "").startswith("Copernicus"):
            try:
                from osgeo import gdal  # noqa: F401
            except ImportError:
                p["source"].setErrorMessage("GDAL is not available in this Python environment - "
                                            "use AWS Terrain Tiles.")

    def execute(self, parameters, messages):
        p = _by_name(parameters)
        fs = p["in_features"].valueAsText
        geoms, sr = _read_geoms(fs)
        pts = []
        for g in geoms:
            g84 = _project(g, WGS84)
            for part in (g84 if not isinstance(g84, arcpy.PointGeometry) else [[g84.firstPoint]]):
                pts += [(q.X, q.Y) for q in part if q]
        if not pts:
            raise arcpy.ExecuteError("No geometry to download a DEM for.")
        _download_dem(p["source"].valueAsText, pts, float(_val(p, "buffer", 1500)),
                      p["out_raster"].valueAsText)


class Toolbox(object):
    def __init__(self):
        self.label = "Drone Mission Planner"
        self.alias = "dronemission"
        self.tools = [AreaMission, RouteMission, FlightTimeCalculator, UploadMission, DownloadDEM]
