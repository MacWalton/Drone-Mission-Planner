# SPDX-License-Identifier: GPL-2.0-only
"""Memory layers that hold the editable inputs and the planned mission."""
from qgis.PyQt.QtCore import QVariant
from qgis.PyQt.QtGui import QColor
from qgis.core import (QgsProject, QgsVectorLayer, QgsFeature, QgsField, QgsGeometry,
                       QgsPointXY, QgsCoordinateReferenceSystem, QgsCoordinateTransform,
                       QgsMarkerSymbol, QgsLineSymbol, QgsFillSymbol,
                       QgsCategorizedSymbolRenderer, QgsRendererCategory,
                       QgsPalLayerSettings, QgsVectorLayerSimpleLabeling, QgsWkbTypes)

GROUP = "Drone Mission"
BOUNDARY = "Mission boundary"
CENTRELINE = "Route centreline"
SITE = "Take-off & landing"
LEGS = "Survey legs"
ROUTES = "Flight routes"
WAYPOINTS = "Waypoints"
PHOTOS = "Photo centres"
FOOTPRINT = "Corridor footprint"
CHAINAGE = "Chainage"
TERRAIN = "Terrain warnings"

try:
    from qgis.core import QgsField as _F
    from qgis.PyQt.QtCore import QMetaType
    T_STR, T_INT, T_DBL = QMetaType.Type.QString, QMetaType.Type.Int, QMetaType.Type.Double
    _ = _F("x", T_STR)
except Exception:
    T_STR, T_INT, T_DBL = QVariant.String, QVariant.Int, QVariant.Double


def project():
    return QgsProject.instance()


def group():
    root = project().layerTreeRoot()
    g = root.findGroup(GROUP)
    if g is None:
        g = root.insertGroup(0, GROUP)
    return g


def find(name):
    for lyr in project().mapLayersByName(name):
        if isinstance(lyr, QgsVectorLayer) and lyr.customProperty("dmp_role") == name:
            return lyr
    return None


def get_or_create(name, geom, crs, fields, style=None):
    lyr = find(name)
    if lyr is not None and lyr.crs() != crs and lyr.featureCount() == 0:
        project().removeMapLayer(lyr.id())
        lyr = None
    if lyr is None:
        lyr = QgsVectorLayer(geom, name, "memory")
        lyr.setCrs(crs)
        lyr.dataProvider().addAttributes([QgsField(n, t) for n, t in fields])
        lyr.updateFields()
        lyr.setCustomProperty("dmp_role", name)
        if style:
            style(lyr)
        project().addMapLayer(lyr, False)
        group().insertLayer(0, lyr)
    return lyr


def clear(lyr):
    if lyr.isEditable():
        lyr.rollBack()
    lyr.dataProvider().truncate()
    lyr.triggerRepaint()


def add(lyr, geoms_attrs):
    feats = []
    for g, attrs in geoms_attrs:
        f = QgsFeature(lyr.fields())
        f.setGeometry(g)
        f.setAttributes(list(attrs))
        feats.append(f)
    lyr.dataProvider().addFeatures(feats)
    lyr.updateExtents()
    lyr.triggerRepaint()


# ------------------------------------------------------------------ styles
def _style_boundary(l):
    l.renderer().setSymbol(QgsFillSymbol.createSimple(
        {"color": "255,165,0,40", "outline_color": "255,140,0", "outline_width": "0.8"}))


def _style_centre(l):
    l.renderer().setSymbol(QgsLineSymbol.createSimple(
        {"line_color": "255,140,0", "line_width": "0.9", "line_style": "dash"}))


def _style_legs(l):
    l.renderer().setSymbol(QgsLineSymbol.createSimple({"line_color": "0,160,220", "line_width": "0.6"}))


def _style_routes(l):
    cats = []
    palette = ["30,120,255", "230,60,60", "40,170,80", "160,70,200", "240,150,20", "0,170,170"]
    for i in range(1, 101):
        s = QgsLineSymbol.createSimple({"line_color": palette[(i - 1) % len(palette)],
                                        "line_width": "0.5"})
        cats.append(QgsRendererCategory(i, s, "Flight %d" % i))
    l.setRenderer(QgsCategorizedSymbolRenderer("flight", cats))


def _style_wps(l):
    l.renderer().setSymbol(QgsMarkerSymbol.createSimple(
        {"name": "circle", "color": "255,255,255", "outline_color": "30,30,30", "size": "1.6"}))


def _style_photos(l):
    l.renderer().setSymbol(QgsMarkerSymbol.createSimple(
        {"name": "cross2", "color": "120,120,120", "size": "1.2"}))


def _style_site(l):
    tk = QgsMarkerSymbol.createSimple({"name": "triangle", "color": "40,200,60",
                                       "outline_color": "0,0,0", "size": "5"})
    st = QgsMarkerSymbol.createSimple({"name": "triangle", "color": "30,130,230",
                                       "outline_color": "0,0,0", "size": "4.5"})
    ld = QgsMarkerSymbol.createSimple({"name": "square", "color": "220,40,40",
                                       "outline_color": "0,0,0", "size": "4.5"})
    l.setRenderer(QgsCategorizedSymbolRenderer("role", [
        QgsRendererCategory("takeoff", tk, "Take-off (S1)"),
        QgsRendererCategory("station", st, "Launch station"),
        QgsRendererCategory("landing", ld, "Final landing")]))
    s = QgsPalLayerSettings()
    s.isExpression = True
    s.fieldName = "coalesce(nullif(\"name\", ''), \"role\")"
    s.enabled = True
    l.setLabeling(QgsVectorLayerSimpleLabeling(s))
    l.setLabelsEnabled(True)


def boundary_layer(crs, route=False):
    if route:
        return get_or_create(CENTRELINE, "LineString", crs, [("name", T_STR)], _style_centre)
    return get_or_create(BOUNDARY, "Polygon", crs, [("name", T_STR)], _style_boundary)


def site_layer(crs):
    return get_or_create(SITE, "Point", crs, [("role", T_STR), ("name", T_STR)], _style_site)


def _style_chainage(l):
    l.renderer().setSymbol(QgsMarkerSymbol.createSimple(
        {"name": "line", "color": "120,60,0", "outline_color": "120,60,0", "size": "3",
         "angle": "90"}))
    s = QgsPalLayerSettings()
    s.isExpression = True
    s.fieldName = "'KP ' || format_number(\"kp_km\", 3)"
    l.setLabeling(QgsVectorLayerSimpleLabeling(s))
    l.setLabelsEnabled(True)


def chainage_layer(crs):
    lyr = get_or_create(CHAINAGE, "Point", crs, [("kp_km", T_DBL)], _style_chainage)
    if lyr.crs() != crs:
        clear(lyr)
        lyr.setCrs(crs)
    return lyr


def output_layers(crs):
    out = {
        LEGS: get_or_create(LEGS, "LineString", crs, [("leg", T_INT), ("length_m", T_DBL)], _style_legs),
        ROUTES: get_or_create(ROUTES, "LineString", crs,
                              [("flight", T_INT), ("minutes", T_DBL), ("battery_pct", T_DBL),
                               ("photos", T_INT), ("length_m", T_DBL), ("launch", T_STR),
                               ("land", T_STR), ("battery_set", T_INT), ("start", T_STR),
                               ("end", T_STR), ("kp_from", T_DBL), ("kp_to", T_DBL),
                               ("ends_by", T_STR)], _style_routes),
        WAYPOINTS: get_or_create(WAYPOINTS, "Point", crs,
                                 [("flight", T_INT), ("seq", T_INT), ("kind", T_STR),
                                  ("alt_m", T_DBL), ("speed_ms", T_DBL), ("photo_m", T_DBL)],
                                 _style_wps),
        PHOTOS: get_or_create(PHOTOS, "Point", crs, [("n", T_INT)], _style_photos),
    }
    for lyr in out.values():                 # working CRS may have changed
        if lyr.crs() != crs:
            clear(lyr)
            lyr.setCrs(crs)
    return out


def transform(src_crs, dst_crs):
    return QgsCoordinateTransform(src_crs, dst_crs, project())


def _style_terrain(l):
    lo = QgsMarkerSymbol.createSimple({"name": "triangle", "color": "220,38,38",
                                       "outline_color": "255,255,255", "size": "5", "angle": "180"})
    hi = QgsMarkerSymbol.createSimple({"name": "triangle", "color": "234,140,0",
                                       "outline_color": "255,255,255", "size": "5"})
    ob = QgsMarkerSymbol.createSimple({"name": "diamond", "color": "208,59,59",
                                       "outline_color": "255,255,255", "size": "5"})
    l.setRenderer(QgsCategorizedSymbolRenderer("kind", [
        QgsRendererCategory("low", lo, "Low clearance"),
        QgsRendererCategory("obstacle", ob, "Too close to an obstacle"),
        QgsRendererCategory("high", hi, "Above height limit")]))
    s = QgsPalLayerSettings()
    s.isExpression = True
    s.fieldName = ("CASE WHEN \"kind\" = 'low' THEN 'F' || \"flight\" || ' clear ' || "
                   "round(\"value_m\") || ' m' ELSE 'F' || \"flight\" || ' ' || round(\"value_m\") || "
                   "' m AGL' END")
    l.setLabeling(QgsVectorLayerSimpleLabeling(s))
    l.setLabelsEnabled(True)


def terrain_layer(crs):
    lyr = get_or_create(TERRAIN, "Point", crs,
                        [("kind", T_STR), ("flight", T_INT), ("value_m", T_DBL), ("ground_m", T_DBL),
                         ("alt_amsl", T_DBL), ("kp_km", T_DBL)], _style_terrain)
    if lyr.crs() != crs:
        clear(lyr)
        lyr.setCrs(crs)
    return lyr
