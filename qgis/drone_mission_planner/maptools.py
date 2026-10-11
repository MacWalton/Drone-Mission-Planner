# SPDX-License-Identifier: GPL-2.0-only
"""Map tools: draw a polygon / line, pick a point."""
from qgis.PyQt.QtCore import Qt, pyqtSignal
from qgis.PyQt.QtGui import QColor
from qgis.core import QgsGeometry, QgsPointXY, QgsWkbTypes
from qgis.gui import QgsMapTool, QgsRubberBand

try:                                   # QGIS >= 3.30
    from qgis.core import Qgis
    POLY_T, LINE_T = Qgis.GeometryType.Polygon, Qgis.GeometryType.Line
except Exception:                      # older QGIS 3.x
    POLY_T, LINE_T = QgsWkbTypes.PolygonGeometry, QgsWkbTypes.LineGeometry

LEFT = Qt.MouseButton.LeftButton
RIGHT = Qt.MouseButton.RightButton


class CaptureTool(QgsMapTool):
    """Left-click to add vertices, right-click (or Enter) to finish,
    Backspace removes the last vertex, Esc cancels."""
    finished = pyqtSignal(object)      # QgsGeometry in canvas CRS
    cancelled = pyqtSignal()

    def __init__(self, canvas, polygon=True):
        super().__init__(canvas)
        self.canvas = canvas
        self.polygon = polygon
        self.points = []
        self.band = QgsRubberBand(canvas, POLY_T if polygon else LINE_T)
        self.band.setColor(QColor(255, 165, 0, 200))
        self.band.setFillColor(QColor(255, 165, 0, 50))
        self.band.setWidth(2)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def _refresh(self, hover=None):
        pts = list(self.points) + ([hover] if hover else [])
        self.band.reset(POLY_T if self.polygon else LINE_T)
        for p in pts:
            self.band.addPoint(p, False)
        self.band.show()
        self.band.updatePosition()
        self.canvas.refresh()

    def canvasMoveEvent(self, e):
        if self.points:
            self._refresh(self.toMapCoordinates(e.pos()))

    def canvasReleaseEvent(self, e):
        if e.button() == LEFT:
            self.points.append(QgsPointXY(self.toMapCoordinates(e.pos())))
            self._refresh()
        elif e.button() == RIGHT:
            self._finish()

    def keyPressEvent(self, e):
        k = e.key()
        if k in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            self._finish()
        elif k == Qt.Key.Key_Backspace and self.points:
            self.points.pop()
            self._refresh()
        elif k == Qt.Key.Key_Escape:
            self.reset()
            self.cancelled.emit()

    def _finish(self):
        need = 3 if self.polygon else 2
        if len(self.points) < need:
            return
        if self.polygon:
            g = QgsGeometry.fromPolygonXY([self.points + [self.points[0]]])
        else:
            g = QgsGeometry.fromPolylineXY(self.points)
        self.reset()
        self.finished.emit(g)

    def reset(self):
        self.points = []
        self.band.reset(POLY_T if self.polygon else LINE_T)

    def deactivate(self):
        self.reset()
        super().deactivate()


class PointTool(QgsMapTool):
    picked = pyqtSignal(object)        # QgsPointXY in canvas CRS

    def __init__(self, canvas):
        super().__init__(canvas)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def canvasReleaseEvent(self, e):
        if e.button() == LEFT:
            self.picked.emit(QgsPointXY(self.toMapCoordinates(e.pos())))



class LinePickerTool(QgsMapTool):
    """Click line features (roads, pipelines, powerlines…) in any visible line
    layer to collect them; click again to drop one. Right-click or Enter uses
    the collection, Esc cancels. Pieces are chained into one route afterwards."""
    finished = pyqtSignal(object)      # list of QgsGeometry in canvas CRS
    cancelled = pyqtSignal()

    def __init__(self, canvas, exclude_ids=()):
        super().__init__(canvas)
        self.canvas = canvas
        self.exclude = set(exclude_ids)
        self.picked = {}                       # (layer_id, fid) -> (QgsGeometry, rubber band)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def _line_layers(self):
        from qgis.core import QgsVectorLayer
        out = []
        for lyr in self.canvas.layers():
            if isinstance(lyr, QgsVectorLayer) and lyr.id() not in self.exclude:
                t = lyr.geometryType()
                name = (getattr(t, "name", None) or str(t)).lower()
                if "line" in name or t == 1:
                    out.append(lyr)
        return out

    def canvasReleaseEvent(self, e):
        if e.button() == RIGHT:
            self._finish()
            return
        if e.button() != LEFT:
            return
        from qgis.core import (QgsFeatureRequest, QgsRectangle, QgsCoordinateTransform,
                               QgsProject)
        pt = self.toMapCoordinates(e.pos())
        tol = self.canvas.mapUnitsPerPixel() * 6
        click = QgsGeometry.fromPointXY(QgsPointXY(pt))
        best = None
        dst = self.canvas.mapSettings().destinationCrs()
        for lyr in self._line_layers():
            xf = QgsCoordinateTransform(dst, lyr.crs(), QgsProject.instance())
            back = QgsCoordinateTransform(lyr.crs(), dst, QgsProject.instance())
            rect = xf.transformBoundingBox(QgsRectangle(pt.x() - tol, pt.y() - tol,
                                                        pt.x() + tol, pt.y() + tol))
            for f in lyr.getFeatures(QgsFeatureRequest().setFilterRect(rect)):
                g = QgsGeometry(f.geometry())
                g.transform(back)
                dd = g.distance(click)
                if dd <= tol and (best is None or dd < best[0]):
                    best = (dd, lyr, f.id(), g)
        if best is None:
            return
        _, lyr, fid, g = best
        key = (lyr.id(), fid)
        if key in self.picked:                       # toggle off
            self.picked.pop(key)[1].reset(LINE_T)
        else:
            rb = QgsRubberBand(self.canvas, LINE_T)
            rb.setColor(QColor(255, 120, 0, 220))
            rb.setWidth(4)
            rb.setToGeometry(g, None)
            self.picked[key] = (g, rb)
        self.canvas.refresh()

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            self._finish()
        elif e.key() == Qt.Key.Key_Escape:
            self.reset()
            self.cancelled.emit()

    def _finish(self):
        geoms = [g for g, _ in self.picked.values()]
        self.reset()
        if geoms:
            self.finished.emit(geoms)

    def reset(self):
        for _, rb in self.picked.values():
            rb.reset(LINE_T)
        self.picked = {}

    def deactivate(self):
        self.reset()
        super().deactivate()
