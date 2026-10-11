# SPDX-License-Identifier: GPL-2.0-only
"""Route elevation profile chart: ground vs flight altitude by chainage (KP)."""
from qgis.PyQt.QtCore import Qt, QPointF, QRectF
from qgis.PyQt.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF, QFont
from qgis.PyQt.QtWidgets import QWidget, QToolTip

GROUND = QColor(214, 199, 161)
GROUND_LINE = QColor(139, 111, 62)
FLIGHT = QColor(14, 116, 144)
LOW = QColor(220, 38, 38)
GRID = QColor(128, 128, 128, 60)


class ProfileChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rows = []
        self.limit = 30.0
        self.unit = "m"
        self.setMinimumHeight(170)
        self.setMouseTracking(True)
        self.setToolTip("")

    def set_rows(self, rows, clearance_limit=30.0, unit="m"):
        self.rows = [r for r in (rows or []) if r.get("ground") is not None]
        self.limit = clearance_limit
        self.unit = unit
        self.setVisible(bool(self.rows))
        self.update()

    # geometry helpers
    def _frame(self):
        L, R, T, B = 46, 8, 18, 22
        return QRectF(L, T, max(10, self.width() - L - R), max(10, self.height() - T - B))

    def _ranges(self):
        zs = [r["ground"] for r in self.rows]
        zs += [r["flight_max"] for r in self.rows if r["flight_max"] is not None]
        zs += [r["flight_min"] for r in self.rows if r["flight_min"] is not None]
        z0, z1 = min(zs), max(zs)
        pad = max(5.0, (z1 - z0) * 0.08)
        return self.rows[0]["kp"], self.rows[-1]["kp"], z0 - pad, z1 + pad

    def paintEvent(self, _):
        if not self.rows:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        fr = self._frame()
        k0, k1, z0, z1 = self._ranges()
        X = lambda k: fr.left() + (k - k0) / max(k1 - k0, 1e-9) * fr.width()
        Y = lambda z: fr.top() + (z1 - z) / max(z1 - z0, 1e-9) * fr.height()
        small = QFont(self.font())
        small.setPointSizeF(max(6.5, small.pointSizeF() - 1.5))
        p.setFont(small)
        pal = self.palette()
        txt = pal.color(pal.ColorRole.WindowText) if hasattr(pal, "ColorRole") else pal.color(pal.WindowText)
        # grid + axes labels
        for i in range(5):
            z = z0 + (z1 - z0) * i / 4
            y = Y(z)
            p.setPen(QPen(GRID, 1))
            p.drawLine(QPointF(fr.left(), y), QPointF(fr.right(), y))
            p.setPen(txt)
            p.drawText(QRectF(0, y - 7, fr.left() - 4, 14),
                       int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
                       "%.0f" % (z / 0.3048 if self.unit == "ft" else z))
        for i in range(6):
            k = k0 + (k1 - k0) * i / 5
            if i == 5:
                rect, al = QRectF(X(k) - 60, fr.bottom() + 3, 60, 14), Qt.AlignmentFlag.AlignRight
            else:
                rect, al = QRectF(X(k) - 30, fr.bottom() + 3, 60, 14), Qt.AlignmentFlag.AlignHCenter
            p.drawText(rect, int(al), "%.2f" % (k / 1000.0))
        p.drawText(QRectF(fr.left(), 0, fr.width(), 16), int(Qt.AlignmentFlag.AlignLeft),
                   "Ground (brown) / flight altitude (blue), %s AMSL - by KP (km)" % self.unit)
        # ground area
        poly = QPolygonF([QPointF(X(k0), fr.bottom())] +
                         [QPointF(X(r["kp"]), Y(r["ground"])) for r in self.rows] +
                         [QPointF(X(k1), fr.bottom())])
        p.setPen(QPen(GROUND_LINE, 1.2))
        p.setBrush(QBrush(GROUND))
        p.drawPolygon(poly)
        # flight band
        fl = [r for r in self.rows if r["flight_max"] is not None]
        if fl:
            band = QPolygonF([QPointF(X(r["kp"]), Y(r["flight_max"])) for r in fl] +
                             [QPointF(X(r["kp"]), Y(r["flight_min"])) for r in reversed(fl)])
            c = QColor(FLIGHT)
            c.setAlpha(80)
            p.setPen(QPen(FLIGHT, 1.2))
            p.setBrush(QBrush(c))
            p.drawPolygon(band)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(LOW))
            for r in fl:
                if r["clearance_min"] is not None and r["clearance_min"] < self.limit:
                    p.drawEllipse(QPointF(X(r["kp"]), Y(r["flight_min"])), 3, 3)
        p.end()

    def mouseMoveEvent(self, e):
        if not self.rows:
            return
        fr = self._frame()
        k0, k1, _, _ = self._ranges()
        pos = e.position() if hasattr(e, "position") else e.pos()
        k = k0 + (pos.x() - fr.left()) / max(fr.width(), 1) * (k1 - k0)
        r = min(self.rows, key=lambda q: abs(q["kp"] - k))
        k = 1 / 0.3048 if self.unit == "ft" else 1.0
        u = self.unit
        s = "KP %.3f km\nground %.0f %s" % (r["kp"] / 1000.0, r["ground"] * k, u)
        if r["flight_max"] is not None:
            s += "\nflight %.0f-%.0f %s" % (r["flight_min"] * k, r["flight_max"] * k, u)
        if r["clearance_min"] is not None:
            s += "\nmin clearance %.0f %s" % (r["clearance_min"] * k, u)
        gp = e.globalPosition().toPoint() if hasattr(e, "globalPosition") else e.globalPos()
        QToolTip.showText(gp, s, self)
