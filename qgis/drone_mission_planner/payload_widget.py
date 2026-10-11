# SPDX-License-Identifier: GPL-2.0-only
"""Payload list: items carried on the aircraft with weight and power draw."""
import json

from qgis.PyQt.QtCore import Qt, pyqtSignal, QSettings
from qgis.PyQt.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
                                 QPushButton, QComboBox, QLabel, QHeaderView, QAbstractItemView)

import dronemapper_core as core

KEY = "dmp/payload_items"


class PayloadTable(QWidget):
    """Editable table: Item | Weight | Power (W). Weights are entered in the
    chosen unit (kg or lb) and handed to the planner in kg."""
    changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Item", "Weight", "Power (W)"])
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        hh.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setMinimumHeight(110)
        self.table.setToolTip("List everything carried: camera/sensor, lights, LiDAR, gimbal, mounts, "
                              "parachute… Power = watts drawn from the flight battery (0 if the item "
                              "has its own battery). Double-click a cell to edit.")
        self.table.itemChanged.connect(self._edited)
        lay.addWidget(self.table)
        row = QHBoxLayout()
        self.add_btn = QPushButton("Add item")
        self.add_btn.clicked.connect(lambda: self.add_item("New item", 0.0, 0.0, edit=True))
        self.del_btn = QPushButton("Remove")
        self.del_btn.clicked.connect(self.remove_selected)
        self.unit_cb = QComboBox()
        self.unit_cb.addItem("kg", "kg")
        self.unit_cb.addItem("lb", "lb")
        self.unit_cb.setToolTip("Weight unit for the list (the planner works in kg).")
        self.unit_cb.currentIndexChanged.connect(self._unit_changed)
        row.addWidget(self.add_btn)
        row.addWidget(self.del_btn)
        row.addStretch(1)
        row.addWidget(QLabel("Weights in"))
        row.addWidget(self.unit_cb)
        lay.addLayout(row)
        self.total_lbl = QLabel()
        self.total_lbl.setWordWrap(True)
        lay.addWidget(self.total_lbl)
        self._unit = "kg"
        self._loading = False
        self.load()

    # ------------------------------------------------------------ data
    @property
    def unit(self):
        return self._unit

    def add_item(self, name, weight, power, edit=False):
        self._loading = True
        r = self.table.rowCount()
        self.table.insertRow(r)
        self.table.setItem(r, 0, QTableWidgetItem(str(name)))
        for c, v in ((1, weight), (2, power)):
            it = QTableWidgetItem("%g" % round(float(v), 3))
            it.setTextAlignment(int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter))
            self.table.setItem(r, c, it)
        self._loading = False
        if edit:
            self.table.setCurrentCell(r, 0)
            self.table.editItem(self.table.item(r, 0))
        self._edited()

    def remove_selected(self):
        rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
        if not rows and self.table.rowCount():
            rows = [self.table.rowCount() - 1]
        for r in rows:
            self.table.removeRow(r)
        self._edited()

    @staticmethod
    def _num(item):
        try:
            return max(0.0, float((item.text() if item else "0").replace(",", ".")))
        except ValueError:
            return 0.0

    def rows(self):
        """[(name, weight_in_unit, power_w)] as entered."""
        out = []
        for r in range(self.table.rowCount()):
            name = (self.table.item(r, 0).text() if self.table.item(r, 0) else "").strip() or "Item %d" % (r + 1)
            out.append((name, self._num(self.table.item(r, 1)), self._num(self.table.item(r, 2))))
        return out

    def items(self):
        """PayloadItem list in kg for the planner."""
        return [core.PayloadItem(n, core.units.kg(w, self._unit), p) for n, w, p in self.rows()]

    def total_kg(self):
        return sum(i.mass_kg for i in self.items())

    def total_w(self):
        return sum(i.power_w for i in self.items())

    # ------------------------------------------------------- behaviour
    def _edited(self, *_):
        if self._loading:
            return
        kg, w = self.total_kg(), self.total_w()
        self.total_lbl.setText("Total: <b>%s</b>, <b>%.0f W</b> accessory power"
                               % (core.units.mass_fmt(kg, self._unit), w))
        self.save()
        self.changed.emit()

    def _unit_changed(self, *_):
        new = self.unit_cb.currentData()
        if new == self._unit:
            return
        self._loading = True
        for r in range(self.table.rowCount()):
            it = self.table.item(r, 1)
            kgv = core.units.kg(self._num(it), self._unit)
            v = kgv * core.units.LB_PER_KG if new == "lb" else kgv
            it.setText("%g" % round(v, 3))
        self._loading = False
        self._unit = new
        self._edited()

    def set_limit(self, max_payload_kg):
        """Colour the total red when over the aircraft's payload limit."""
        over = self.total_kg() > max_payload_kg + 1e-9
        self.total_lbl.setStyleSheet("color: #b91c1c" if over else "")
        if over:
            self.total_lbl.setText(self.total_lbl.text().split("<br>")[0] +
                                   "<br>Over the aircraft's %s payload limit." %
                                   core.units.mass_fmt(max_payload_kg, self._unit))

    # ------------------------------------------------------ persistence
    def save(self):
        QSettings().setValue(KEY, json.dumps({"unit": self._unit, "rows": self.rows()}))

    def load(self):
        try:
            data = json.loads(QSettings().value(KEY, "") or "{}")
        except Exception:
            data = {}
        unit = data.get("unit", "kg")
        self.unit_cb.blockSignals(True)
        self.unit_cb.setCurrentIndex(1 if unit == "lb" else 0)
        self.unit_cb.blockSignals(False)
        self._unit = unit
        for n, w, p in data.get("rows", []):
            self.add_item(n, w, p)
        self._edited()

    def set_rows(self, rows, unit="kg"):
        self._loading = True
        self.table.setRowCount(0)
        self._loading = False
        self.unit_cb.blockSignals(True)
        self.unit_cb.setCurrentIndex(1 if unit == "lb" else 0)
        self.unit_cb.blockSignals(False)
        self._unit = unit
        for n, w, p in rows:
            self.add_item(n, w, p)
        self._edited()
