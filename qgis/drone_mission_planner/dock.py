# SPDX-License-Identifier: GPL-2.0-only
"""Drone Mission Planner dock panel."""
import os
import tempfile
import traceback

from qgis.PyQt.QtCore import Qt, QTimer, QSettings
from qgis.PyQt.QtGui import QFont
from qgis.PyQt.QtWidgets import (
    QDockWidget, QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout, QPushButton,
    QComboBox, QDoubleSpinBox, QSpinBox, QCheckBox, QLabel, QPlainTextEdit, QScrollArea,
    QFileDialog, QMessageBox, QDialog, QDialogButtonBox, QLineEdit, QApplication, QGroupBox,
    QRadioButton, QButtonGroup, QToolButton, QSizePolicy)
from qgis.core import (QgsProject, QgsVectorLayer, QgsRasterLayer, QgsGeometry, QgsPointXY,
                       QgsCoordinateReferenceSystem, QgsMapLayerProxyModel, QgsWkbTypes,
                       QgsFeatureRequest, Qgis)
from qgis.gui import (QgsMapLayerComboBox, QgsProjectionSelectionWidget, QgsCollapsibleGroupBox,
                      QgsFieldComboBox)

import dronemapper_core as core
from dronemapper_core import exporters, uploader, photogrammetry as P
from . import layers as L
from .maptools import CaptureTool, PointTool

WGS84 = QgsCoordinateReferenceSystem("EPSG:4326")

try:
    F_POLY = QgsMapLayerProxyModel.Filter.PolygonLayer
    F_LINE = QgsMapLayerProxyModel.Filter.LineLayer
    F_RAST = QgsMapLayerProxyModel.Filter.RasterLayer
except AttributeError:
    F_POLY, F_LINE, F_RAST = (QgsMapLayerProxyModel.PolygonLayer,
                              QgsMapLayerProxyModel.LineLayer,
                              QgsMapLayerProxyModel.RasterLayer)

VECTOR_FILTER = ("GIS / CAD files (*.shp *.gpkg *.geojson *.json *.kml *.kmz *.gml *.dxf *.dwg "
                 "*.tab *.mif *.csv *.fgb *.sqlite *.gdb);;CAD drawings (*.dxf *.dwg);;All files (*)")


def dspin(lo, hi, val, dec=1, suffix="", step=1.0):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setSingleStep(step)
    s.setValue(val)
    if suffix:
        s.setSuffix(" " + suffix)
    s.setKeyboardTracking(False)
    return s


def _gtype(layer):
    """'polygon' | 'line' | 'point' for a vector layer (QGIS 3 and 4)."""
    t = layer.geometryType()
    name = getattr(t, "name", None) or str(t)
    name = name.lower()
    if "polygon" in name or t == 2:
        return "polygon"
    if "line" in name or t == 1:
        return "line"
    return "point"


class MissionDock(QDockWidget):
    def __init__(self, iface, parent=None):
        super().__init__("Drone Mission Planner", parent)
        self.iface = iface
        self.canvas = iface.mapCanvas()
        self.mission = None
        self.work_crs = None
        self._prev_tool = None
        self._replan_timer = QTimer(self)
        self._replan_timer.setSingleShot(True)
        self._replan_timer.setInterval(700)
        self._replan_timer.timeout.connect(lambda: self.plan(quiet=True))
        self._build_ui()
        self._load_drone(self.drone_cb.currentText())
        self._sync_from_height()
        self._update_mode()
        self._refresh_name_hint()

    # =============================================================== UI
    def _build_ui(self):
        root = QWidget()
        lay = QVBoxLayout(root)
        lay.setContentsMargins(6, 6, 6, 6)

        # ---- 1. Mission & boundary
        g = QGroupBox("1  Mission boundary")
        f = QGridLayout(g)
        self.type_cb = QComboBox()
        self.type_cb.addItems(["Area mapping (grid)", "Route / corridor mapping"])
        self.type_cb.currentIndexChanged.connect(self._update_mode)
        self.name_le = QLineEdit()
        self.name_le.setClearButtonEnabled(True)
        self.name_le.setToolTip("Shown on the field brief and report, and used for export file names. "
                                "Leave blank for an automatic name.")
        self.name_le.textChanged.connect(self._name_changed)
        self._source_name = ""
        f.addWidget(QLabel("Mission type"), 0, 0)
        f.addWidget(self.type_cb, 0, 1, 1, 2)
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("<b>Mission name</b>"))
        name_row.addWidget(self.name_le, 1)
        lay.addLayout(name_row)
        self.draw_btn = QPushButton("Draw on map")
        self.draw_btn.setToolTip("Left-click vertices, right-click or Enter to finish, "
                                 "Backspace to undo, Esc to cancel.")
        self.draw_btn.clicked.connect(self.start_draw)
        self.import_btn = QPushButton("Import GIS / CAD file…")
        self.import_btn.clicked.connect(self.import_file)
        f.addWidget(self.draw_btn, 1, 0, 1, 1)
        f.addWidget(self.import_btn, 1, 1, 1, 2)
        self.layer_cb = QgsMapLayerComboBox()
        self.layer_cb.setFilters(F_POLY)
        self.use_layer_btn = QPushButton("Use layer")
        self.use_layer_btn.setToolTip("Uses the selected features (or all features) of this layer.")
        self.use_layer_btn.clicked.connect(lambda: self.use_layer(self.layer_cb.currentLayer()))
        f.addWidget(QLabel("From layer"), 2, 0)
        f.addWidget(self.layer_cb, 2, 1)
        f.addWidget(self.use_layer_btn, 2, 2)
        self.edit_btn = QPushButton("Edit boundary vertices")
        self.edit_btn.setCheckable(True)
        self.edit_btn.toggled.connect(self.toggle_edit_boundary)
        f.addWidget(self.edit_btn, 3, 0, 1, 2)
        self.pick_lines_btn = QPushButton("Pick line features on map")
        self.pick_lines_btn.setToolTip("Click road / pipeline / powerline features in any visible line "
                                       "layer (click again to drop one). Right-click or Enter to use "
                                       "them - pieces are joined into one route automatically.")
        self.pick_lines_btn.clicked.connect(self.start_pick_lines)
        f.addWidget(self.pick_lines_btn, 4, 0, 1, 3)
        lay.addWidget(g)

        # ---- 1b. Linear feature / corridor (route mode)
        self.lin_box = QgsCollapsibleGroupBox("Linear feature && corridor")
        f = QFormLayout(self.lin_box)
        self.feat_lbl = QLabel("–")
        f.addRow("Feature length", self.feat_lbl)
        self.left_w = dspin(0, 5000, 30, 1, "m", 5)
        self.right_w = dspin(0, 5000, 30, 1, "m", 5)
        row = QHBoxLayout(); row.addWidget(QLabel("L")); row.addWidget(self.left_w)
        row.addWidget(QLabel("R")); row.addWidget(self.right_w)
        f.addRow("Width each side", row)
        self.corr_n = QSpinBox()
        self.corr_n.setRange(0, 50)
        self.corr_n.setSpecialValueText("auto")
        self.corr_n.setSuffix(" lines")
        f.addRow("Flight lines", self.corr_n)
        self.kp_from = dspin(0, 100000, 0, 3, "km", 0.1)
        self.kp_to = dspin(0, 100000, 0, 3, "km", 0.1)
        self.kp_to.setSpecialValueText("end")
        row = QHBoxLayout(); row.addWidget(self.kp_from); row.addWidget(QLabel("to")); row.addWidget(self.kp_to)
        f.addRow("Map chainage (KP)", row)
        self.rev_kp = QCheckBox("Reverse KP direction (KP 0 at the other end)")
        f.addRow(self.rev_kp)
        self.simplify = dspin(0, 100, 2, 1, "m", 1)
        self.simplify.setToolTip("Removes digitising noise so the drone flies smooth lines "
                                 "(fewer waypoints). 0 = keep every vertex.")
        f.addRow("Simplify line", self.simplify)
        self.sec_mode = QComboBox()
        self.sec_mode.addItems(["Auto (battery && range)", "Fixed length", "None – full length"])
        self.sec_len = dspin(0.2, 100, 2.0, 2, "km", 0.5)
        self.sec_len.setEnabled(False)
        self.sec_mode.currentIndexChanged.connect(lambda i: self.sec_len.setEnabled(i == 1))
        row = QHBoxLayout(); row.addWidget(self.sec_mode); row.addWidget(self.sec_len)
        f.addRow("Sections", row)
        self.kp_int = dspin(0, 10000, 100, 0, "m", 50)
        self.kp_int.setSpecialValueText("off")
        f.addRow("Chainage markers every", self.kp_int)
        hint = QLabel("Long routes are flown section by section; each section's lines are flown "
                      "back and forth before moving on, so batteries and launch stations line up "
                      "along the route.")
        hint.setWordWrap(True); hint.setStyleSheet("color: gray")
        f.addRow(hint)
        lay.addWidget(self.lin_box)

        # ---- 2. Launch stations, landing & range
        g = QGroupBox("2  Launch stations, landing && range")
        f = QGridLayout(g)
        self.tk_btn = QPushButton("Pick take-off (S1)")
        self.tk_btn.clicked.connect(lambda: self.start_pick("takeoff"))
        self.st_btn = QPushButton("Add launch station")
        self.st_btn.setToolTip("Extra places to launch from for battery changes on big sites / long "
                               "routes. Click once per station; each flight uses the nearest one.")
        self.st_btn.clicked.connect(lambda: self.start_pick("station"))
        self.clear_st_btn = QPushButton("Clear stations")
        self.clear_st_btn.clicked.connect(self.clear_stations)
        self.auto_st_btn = QPushButton("Auto-place stations")
        self.auto_st_btn.setToolTip("Area: a grid so every point is within the range limit. Route: one "
                                    "station per section. Then drag them onto roads / access points "
                                    "with the vertex tool on the 'Take-off & landing' layer.")
        self.auto_st_btn.clicked.connect(self.auto_stations)
        self.st_side = dspin(-2000, 2000, 0, 0, "m off line", 10)
        self.st_side.setToolTip("Route: place stations this far to the left (+) / right (-) of the line.")
        self.ld_btn = QPushButton("Pick final landing")
        self.ld_btn.clicked.connect(lambda: self.start_pick("landing"))
        self.same_ld = QCheckBox("Last flight lands at its launch station")
        self.same_ld.setChecked(True)
        self.same_ld.toggled.connect(lambda c: (self.ld_btn.setEnabled(not c), self._param_changed()))
        self.ld_btn.setEnabled(False)
        self.range_cb = QCheckBox("Max distance from pilot (VLOS)")
        self.range_m = dspin(100, 20000, 1000, 0, "m", 100)
        self.range_m.setEnabled(False)
        self.range_cb.toggled.connect(lambda c: (self.range_m.setEnabled(c), self._param_changed()))
        self.leapfrog = QCheckBox("Leapfrog – may land at the next station (crew moves during flight)")
        self.site_lbl = QLabel("Not set – defaults to the boundary corner nearest the map centre.")
        self.site_lbl.setWordWrap(True)
        f.addWidget(self.tk_btn, 0, 0)
        f.addWidget(self.st_btn, 0, 1)
        f.addWidget(self.auto_st_btn, 1, 0)
        f.addWidget(self.st_side, 1, 1)
        f.addWidget(self.clear_st_btn, 2, 0)
        f.addWidget(self.ld_btn, 2, 1)
        f.addWidget(self.same_ld, 3, 0, 1, 2)
        f.addWidget(self.range_cb, 4, 0)
        f.addWidget(self.range_m, 4, 1)
        f.addWidget(self.leapfrog, 5, 0, 1, 2)
        f.addWidget(self.site_lbl, 6, 0, 1, 2)
        lay.addWidget(g)

        # ---- 3. Coordinate system
        g = QgsCollapsibleGroupBox("3  Coordinate system")
        f = QFormLayout(g)
        self.auto_utm = QCheckBox("Automatic UTM zone (recommended)")
        self.auto_utm.setChecked(True)
        self.crs_w = QgsProjectionSelectionWidget()
        self.crs_w.setCrs(QgsCoordinateReferenceSystem("EPSG:3857"))
        self.crs_w.setEnabled(False)
        self.auto_utm.toggled.connect(lambda c: (self.crs_w.setEnabled(not c), self._param_changed()))
        self.crs_w.crsChanged.connect(self._param_changed)
        f.addRow(self.auto_utm)
        f.addRow("Planning / output CRS", self.crs_w)
        hint = QLabel("Must be a projected CRS in metres (e.g. MGA2020 / GDA2020 zone, State "
                      "Plane, ETRS89/UTM). Exports are always converted to WGS84 for the drone.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: gray")
        f.addRow(hint)
        lay.addWidget(g)

        # ---- 4. Aircraft & camera
        g = QgsCollapsibleGroupBox("4  Aircraft && camera")
        f = QFormLayout(g)
        self.drone_cb = QComboBox()
        self.drone_cb.addItems(core.drone_names())
        self.drone_cb.currentTextChanged.connect(self._load_drone)
        f.addRow("Aircraft", self.drone_cb)
        self.sw = dspin(1, 100, 13.2, 2, "mm", 0.1)
        self.sh = dspin(1, 100, 8.8, 2, "mm", 0.1)
        self.fl = dspin(1, 500, 8.8, 2, "mm", 0.1)
        self.iw = QSpinBox(); self.iw.setRange(320, 30000); self.iw.setSuffix(" px")
        self.ih = QSpinBox(); self.ih.setRange(240, 30000); self.ih.setSuffix(" px")
        self.interval = dspin(0.1, 30, 2.0, 1, "s", 0.1)
        self.mass = dspin(0.1, 50, 1.0, 3, "kg", 0.1)
        self.maxpay = dspin(0, 30, 0.0, 2, "kg", 0.1)
        self.rated = dspin(1, 600, 30, 0, "min")
        self.vmax = dspin(1, 60, 15, 1, "m/s")
        self.maxwind = dspin(1, 80, 23, 0, "kn")
        self._wunit = "kn"
        self.batt_wh = dspin(0, 20000, 77, 0, "Wh", 10)
        self.batt_wh.setToolTip("Total energy of the battery set for one flight (both packs on dual-"
                                "battery aircraft). Needed to account for accessory power.")
        self.enum = QLineEdit()
        self.enum.setToolTip("DJI WPML droneEnumValue,payloadEnumValue (e.g. 77,66 for Mavic 3E). "
                             "Pilot 2 refuses routes made for a different aircraft.")
        row = QHBoxLayout(); row.addWidget(self.sw); row.addWidget(QLabel("×")); row.addWidget(self.sh)
        f.addRow("Sensor W × H", row)
        f.addRow("Focal length", self.fl)
        row = QHBoxLayout(); row.addWidget(self.iw); row.addWidget(QLabel("×")); row.addWidget(self.ih)
        f.addRow("Image W × H", row)
        f.addRow("Min photo interval", self.interval)
        f.addRow("Take-off mass (no payload)", self.mass)
        f.addRow("Max payload", self.maxpay)
        f.addRow("Rated flight time", self.rated)
        f.addRow("Max speed", self.vmax)
        f.addRow("Max wind", self.maxwind)
        f.addRow("Battery capacity", self.batt_wh)
        f.addRow("DJI enum (drone,payload)", self.enum)
        g.setCollapsed(True)
        lay.addWidget(g)

        # ---- 5. Flight parameters
        g = QGroupBox("5  Flight parameters")
        f = QFormLayout(g)
        self.unit_cb = QComboBox()
        self.unit_cb.addItem("Metres (m)", "m")
        self.unit_cb.addItem("Feet (ft)", "ft")
        self.unit_cb.setToolTip("Units for all heights and elevations. Mission files are always "
                                "written in metres, which is what the aircraft expect.")
        f.addRow("Height units", self.unit_cb)
        self.height = dspin(5, 1500, 100, 1, "m AGL", 5)
        self.gsd = dspin(0.05, 100, 2.5, 2, "cm/px", 0.1)
        self.height.valueChanged.connect(self._sync_from_height)
        self.gsd.valueChanged.connect(self._sync_from_gsd)
        f.addRow("Flying height", self.height)
        self.rth_h = dspin(0, 1500, 0, 0, "m above take-off", 5)
        self.rth_h.setSpecialValueText("same as flying height")
        self.rth_h.setToolTip("Height for the final flight home to the take-off / landing point. "
                              "Also set as the autopilot's failsafe RTL altitude on MAVLink upload.")
        self.batt_h = dspin(0, 1500, 0, 0, "m above take-off", 5)
        self.batt_h.setSpecialValueText("same as return height")
        self.batt_h.setToolTip("Height for returning to change a battery and for flying back out "
                               "to resume the survey.")
        f.addRow("Ground pixel size (GSD)", self.gsd)
        f.addRow("Return-to-take-off height", self.rth_h)
        f.addRow("Battery-change height", self.batt_h)
        self.min_agl = dspin(0, 1500, 0, 0, "m above ground", 5)
        self.min_agl.setSpecialValueText("off")
        self.min_agl.setToolTip("Lowest height above ground allowed anywhere - set it above tree canopy, "
                                "powerlines and buildings. The flying height is raised to it if lower, and "
                                "with a DEM it is also kept where the ground rises. Transits, battery-change "
                                "returns and the return home obey it too.")
        f.addRow("Minimum height above ground", self.min_agl)
        self.front = dspin(10, 95, 75, 0, "%")
        self.side = dspin(10, 95, 65, 0, "%")
        f.addRow("Front overlap", self.front)
        f.addRow("Side overlap", self.side)
        self.head_mode = QComboBox()
        self.head_mode.addItems(["Auto – fewest lines", "Manual", "Align with wind"])
        self.head = dspin(0, 359.9, 0, 1, "°", 5)
        row = QHBoxLayout(); row.addWidget(self.head_mode); row.addWidget(self.head)
        f.addRow("Line heading", row)
        self.speed_auto = QCheckBox("Optimise")
        self.speed_auto.setChecked(True)
        self.speed = dspin(1, 60, 8, 1, "m/s", 0.5)
        self.speed.setEnabled(False)
        self.speed_auto.toggled.connect(lambda c: self.speed.setEnabled(not c))
        row = QHBoxLayout(); row.addWidget(self.speed_auto); row.addWidget(self.speed)
        f.addRow("Speed", row)
        self.crosshatch = QCheckBox("Double grid (crosshatch) for 3D models")
        f.addRow(self.crosshatch)
        self.margin = dspin(-200, 500, 0, 0, "m", 5)
        self.overshoot = dspin(0, 300, 10, 0, "m", 5)
        f.addRow("Extend coverage beyond boundary", self.margin)
        f.addRow("Line run-in / run-out", self.overshoot)
        self.pitch = dspin(-90, 0, -90, 0, "°", 5)
        f.addRow("Gimbal pitch", self.pitch)
        self.shutter = QComboBox()
        for s in ["1/500", "1/800", "1/1000", "1/1600", "1/2000", "1/3200"]:
            self.shutter.addItem(s, 1.0 / float(s.split("/")[1]))
        self.shutter.setCurrentIndex(2)
        f.addRow("Shutter (motion blur limit)", self.shutter)
        self.trigger_cb = QComboBox()
        self.trigger_cb.addItem("Every N m (autopilot / Litchi / Pilot 2)", "distance")
        self.trigger_cb.addItem("Camera Timed shot (DJI Fly)", "timed")
        self.trigger_cb.addItem("Stop, hover && shoot at each photo point", "stop")
        self.trigger_cb.setToolTip("How photos are triggered. DJI Fly (e.g. Mini 5 Pro) cannot trigger by "
                                   "distance: use the camera's Timed shot (speed is matched to it) or "
                                   "stop-and-shoot (slower, one waypoint per photo).")
        self.stop_hover = dspin(1, 10, 1, 1, "s hover", 0.5)
        row = QHBoxLayout(); row.addWidget(self.trigger_cb, 1); row.addWidget(self.stop_hover)
        f.addRow("Photo trigger", row)
        self.trigger_cb.currentIndexChanged.connect(
            lambda *_: (self.stop_hover.setEnabled(self.trigger_cb.currentData() == "stop"), self._param_changed()))
        self.stop_hover.valueChanged.connect(self._param_changed)
        self.stop_hover.setEnabled(False)
        self.portrait = QCheckBox("Camera in portrait (short side across track)")
        f.addRow(self.portrait)
        lay.addWidget(g)

        # ---- 6. Conditions
        g = QGroupBox("6  Conditions (flight-time optimisation)")
        f = QFormLayout(g)
        self.wind = dspin(0, 80, 0, 0, "kn", 1)
        self.wdir = dspin(0, 359, 0, 0, "° from", 10)
        self.wind_unit_cb = QComboBox()
        for u in ("kn", "m/s", "km/h"):
            self.wind_unit_cb.addItem(u, u)
        self.wind_unit_cb.setToolTip("Wind speed unit (also used for the aircraft's max wind). "
                                     "Calculations use m/s internally.")
        row = QHBoxLayout(); row.addWidget(self.wind, 2); row.addWidget(self.wind_unit_cb, 1); row.addWidget(self.wdir, 2)
        f.addRow("Wind speed / direction", row)
        self.temp = dspin(-40, 55, 20, 0, "°C")
        f.addRow("Air temperature", self.temp)
        self.elev = dspin(-450, 6000, 0, 0, "m AMSL", 10)
        self.elev_btn = QToolButton(); self.elev_btn.setText("From DEM")
        self.elev_btn.clicked.connect(self.elev_from_dem)
        row = QHBoxLayout(); row.addWidget(self.elev); row.addWidget(self.elev_btn)
        f.addRow("Take-off elevation", row)
        from .payload_widget import PayloadTable
        self.payload_tbl = PayloadTable()
        self.payload_tbl.changed.connect(self._param_changed)
        f.addRow(QLabel("Payload carried (weight and power both reduce flight time):"))
        f.addRow(self.payload_tbl)
        self.reserve = dspin(5, 60, 25, 0, "%")
        f.addRow("Battery reserve", self.reserve)
        self.health = dspin(30, 100, 100, 0, "%")
        f.addRow("Battery health", self.health)
        self.dem_cb = QgsMapLayerComboBox()
        self.dem_cb.setFilters(F_RAST)
        self.dem_cb.setAllowEmptyLayer(True)
        self.dem_cb.setLayer(None)
        self.dem_cb.layerChanged.connect(self._dem_changed)
        f.addRow("DEM (optional)", self.dem_cb)
        self.dem_src = QComboBox()
        for k, label in core.dem.SOURCES.items():
            self.dem_src.addItem(label, k)
        self.dem_dl = QPushButton("Download DEM")
        self.dem_dl.setToolTip("Downloads free elevation data for the mission area (plus a margin) "
                               "and loads it here. Copernicus GLO-30 is a surface model (includes "
                               "trees/buildings - safest for clearance checks).")
        self.dem_dl.clicked.connect(self.download_dem)
        row = QHBoxLayout(); row.addWidget(self.dem_src, 1); row.addWidget(self.dem_dl)
        f.addRow("Get a DEM", row)
        self.terrain = QCheckBox("Terrain follow (constant height above ground)")
        f.addRow(self.terrain)
        self.tcheck = QCheckBox("Check terrain clearance on every flight path")
        self.tcheck.setChecked(True)
        self.tclear = dspin(5, 500, 30, 0, "m min", 5)
        self.tagl = dspin(10, 1000, 120, 0, "m AGL max", 5)
        row = QHBoxLayout(); row.addWidget(self.tclear); row.addWidget(self.tagl)
        f.addRow(self.tcheck)
        f.addRow("Clearance / height limit", row)
        lay.addWidget(g)

        # ---- Obstacles (powerlines, buildings, trees)
        g = QgsCollapsibleGroupBox("Obstacles (powerlines, buildings, trees)")
        f = QFormLayout(g)
        self.obs_cb = QgsMapLayerComboBox()
        try:
            self.obs_cb.setFilters(QgsMapLayerProxyModel.Filter.VectorLayer)
        except AttributeError:
            self.obs_cb.setFilters(QgsMapLayerProxyModel.VectorLayer)
        self.obs_cb.setAllowEmptyLayer(True)
        self.obs_cb.setLayer(None)
        self.obs_cb.setToolTip("Any point, line or polygon layer: powerlines, towers, masts, building "
                               "footprints, tree stands. Selected features only, if some are selected.")
        f.addRow("Obstacle layer", self.obs_cb)
        self.obs_field = QgsFieldComboBox()
        self.obs_field.setAllowEmptyFieldName(True)
        try:
            from qgis.core import QgsFieldProxyModel
            self.obs_field.setFilters(QgsFieldProxyModel.Filter.Numeric if hasattr(QgsFieldProxyModel, "Filter")
                                      else QgsFieldProxyModel.Numeric)
        except Exception:
            pass
        self.obs_field.setToolTip("Numeric field with each obstacle's height above ground (in the height "
                                  "units above). Blank = use the default height for every feature.")
        self.obs_cb.layerChanged.connect(self._obstacle_layer_changed)
        f.addRow("Height field", self.obs_field)
        self.obs_default = dspin(0, 1000, 25, 0, "m", 1)
        self.obs_margin = dspin(0, 200, 10, 0, "m above top", 1)
        self.obs_buffer = dspin(0, 500, 15, 0, "m sideways", 1)
        f.addRow("Default height", self.obs_default)
        f.addRow("Vertical margin", self.obs_margin)
        f.addRow("Horizontal buffer", self.obs_buffer)
        self.obs_lbl = QLabel("")
        self.obs_lbl.setWordWrap(True)
        self.obs_lbl.setStyleSheet("color: gray")
        f.addRow(self.obs_lbl)
        lay.addWidget(g)

        # ---- Battery changes & field logistics
        g = QgsCollapsibleGroupBox("Battery changes && field schedule")
        f = QFormLayout(g)
        self.split = QCheckBox("Split into flights per battery")
        self.split.setChecked(True)
        self.midline = QCheckBox("Change battery mid-line (resume where it stopped)")
        self.midline.setChecked(True)
        self.midline.setToolTip("Uses each battery fully: the flight stops part-way along a line and "
                                "the next one resumes there with one photo of overlap.")
        f.addRow(self.split)
        f.addRow(self.midline)
        self.sets = QSpinBox(); self.sets.setRange(1, 60); self.sets.setValue(4); self.sets.setSuffix(" sets")
        self.chargers = QSpinBox(); self.chargers.setRange(1, 20); self.chargers.setValue(2)
        self.charge_min = dspin(5, 300, 60, 0, "min", 5)
        self.swap_min = dspin(0, 60, 4, 0, "min")
        self.setup_min = dspin(0, 180, 15, 0, "min", 5)
        self.drive_kmh = dspin(1, 120, 30, 0, "km/h", 5)
        self.reloc_min = dspin(0, 120, 8, 0, "min")
        from qgis.PyQt.QtWidgets import QTimeEdit
        from qgis.PyQt.QtCore import QTime
        self.start_t = QTimeEdit(QTime(8, 0)); self.start_t.setDisplayFormat("HH:mm")
        f.addRow("Charged battery sets", self.sets)
        f.addRow("Chargers (sets at once)", self.chargers)
        f.addRow("Charge time per set", self.charge_min)
        f.addRow("Battery swap + checks", self.swap_min)
        f.addRow("Site setup", self.setup_min)
        f.addRow("Drive between stations", self.drive_kmh)
        f.addRow("Pack / set-up per move", self.reloc_min)
        f.addRow("Arrive on site", self.start_t)
        g.setCollapsed(True)
        lay.addWidget(g)

        # ---- Plan
        row = QHBoxLayout()
        self.plan_btn = QPushButton("Plan mission")
        self.plan_btn.setStyleSheet("font-weight: bold; padding: 6px")
        self.plan_btn.clicked.connect(lambda: self.plan(quiet=False))
        self.auto_cb = QCheckBox("Live update")
        self.auto_cb.setChecked(True)
        self.auto_cb.setToolTip("Re-plan automatically when a parameter or the boundary changes.")
        row.addWidget(self.plan_btn, 2)
        row.addWidget(self.auto_cb, 1)
        lay.addLayout(row)
        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        self.report.setMinimumHeight(190)
        self.report.setFont(QFont("Consolas", 8))
        lay.addWidget(self.report)
        from .profile_widget import ProfileChart
        self.profile = ProfileChart()
        self.profile.setVisible(False)
        lay.addWidget(self.profile)

        # ---- 7. Export / upload
        g = QGroupBox("7  Export && upload")
        f = QFormLayout(g)
        self.fmt_cb = QComboBox()
        for k, (label, ext) in exporters.FORMATS.items():
            self.fmt_cb.addItem(label, k)
        f.addRow("Format", self.fmt_cb)
        self.fw_cb = QComboBox(); self.fw_cb.addItems(["PX4", "ArduPilot"])
        f.addRow("QGC firmware", self.fw_cb)
        self.per_flight = QCheckBox("One file per flight/battery")
        self.per_flight.setChecked(True)
        f.addRow(self.per_flight)
        self.brief_bg = QComboBox()
        for label, key in (("Current map – visible layers (OpenStreetMap if none)", "canvas"),
                           ("OpenStreetMap", "osm"), ("Esri World Imagery", "esri_imagery"),
                           ("Esri World Topographic", "esri_topo"), ("None – plain map", "none")):
            self.brief_bg.addItem(label, key)
        self.brief_bg.setCurrentIndex(max(0, self.brief_bg.findData(QSettings().value("dmp/brief_basemap", "canvas"))))
        self.brief_bg.currentIndexChanged.connect(
            lambda *_: QSettings().setValue("dmp/brief_basemap", self.brief_bg.currentData()))
        self.brief_bg.setToolTip("Basemap under the field brief's maps for context. 'Current map' renders "
                                 "the layers visible in QGIS (your basemap, imagery, cadastre…); online "
                                 "basemaps are downloaded as tiles with their attribution.")
        f.addRow("Brief basemap", self.brief_bg)
        row = QHBoxLayout()
        b = QPushButton("Export…"); b.clicked.connect(self.export)
        u = QPushButton("Upload to drone…"); u.clicked.connect(self.upload)
        row.addWidget(b); row.addWidget(u)
        f.addRow(row)
        lay.addWidget(g)
        lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(root)
        self.setWidget(scroll)
        self.setMinimumWidth(360)

        # height-unit machinery: (spin, min_m, max_m, suffix template)
        self._hspins = [(self.height, 5, 1500, "{u} AGL"), (self.rth_h, 0, 1500, "{u} above take-off"),
                        (self.batt_h, 0, 1500, "{u} above take-off"), (self.elev, -450, 6000, "{u} AMSL"),
                        (self.tclear, 5, 500, "{u} min"), (self.tagl, 10, 1000, "{u} AGL max"),
                        (self.min_agl, 0, 1500, "{u} above ground"), (self.obs_default, 0, 1000, "{u}"),
                        (self.obs_margin, 0, 200, "{u} above top"), (self.obs_buffer, 0, 500, "{u} sideways")]
        for w in (self.min_agl, self.obs_default, self.obs_margin, self.obs_buffer):
            w.valueChanged.connect(self._param_changed)
        self.obs_field.fieldChanged.connect(self._param_changed)
        self._unit = "m"
        self.unit_cb.currentIndexChanged.connect(self._unit_changed)
        self.rth_h.valueChanged.connect(self._param_changed)
        self.batt_h.valueChanged.connect(self._param_changed)
        self._wunit = "kn"
        self.wind_unit_cb.currentIndexChanged.connect(self._wind_unit_changed)
        wsaved = QSettings().value("dmp/wind_unit", "kn")
        if wsaved in ("m/s", "km/h"):
            self.wind_unit_cb.setCurrentIndex(self.wind_unit_cb.findData(wsaved))
        saved = QSettings().value("dmp/height_unit", "m")
        if saved == "ft":
            self.unit_cb.setCurrentIndex(1)

        # live update wiring
        for w in (self.height, self.front, self.side, self.head, self.speed, self.margin,
                  self.overshoot, self.pitch, self.wind, self.wdir, self.temp, self.elev,
                  self.reserve, self.health, self.left_w, self.right_w, self.sw, self.sh,
                  self.kp_from, self.kp_to, self.simplify, self.sec_len, self.kp_int, self.range_m,
                  self.charge_min, self.swap_min, self.setup_min, self.drive_kmh, self.reloc_min,
                  self.fl, self.interval, self.mass, self.maxpay, self.rated, self.vmax, self.batt_wh,
                  self.maxwind):
            w.valueChanged.connect(self._param_changed)
        for w in (self.iw, self.ih, self.corr_n, self.sets, self.chargers):
            w.valueChanged.connect(self._param_changed)
        for w in (self.tclear, self.tagl):
            w.valueChanged.connect(self._param_changed)
        for w in (self.crosshatch, self.portrait, self.split, self.terrain, self.speed_auto, self.tcheck,
                  self.midline, self.leapfrog, self.rev_kp):
            w.toggled.connect(self._param_changed)
        self.start_t.timeChanged.connect(self._param_changed)
        for w in (self.head_mode, self.shutter, self.sec_mode):
            w.currentIndexChanged.connect(self._param_changed)
        self.sw.valueChanged.connect(self._sync_from_height)
        self.fl.valueChanged.connect(self._sync_from_height)
        self.iw.valueChanged.connect(self._sync_from_height)
        self.portrait.toggled.connect(self._sync_from_height)

    # ======================================================== obstacles
    def _obstacle_layer_changed(self, lyr):
        self.obs_field.setLayer(lyr)
        guess = ""
        if lyr is not None:
            for fld in lyr.fields():
                if fld.isNumeric() and fld.name().lower() in ("height", "height_m", "hgt", "ht", "elev_top",
                                                            "obs_height", "agl", "tree_ht", "bldg_ht"):
                    guess = fld.name()
                    break
        self.obs_field.setField(guess)
        self._param_changed()

    def _obstacles(self, work, ext_box):
        """Obstacles from the chosen layer, in the planning CRS, near the mission."""
        lyr = self.obs_cb.currentLayer()
        self.obs_lbl.setText("")
        if not isinstance(lyr, QgsVectorLayer) or not lyr.isValid():
            return []
        xf = L.transform(lyr.crs(), work)
        back = L.transform(work, lyr.crs())
        from qgis.core import QgsRectangle
        rect = back.transformBoundingBox(QgsRectangle(ext_box[0][0], ext_box[0][1], ext_box[1][0], ext_box[1][1]))
        req = QgsFeatureRequest().setFilterRect(rect)
        feats = lyr.getSelectedFeatures(req) if lyr.selectedFeatureCount() else lyr.getFeatures(req)
        fname = self.obs_field.currentField()
        default_h = self._m(self.obs_default)
        out, missing = [], 0
        for ft in feats:
            if not ft.hasGeometry():
                continue
            h = None
            if fname:
                v = ft[fname]
                try:
                    h = core.units.to_m(float(v), self._unit) if v not in (None, "") else None
                except (TypeError, ValueError):
                    h = None
            if h is None:
                missing += 1 if fname else 0
                h = default_h
            if not h or h <= 0:
                continue
            g = QgsGeometry(ft.geometry())
            g.transform(xf)
            for part in (g.asGeometryCollection() if g.isMultipart() else [g]):
                gt = _gtype_geom(part)
                name = "%s #%s" % (lyr.name(), ft.id())
                if gt == "point":
                    p = part.asPoint()
                    out.append(core.Obstacle("point", [(p.x(), p.y())], h, name))
                elif gt == "line":
                    out.append(core.Obstacle("line", [(p.x(), p.y()) for p in part.asPolyline()], h, name))
                else:
                    out.append(core.Obstacle("polygon", [[(p.x(), p.y()) for p in r] for r in part.asPolygon()],
                                             h, name))
        txt = "%d obstacle feature(s) near the mission" % len(out)
        if out:
            txt += ", tallest %s" % core.units.fmt(max(o.height_m for o in out), self._unit, 0, True)
        if missing:
            txt += "; %d without a height used the default" % missing
        self.obs_lbl.setText(txt + ".")
        return out

    # ============================================================ name
    def mission_name(self):
        typed = self.name_le.text().strip()
        if typed:
            return typed
        return core.default_mission_name(core.ROUTE if self.is_route else core.AREA, self._source_name)

    def _name_changed(self, *_):
        self._refresh_name_hint()
        if self.mission is not None:
            self.mission.params.mission_name = self.mission_name()
            self.report.setPlainText(self.mission.report())

    def _refresh_name_hint(self):
        self.name_le.setPlaceholderText(core.default_mission_name(
            core.ROUTE if self.is_route else core.AREA, self._source_name))

    # ============================================================ units
    def _m(self, spin):
        """Spin box value in metres."""
        return core.units.to_m(spin.value(), self._unit)

    def _set_m(self, spin, metres):
        spin.setValue(core.units.from_m(metres, self._unit))

    def _wind_ms(self, spin):
        return core.units.wind_to_ms(spin.value(), self._wunit)

    def _set_wind_ms(self, spin, ms):
        spin.setValue(core.units.wind_from_ms(ms, self._wunit))

    def _wind_unit_changed(self, *_):
        new = self.wind_unit_cb.currentData()
        old = self._wunit
        if new == old:
            return
        for spin, hi_ms in ((self.wind, 41.0), (self.maxwind, 41.0)):
            ms = core.units.wind_to_ms(spin.value(), old)
            spin.blockSignals(True)
            spin.setDecimals(1 if new == "m/s" else 0)
            spin.setSingleStep(0.5 if new == "m/s" else 1)
            spin.setRange(0, core.units.wind_from_ms(hi_ms, new))
            spin.setSuffix(" " + new)
            spin.setValue(core.units.wind_from_ms(ms, new))
            spin.blockSignals(False)
        self._wunit = new
        QSettings().setValue("dmp/wind_unit", new)
        if self.mission is not None:
            self.plan(quiet=True)

    def _unit_changed(self, *_):
        new = self.unit_cb.currentData()
        old = self._unit
        if new == old:
            return
        for spin, lo, hi, tpl in self._hspins:
            metres = core.units.to_m(spin.value(), old)
            spin.blockSignals(True)
            spin.setRange(core.units.from_m(lo, new), core.units.from_m(hi, new))
            spin.setSuffix(" " + tpl.format(u=new))
            spin.setSingleStep(10 if new == "ft" else 5)
            spin.setValue(core.units.from_m(metres, new))
            spin.blockSignals(False)
        self._unit = new
        QSettings().setValue("dmp/height_unit", new)
        if self.mission is not None:
            self.plan(quiet=True)

    # =========================================================== helpers
    @property
    def is_route(self):
        return self.type_cb.currentIndex() == 1

    def _update_mode(self):
        r = self.is_route
        self.layer_cb.setFilters(F_LINE if r else F_POLY)
        self.draw_btn.setText("Draw route line" if r else "Draw polygon")
        self.lin_box.setVisible(r)
        self.pick_lines_btn.setVisible(r)
        self.st_side.setEnabled(r)
        for w in (self.head_mode, self.head, self.crosshatch, self.margin):
            w.setEnabled(not r)
        self._param_changed()

    def _load_drone(self, name):
        d = core.get_drone(name)
        c = d.camera
        widgets = [self.sw, self.sh, self.fl, self.iw, self.ih, self.interval, self.mass,
                   self.maxpay, self.rated, self.vmax, self.maxwind, self.batt_wh]
        for w in widgets:
            w.blockSignals(True)
        self.sw.setValue(c.sensor_width_mm); self.sh.setValue(c.sensor_height_mm)
        self.fl.setValue(c.focal_length_mm); self.iw.setValue(c.image_width_px)
        self.ih.setValue(c.image_height_px); self.interval.setValue(c.min_trigger_interval_s)
        self.mass.setValue(d.empty_mass_kg); self.maxpay.setValue(d.max_payload_kg)
        self.rated.setValue(d.rated_flight_time_min); self.vmax.setValue(d.max_speed_ms)
        self._set_wind_ms(self.maxwind, d.max_wind_ms)
        self.batt_wh.setValue(d.battery_wh)
        self.enum.setText(d.drone_enum)
        for w in widgets:
            w.blockSignals(False)
        default = exporters.DEFAULT_FORMAT_FOR_PLATFORM.get(d.platform, "kml")
        if hasattr(self, "trigger_cb"):
            self.trigger_cb.setCurrentIndex(max(0, self.trigger_cb.findData(d.photo_trigger)))
        if getattr(d, "notes", "") and hasattr(self, "iface"):
            self.msg(d.notes, dur=10)
        self.fmt_cb.setCurrentIndex(self.fmt_cb.findData(default))
        self._sync_from_height()
        self._param_changed()

    def drone(self):
        d = core.get_drone(self.drone_cb.currentText())
        c = d.camera
        c.sensor_width_mm, c.sensor_height_mm = self.sw.value(), self.sh.value()
        c.focal_length_mm = self.fl.value()
        c.image_width_px, c.image_height_px = self.iw.value(), self.ih.value()
        c.min_trigger_interval_s = self.interval.value()
        d.empty_mass_kg, d.max_payload_kg = self.mass.value(), self.maxpay.value()
        d.rated_flight_time_min, d.max_speed_ms = self.rated.value(), self.vmax.value()
        d.max_wind_ms = self._wind_ms(self.maxwind)
        d.battery_wh = self.batt_wh.value()
        d.drone_enum = self.enum.text().strip()
        return d

    def _sync_from_height(self, *_):
        c = self.drone().camera
        self.gsd.blockSignals(True)
        self.gsd.setValue(P.gsd_from_height(c, self._m(self.height)))
        self.gsd.blockSignals(False)

    def _sync_from_gsd(self, *_):
        c = self.drone().camera
        self.height.blockSignals(True)
        self._set_m(self.height, P.height_from_gsd(c, self.gsd.value()))
        self.height.blockSignals(False)
        self._param_changed()

    def _has_input(self):
        lyr = L.find(L.CENTRELINE if self.is_route else L.BOUNDARY)
        return lyr is not None and lyr.featureCount() > 0

    def _param_changed(self, *_):
        if self.auto_cb.isChecked() and (self.mission is not None or self._has_input()):
            self._replan_timer.start()

    def msg(self, text, level=None, dur=6):
        lvl = level if level is not None else Qgis.MessageLevel.Info if hasattr(Qgis, "MessageLevel") else Qgis.Info
        self.iface.messageBar().pushMessage("Drone Mission Planner", text, lvl, dur)

    def _warn(self, text):
        lvl = Qgis.MessageLevel.Warning if hasattr(Qgis, "MessageLevel") else Qgis.Warning
        self.msg(text, lvl, 8)

    # ============================================================ input
    def _set_tool(self, tool):
        self._prev_tool = self.canvas.mapTool()
        self._tool = tool
        self.canvas.setMapTool(tool)

    def _restore_tool(self):
        tool = getattr(self, "_tool", None)
        if self._prev_tool is not None and self._prev_tool is not tool:
            self.canvas.setMapTool(self._prev_tool)
        elif tool is not None:
            self.canvas.unsetMapTool(tool)
        self._tool = None

    def start_draw(self):
        tool = CaptureTool(self.canvas, polygon=not self.is_route)
        tool.finished.connect(self._drawn)
        tool.cancelled.connect(self._restore_tool)
        self._set_tool(tool)
        self.msg("Click to add vertices; right-click or Enter to finish; Backspace undoes.", dur=8)

    def _drawn(self, geom):
        self._restore_tool()
        self._source_name = ""
        self.set_boundary(geom, self.canvas.mapSettings().destinationCrs())

    def import_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import boundary / route (GIS or CAD)",
                                              QSettings().value("dmp/lastdir", ""), VECTOR_FILTER)
        if not path:
            return
        QSettings().setValue("dmp/lastdir", os.path.dirname(path))
        want = "line" if self.is_route else "polygon"
        lyr = None
        # multi-layer sources (DXF entities, GPKG, KML folders): pick a sub-layer of the right type
        probe = QgsVectorLayer(path, "probe", "ogr")
        subs = probe.dataProvider().subLayers() if probe.isValid() else []
        cands = []
        for s in subs:
            parts = s.split("!!::!!")
            if len(parts) >= 2:
                uri = "%s|layername=%s" % (path, parts[1])
                cands.append(uri)
        cands = cands or [path]
        for uri in cands:
            for suffix in ("", "|geometrytype=Polygon", "|geometrytype=LineString",
                           "|geometrytype=Polygon25D", "|geometrytype=LineString25D"):
                l = QgsVectorLayer(uri + suffix, os.path.basename(path), "ogr")
                if l.isValid() and l.featureCount() > 0:
                    gt = _gtype(l)
                    if gt == want or (want == "polygon" and gt == "line"):
                        lyr = l
                        if gt == want:
                            break
            if lyr is not None and _gtype(lyr) == want:
                break
        if lyr is None:
            QMessageBox.warning(self, "Import", "No %s features found in this file." % want)
            return
        if not lyr.crs().isValid():
            QMessageBox.information(self, "Import", "The file has no coordinate system (common for "
                                    "CAD). Assuming the project CRS: %s" % QgsProject.instance().crs().authid())
            lyr.setCrs(QgsProject.instance().crs())
        self._source_name = os.path.splitext(os.path.basename(path))[0]
        self.use_layer(lyr, all_features=True, keep_name=True)

    def use_layer(self, lyr, all_features=False, keep_name=False):
        if lyr is None:
            return
        if not keep_name and not lyr.customProperty("dmp_role"):
            self._source_name = lyr.name()
        feats = list(lyr.getSelectedFeatures()) if (lyr.selectedFeatureCount() and not all_features) \
            else list(lyr.getFeatures())
        geoms = [f.geometry() for f in feats if f.hasGeometry()]
        if not geoms:
            self._warn("The layer has no geometry.")
            return
        g = QgsGeometry.unaryUnion(geoms)
        gt = _gtype(lyr)
        if not self.is_route and gt == "line":
            # CAD: closed polylines -> polygons
            polys = QgsGeometry.polygonize([g])
            if polys.isEmpty():
                self._warn("No closed polylines found to build a boundary from.")
                return
            g = polys
        if self.is_route and gt == "line":
            g = self._chain(geoms, lyr.crs())
            if g is None:
                return
        self.set_boundary(g, lyr.crs())
        self.canvas.setExtent(self.canvas.mapSettings().layerExtentToOutputExtent(lyr, lyr.extent()))
        self.canvas.refresh()

    def set_boundary(self, geom, crs):
        geom = QgsGeometry(geom)
        if self.is_route and geom.isMultipart():
            geom = self._chain([geom], crs)
            if geom is None:
                return
        if geom.isMultipart():
            parts = geom.asGeometryCollection()
            if self.is_route:
                best = max(parts, key=lambda p: p.length())
            else:
                best = max(parts, key=lambda p: p.area())
            if len(parts) > 1:
                self._warn("%d parts found – using the largest. Plan the others separately."
                           % len(parts))
            geom = best
        if not self.is_route and hasattr(geom, "makeValid") and not geom.isGeosValid():
            fixed = geom.makeValid()
            if not fixed.isEmpty():
                geom = fixed
        lyr = L.boundary_layer(crs, self.is_route)
        if lyr.crs() != crs:
            geom.transform(L.transform(crs, lyr.crs()))
        L.clear(lyr)
        L.add(lyr, [(geom, ["route" if self.is_route else "boundary"])])
        self._watch(lyr)
        if self.is_route:
            self._update_feature_len()
        self._refresh_name_hint()
        self.plan(quiet=False)

    # ------------------------------------------------ linear features
    def _chain(self, geoms, crs):
        """Join line pieces (any order / direction) into one route geometry."""
        pieces = []
        for g in geoms:
            if g.isMultipart():
                pieces += [[(p.x(), p.y()) for p in pl] for pl in g.asMultiPolyline()]
            else:
                pieces.append([(p.x(), p.y()) for p in g.asPolyline()])
        pieces = [p for p in pieces if len(p) >= 2]
        if not pieces:
            self._warn("No line geometry found.")
            return None
        chain, gap = core.geometry.chain_lines(pieces)
        if len(pieces) > 1:
            gap_m = gap
            if crs.isGeographic():
                gap_m = gap * 111000.0
            if gap_m > 1.0:
                self._warn("Joined %d pieces; largest gap bridged %.0f m – check the route "
                           "where pieces don't touch." % (len(pieces), gap_m))
            else:
                self.msg("Joined %d line pieces into one route." % len(pieces))
        return QgsGeometry.fromPolylineXY([QgsPointXY(x, y) for x, y in chain])

    def start_pick_lines(self):
        from .maptools import LinePickerTool
        own = [l.id() for l in QgsProject.instance().mapLayers().values()
               if l.customProperty("dmp_role")]
        tool = LinePickerTool(self.canvas, own)
        tool.finished.connect(self._lines_picked)
        tool.cancelled.connect(self._restore_tool)
        self._set_tool(tool)
        self.msg("Click the line features to map (click again to drop one); right-click or "
                 "Enter when done.", dur=10)

    def _lines_picked(self, geoms):
        self._restore_tool()
        self._source_name = ""
        crs = self.canvas.mapSettings().destinationCrs()
        g = self._chain(geoms, crs)
        if g is not None:
            self.set_boundary(g, crs)

    def _update_feature_len(self):
        lyr = L.find(L.CENTRELINE)
        if lyr is None or not lyr.featureCount():
            return
        from qgis.core import QgsDistanceArea
        da = QgsDistanceArea()
        da.setSourceCrs(lyr.crs(), QgsProject.instance().transformContext())
        ell = QgsProject.instance().ellipsoid()
        da.setEllipsoid(ell if ell and ell.upper() != "NONE" else "EPSG:7030")
        km = da.measureLength(next(lyr.getFeatures()).geometry()) / 1000.0
        self.feat_lbl.setText("%.3f km  (KP 0.000 – %.3f)" % (km, km))

    def _watch(self, lyr):
        if getattr(lyr, "_dmp_watched", False):
            return
        lyr._dmp_watched = True
        lyr.editingStopped.connect(self._param_changed)
        lyr.geometryChanged.connect(lambda *a: self._param_changed())

    def toggle_edit_boundary(self, on):
        lyr = L.find(L.CENTRELINE if self.is_route else L.BOUNDARY)
        if lyr is None:
            self.edit_btn.setChecked(False)
            self._warn("Draw or import a boundary first.")
            return
        self.iface.setActiveLayer(lyr)
        if on:
            lyr.startEditing()
            self._watch(lyr)
            self.iface.actionVertexTool().trigger()
            self.msg("Drag vertices to reshape; double-click an edge to add a vertex. "
                     "Click the button again to save.", dur=8)
        else:
            lyr.commitChanges()
            self.iface.actionPan().trigger()
            self.plan(quiet=True)

    def start_pick(self, role):
        tool = PointTool(self.canvas)
        tool.picked.connect(lambda p: self._picked(role, p))
        self._set_tool(tool)
        self.msg("Click the %s location on the map." % {"takeoff": "take-off", "landing": "final landing",
                                                         "station": "launch station"}[role])

    def _picked(self, role, pt):
        self._restore_tool()
        crs = self.canvas.mapSettings().destinationCrs()
        lyr = L.site_layer(crs)
        g = QgsGeometry.fromPointXY(pt)
        if lyr.crs() != crs:
            g.transform(L.transform(crs, lyr.crs()))
        if role != "station":                      # one take-off / landing, many stations
            ids = [f.id() for f in lyr.getFeatures() if f["role"] == role]
            lyr.dataProvider().deleteFeatures(ids)
        L.add(lyr, [(g, [role, ""])])
        self._watch(lyr)
        self._update_site_label()
        self._param_changed()
        if self.mission is None:
            self.plan(quiet=True)

    def _site_point(self, role, crs):
        lyr = L.find(L.SITE)
        if lyr is None:
            return None
        for f in lyr.getFeatures():
            if f["role"] == role and f.hasGeometry():
                g = QgsGeometry(f.geometry())
                g.transform(L.transform(lyr.crs(), crs))
                p = g.asPoint()
                return (p.x(), p.y())
        return None

    def _site_points(self, role, crs):
        lyr = L.find(L.SITE)
        out = []
        if lyr is None:
            return out
        xf = L.transform(lyr.crs(), crs)
        for f in lyr.getFeatures():
            if f["role"] == role and f.hasGeometry():
                g = QgsGeometry(f.geometry())
                g.transform(xf)
                p = g.asPoint()
                out.append((p.x(), p.y()))
        return out

    def clear_stations(self):
        lyr = L.find(L.SITE)
        if lyr is None:
            return
        lyr.dataProvider().deleteFeatures([f.id() for f in lyr.getFeatures() if f["role"] == "station"])
        lyr.triggerRepaint()
        self._update_site_label()
        self._param_changed()

    def auto_stations(self):
        """Place launch stations: per route section, or a range grid over an area."""
        blyr = L.find(L.CENTRELINE if self.is_route else L.BOUNDARY)
        if blyr is None or not blyr.featureCount():
            self._warn("Draw, import or pick the %s first." % ("route" if self.is_route else "boundary"))
            return
        geom = QgsGeometry(next(blyr.getFeatures()).geometry())
        try:
            work = self._working_crs(geom, blyr.crs())
        except ValueError as e:
            self._warn(str(e))
            return
        geom.transform(L.transform(blyr.crs(), work))
        prm = self.params()
        if self.is_route:
            line = geom.asMultiPolyline()[0] if geom.isMultipart() else geom.asPolyline()
            pts = core.suggest_stations(prm, self.drone(), self.conditions(),
                                        [(p.x(), p.y()) for p in line], self.st_side.value())
        else:
            if not prm.max_range_m:
                self.range_cb.setChecked(True)
                prm.max_range_m = self.range_m.value()
                self.msg("Range limit switched on (%.0f m) to size the station grid." % prm.max_range_m)
            poly = geom.asMultiPolygon()[0] if geom.isMultipart() else geom.asPolygon()
            pts = core.suggest_area_stations([[(p.x(), p.y()) for p in r] for r in poly],
                                             prm.max_range_m)
        if not pts:
            self._warn("No stations could be placed.")
            return
        lyr = L.site_layer(self.canvas.mapSettings().destinationCrs())
        lyr.dataProvider().deleteFeatures([f.id() for f in lyr.getFeatures()
                                           if f["role"] in ("station", "takeoff")])
        xf = L.transform(work, lyr.crs())
        feats = []
        for k, (x, y) in enumerate(pts):
            g = QgsGeometry.fromPointXY(QgsPointXY(x, y))
            g.transform(xf)
            feats.append((g, ["takeoff" if k == 0 else "station", ""]))
        L.add(lyr, feats)
        self._watch(lyr)
        self.msg("Placed %d launch stations. Drag them to accessible spots (vertex tool on the "
                 "'Take-off & landing' layer); the plan updates." % len(pts), dur=10)
        self.plan(quiet=False)

    def _update_site_label(self):
        parts = []
        p = self._site_point("takeoff", WGS84)
        if p:
            parts.append("Take-off S1: %.6f, %.6f" % (p[1], p[0]))
        n = len(self._site_points("station", WGS84))
        if n:
            parts.append("+ %d launch station%s" % (n, "" if n == 1 else "s"))
        p = self._site_point("landing", WGS84)
        if p and not self.same_ld.isChecked():
            parts.append("Final landing: %.6f, %.6f" % (p[1], p[0]))
        if parts:
            self.site_lbl.setText("   ".join(parts) + "\n(drag them with the vertex tool on the "
                                  "'Take-off & landing' layer to move)")

    # ========================================================== planning
    def _working_crs(self, geom, src_crs):
        if not self.auto_utm.isChecked():
            crs = self.crs_w.crs()
            if crs.isGeographic():
                raise ValueError("The planning CRS must be projected (metres), not geographic. "
                                 "Tick 'Automatic UTM zone' or pick a projected CRS.")
            return crs
        c = QgsGeometry(geom.centroid())
        c.transform(L.transform(src_crs, WGS84))
        p = c.asPoint()
        crs = QgsCoordinateReferenceSystem("EPSG:%d" % core.utm_epsg(p.x(), p.y()))
        self.crs_w.blockSignals(True)
        self.crs_w.setCrs(crs)
        self.crs_w.blockSignals(False)
        return crs

    def _dem_changed(self, *_):
        self._dem_cache = None
        self._param_changed()

    def _dem_sampler(self, crs, extent_pts=None):
        """Elevation function in `crs`. File DEMs are read once into memory
        (window around the mission) for fast bilinear sampling."""
        dem = self.dem_cb.currentLayer()
        if not isinstance(dem, QgsRasterLayer):
            return None
        if not dem.crs().isValid():
            self._warn("The DEM '%s' has no coordinate system – set it in Layer Properties > "
                       "Source, then re-plan." % dem.name())
            return None
        xf = L.transform(crs, dem.crs())

        def to_native(x, y):
            p = xf.transform(QgsPointXY(x, y))
            return p.x(), p.y()
        key = (dem.id(), crs.authid(), tuple(round(v) for v in self._bbox(extent_pts)) if extent_pts else None)
        cache = getattr(self, "_dem_cache", None)
        if cache and cache[0] == key:
            g = cache[1]
            g.to_native = to_native
            return g
        try:
            from osgeo import gdal
            ds = gdal.Open(dem.source())
            if ds is None:
                raise RuntimeError
            gt = ds.GetGeoTransform()
            band = ds.GetRasterBand(1)
            c0, r0, c1, r1 = 0, 0, ds.RasterXSize, ds.RasterYSize
            if extent_pts:
                xs, ys = zip(*[to_native(*q) for q in extent_pts])
                pad = 3 * abs(gt[1])
                c0 = max(0, int((min(xs) - pad - gt[0]) / gt[1]))
                c1 = min(ds.RasterXSize, int((max(xs) + pad - gt[0]) / gt[1]) + 2)
                r0 = max(0, int((max(ys) + pad - gt[3]) / gt[5]))
                r1 = min(ds.RasterYSize, int((min(ys) - pad - gt[3]) / gt[5]) + 2)
            if c1 <= c0 or r1 <= r0 or (c1 - c0) * (r1 - r0) > 60_000_000:
                raise RuntimeError
            arr = band.ReadAsArray(c0, r0, c1 - c0, r1 - r0)
            g = core.dem.GridDEM(arr, (gt[0] + c0 * gt[1], gt[1], 0.0, gt[3] + r0 * gt[5], 0.0, gt[5]),
                                 band.GetNoDataValue(), to_native)
            self._dem_cache = (key, g)
            return g
        except Exception:
            prov = dem.dataProvider()

            def f(x, y):
                v, ok = prov.sample(QgsPointXY(*to_native(x, y)), 1)
                if not ok or v != v:
                    raise ValueError("outside DEM")
                return float(v)
            return f

    @staticmethod
    def _bbox(pts):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))

    def _area_points_wgs84(self):
        """Boundary/route vertices and stations in WGS84, for the DEM download area."""
        pts = []
        blyr = L.find(L.CENTRELINE if self.is_route else L.BOUNDARY)
        if blyr is not None and blyr.featureCount():
            xf = L.transform(blyr.crs(), WGS84)
            g = QgsGeometry(next(blyr.getFeatures()).geometry())
            g.transform(xf)
            pts += [(v.x(), v.y()) for v in g.vertices()]
        for role in ("takeoff", "station", "landing"):
            pts += self._site_points(role, WGS84)
        return pts

    def download_dem(self):
        from qgis.PyQt.QtWidgets import QProgressDialog
        pts = self._area_points_wgs84()
        if not pts:
            self._warn("Draw, import or pick the mission area first – the DEM is downloaded for it.")
            return
        buf = max(1000.0, self.range_m.value() if self.range_cb.isChecked() else 0.0)
        bbox = core.dem.bbox_lonlat(pts, buf)
        src = self.dem_src.currentData()
        proj_dir = QgsProject.instance().homePath()
        folder = os.path.join(proj_dir, "dem") if proj_dir else tempfile.mkdtemp(prefix="dmp_dem_")
        os.makedirs(folder, exist_ok=True)
        name = "DEM %s %.3f_%.3f" % ("Copernicus GLO-30" if src == "copernicus" else "Terrain Tiles",
                                     bbox[1], bbox[0])
        path = os.path.join(folder, name.replace(" ", "_") + ".tif")
        dlg = QProgressDialog("Downloading elevation data…", "Cancel", 0, 100, self)
        dlg.setWindowTitle("Download DEM")
        dlg.setMinimumDuration(0)
        dlg.setValue(0)

        def progress(done, total):
            dlg.setValue(int(100 * done / max(total, 1)))
            QApplication.processEvents()
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            if src == "copernicus":
                info = core.dem.fetch_copernicus(bbox, path, progress, dlg.wasCanceled)
            else:
                grid, gt, epsg, info = core.dem.fetch_terrarium(bbox, progress=progress,
                                                                 cancelled=dlg.wasCanceled)
                core.dem.write_geotiff(grid, gt, epsg, path, core.dem.webmerc_bbox(bbox))
        except Exception as e:
            dlg.close()
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, "Download DEM", "%s\n\nCheck your internet connection / "
                                "proxy (Settings > Options > Network), or try the other source." % e)
            return
        QApplication.restoreOverrideCursor()
        dlg.setValue(100)
        lyr = QgsRasterLayer(path, name)
        if not lyr.isValid():
            self._warn("Downloaded file could not be opened: %s" % path)
            return
        lyr.setCustomProperty("dmp_attribution", info["attribution"])
        QgsProject.instance().addMapLayer(lyr, False)
        QgsProject.instance().layerTreeRoot().addLayer(lyr)
        self.dem_cb.setLayer(lyr)
        if info.get("missing_tiles"):
            self._warn("%d tile(s) have no data (sea, or not released) and were set to 0 m."
                       % info["missing_tiles"])
        self.msg("DEM loaded (%s). %s" % (os.path.basename(path), info["attribution"]), dur=10)
        self.elev_from_dem()
        self.plan(quiet=True)

    def elev_from_dem(self):
        dem = self.dem_cb.currentLayer()
        if not isinstance(dem, QgsRasterLayer):
            self._warn("Choose a DEM layer first.")
            return
        p = self._site_point("takeoff", dem.crs())
        if p is None:
            if self.mission is None:
                self._warn("Pick a take-off point first.")
                return
            g = QgsGeometry.fromPointXY(QgsPointXY(*self.mission.takeoff))
            g.transform(L.transform(self.work_crs, dem.crs()))
            p = (g.asPoint().x(), g.asPoint().y())
        v, ok = dem.dataProvider().sample(QgsPointXY(*p), 1)
        if ok:
            self._set_m(self.elev, v)
        else:
            self._warn("Take-off point is outside the DEM.")

    def params(self):
        mode = ["auto", "manual", "wind"][self.head_mode.currentIndex()]
        return core.MissionParams(
            mission_type=core.ROUTE if self.is_route else core.AREA,
            mission_name=self.mission_name(),
            height_m=self._m(self.height), gsd_cm=None,
            rth_height_m=self._m(self.rth_h) or None, battery_height_m=self._m(self.batt_h) or None,
            min_agl_m=self._m(self.min_agl) or None,
            obstacle_margin_m=self._m(self.obs_margin), obstacle_buffer_m=self._m(self.obs_buffer),
            height_unit=self._unit, mass_unit=self.payload_tbl.unit,
            front_overlap_pct=self.front.value(), side_overlap_pct=self.side.value(),
            heading_deg=self.head.value(), heading_mode=mode,
            crosshatch=self.crosshatch.isChecked(), margin_m=self.margin.value(),
            overshoot_m=self.overshoot.value(),
            left_width_m=self.left_w.value(), right_width_m=self.right_w.value(),
            corridor_lines=self.corr_n.value() or None,
            chainage_start_m=self.kp_from.value() * 1000 or None,
            chainage_end_m=self.kp_to.value() * 1000 or None,
            simplify_m=self.simplify.value(), reverse_route=self.rev_kp.isChecked(),
            section_length_m=[None, self.sec_len.value() * 1000, 0][self.sec_mode.currentIndex()],
            split_midline=self.midline.isChecked(),
            max_range_m=self.range_m.value() if self.range_cb.isChecked() else None,
            leapfrog=self.leapfrog.isChecked(),
            speed_ms=None if self.speed_auto.isChecked() else self.speed.value(),
            portrait=self.portrait.isChecked(), shutter_s=self.shutter.currentData(),
            photo_trigger=self.trigger_cb.currentData(), stop_hover_s=self.stop_hover.value(),
            gimbal_pitch_deg=self.pitch.value(), split_batteries=self.split.isChecked(),
            terrain_follow=self.terrain.isChecked())

    def conditions(self):
        return core.Conditions(
            wind_speed_ms=self._wind_ms(self.wind), wind_from_deg=self.wdir.value(), wind_unit=self._wunit,
            temperature_c=self.temp.value(), site_elevation_m=self._m(self.elev),
            payload_kg=0.0, payload_items=self.payload_tbl.items(), reserve_pct=self.reserve.value(),
            battery_health_pct=self.health.value())

    def logistics(self):
        t = self.start_t.time()
        return core.LogisticsParams(
            battery_sets=self.sets.value(), chargers=self.chargers.value(),
            charge_min=self.charge_min.value(), swap_min=self.swap_min.value(),
            setup_min=self.setup_min.value(), drive_kmh=self.drive_kmh.value(),
            relocate_overhead_min=self.reloc_min.value(),
            start_time_min=t.hour() * 60 + t.minute())

    def plan(self, quiet=False):
        blyr = L.find(L.CENTRELINE if self.is_route else L.BOUNDARY)
        if blyr is None or blyr.featureCount() == 0:
            if not quiet:
                self._warn("Draw, import or choose a %s first." % ("route line" if self.is_route else "boundary"))
            return
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            geom = QgsGeometry(next(blyr.getFeatures()).geometry())
            work = self._working_crs(geom, blyr.crs())
            geom.transform(L.transform(blyr.crs(), work))
            if self.is_route:
                if geom.isMultipart():
                    gdata, _ = core.geometry.chain_lines(
                        [[(p.x(), p.y()) for p in pl] for pl in geom.asMultiPolyline()])
                else:
                    gdata = [(p.x(), p.y()) for p in geom.asPolyline()]
            else:
                poly = geom.asMultiPolygon()[0] if geom.isMultipart() else geom.asPolygon()
                gdata = [[(p.x(), p.y()) for p in ring] for ring in poly]
            tk = self._site_point("takeoff", work)
            if tk is None:
                # default: boundary vertex nearest the map centre
                c = QgsGeometry.fromPointXY(self.canvas.center())
                c.transform(L.transform(self.canvas.mapSettings().destinationCrs(), work))
                cp = c.asPoint()
                verts = gdata if self.is_route else gdata[0]
                tk = min(verts, key=lambda v: (v[0] - cp.x()) ** 2 + (v[1] - cp.y()) ** 2)
            ld = None if self.same_ld.isChecked() else self._site_point("landing", work)
            ext = (gdata if self.is_route else gdata[0]) + [tk] + self._site_points("station", work) + \
                ([ld] if ld else [])
            xs, ys = zip(*ext)
            pad = max(1500.0, self.range_m.value() if self.range_cb.isChecked() else 0.0)
            ext_box = [(min(xs) - pad, min(ys) - pad), (max(xs) + pad, max(ys) + pad)]
            dem = self._dem_sampler(work, ext_box)
            if self.terrain.isChecked() and dem is None and not quiet:
                self._warn("Terrain follow needs a DEM layer – planning at constant height.")
            obstacles = self._obstacles(work, ext_box)
            m = core.plan_mission(self.params(), self.drone(), self.conditions(), gdata, tk, ld,
                                  ground_elevation=dem,
                                  stations=self._site_points("station", work),
                                  logistics=self.logistics(), obstacles=obstacles)
            if (dem is not None or obstacles) and self.tcheck.isChecked():
                limit = self._m(self.tclear)
                if self._m(self.min_agl):                   # the minimum height is the clearance target
                    limit = self._m(self.min_agl) - 0.5
                core.terrain.analyse(m, dem, limit, self._m(self.tagl))
            self.payload_tbl.set_limit(m.drone.max_payload_kg)
            self.mission, self.work_crs = m, work
            self._draw_result(m, work)
            self.report.setPlainText(m.report())
            self.profile.set_rows(m.profile if m.profile else [], self._m(self.tclear), self._unit)
            self._update_site_label()
            if m.warnings and not quiet:
                self._warn(m.warnings[0])
        except Exception as e:
            # never leave a stale mission around to be exported or uploaded
            self.mission = None
            if self.work_crs is not None:
                for lyr in L.output_layers(self.work_crs).values():
                    L.clear(lyr)
                ck = L.find(L.CHAINAGE)
                if ck is not None:
                    L.clear(ck)
            detail = "" if isinstance(e, ValueError) else "\n\n" + traceback.format_exc()
            self.report.setPlainText("Planning failed:\n%s%s" % (e, detail))
            self.profile.set_rows([])
            if not quiet:
                self._warn(str(e))
        finally:
            QApplication.restoreOverrideCursor()

    def _draw_result(self, m, crs):
        out = L.output_layers(crs)
        for lyr in out.values():
            L.clear(lyr)
        L.add(out[L.LEGS], [(QgsGeometry.fromPolylineXY([QgsPointXY(*a), QgsPointXY(*b)]),
                             [i + 1, round(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5, 1)])
                            for i, (a, b) in enumerate(m.lines)])
        sched = {r.flight: r for r in (m.schedule.rows if m.schedule else [])}
        clk = lambda v: core.clock(m.schedule.params.start_time_min + v)
        rows = []
        for f in m.flights:
            r = sched.get(f.index + 1)
            rows.append((QgsGeometry.fromPolylineXY([QgsPointXY(w.x, w.y) for w in f.waypoints]),
                         [f.index + 1, round(f.duration_s / 60, 1), round(f.battery_fraction * 100, 0),
                          f.photos, round(f.distance_m, 0), m.label(f.station_start),
                          m.label(f.station_end), r.battery_set if r else None,
                          clk(r.start_min) if r else "", clk(r.end_min) if r else "",
                          round(f.chainage[0] / 1000, 3) if f.chainage else None,
                          round(f.chainage[1] / 1000, 3) if f.chainage else None,
                          f.end_reason]))
        L.add(out[L.ROUTES], rows)
        wps = []
        for f in m.flights:
            for i, w in enumerate(f.waypoints):
                wps.append((QgsGeometry.fromPointXY(QgsPointXY(w.x, w.y)),
                            [f.index + 1, i, w.kind, round(w.alt, 1), round(w.speed, 1),
                             round(w.photo_dist, 1)]))
        L.add(out[L.WAYPOINTS], wps)
        pts = m.photo_points()
        if len(pts) <= 20000:
            L.add(out[L.PHOTOS], [(QgsGeometry.fromPointXY(QgsPointXY(*p)), [i + 1])
                                  for i, p in enumerate(pts)])
        # terrain problem spots
        tl = L.terrain_layer(crs)
        L.clear(tl)
        if m.terrain is not None and m.terrain.issues:
            L.add(tl, [(QgsGeometry.fromPointXY(QgsPointXY(i.x, i.y)),
                        [i.kind, i.flight, round(i.value_m, 1), round(i.ground_m, 1),
                         round(i.alt_amsl, 1), round(i.kp / 1000, 3) if i.kp is not None else None])
                       for i in m.terrain.issues])
        # chainage markers along a route
        kl = L.chainage_layer(crs)
        L.clear(kl)
        if m.centreline and self.kp_int.value() > 0:
            marks = m.chainage_markers(self.kp_int.value())
            if len(marks) <= 5000:
                L.add(kl, [(QgsGeometry.fromPointXY(QgsPointXY(x, y)), [round(c / 1000, 3)])
                           for x, y, c in marks])
        # put take-off on the site layer if it was defaulted; label stations S1, S2...
        site = L.site_layer(crs)
        if not any(f["role"] == "takeoff" for f in site.getFeatures()):
            g = QgsGeometry.fromPointXY(QgsPointXY(*m.takeoff))
            g.transform(L.transform(crs, site.crs()))
            L.add(site, [(g, ["takeoff", ""])])
            self._watch(site)
        xf = L.transform(site.crs(), crs)
        idx = site.fields().indexOf("name")
        changes = {}
        for f in site.getFeatures():
            g = QgsGeometry(f.geometry()); g.transform(xf)
            p = g.asPoint()
            best = min(m.station_labels.items(),
                       key=lambda kv: (kv[0][0] - p.x()) ** 2 + (kv[0][1] - p.y()) ** 2, default=None)
            if best and ((best[0][0] - p.x()) ** 2 + (best[0][1] - p.y()) ** 2) < 1.0:
                if f["name"] != best[1]:
                    changes[f.id()] = {idx: best[1]}
        if changes and not site.isEditable():
            site.dataProvider().changeAttributeValues(changes)
            site.triggerRepaint()
        self.canvas.refresh()

    def to_wgs84(self):
        xf = L.transform(self.work_crs, WGS84)

        def f(x, y):
            p = xf.transform(QgsPointXY(x, y))
            return p.x(), p.y()
        return f

    # ================================================== export / upload
    def _need_mission(self):
        if self.mission is None:
            self._warn("Plan a mission first.")
            return False
        return True

    def export(self):
        if not self._need_mission():
            return
        key = self.fmt_cb.currentData()
        ext = exporters.FORMATS[key][1]
        path, _ = QFileDialog.getSaveFileName(self, "Export mission",
                                              os.path.join(QSettings().value("dmp/lastdir", ""),
                                                           core.safe_filename(self.mission.params.mission_name)
                                                           + ("_waypoints" if key == "csv" else "") + ext),
                                              "%s (*%s)" % (exporters.FORMATS[key][0], ext))
        if not path:
            return
        if not path.lower().endswith(ext):
            path += ext
        QSettings().setValue("dmp/lastdir", os.path.dirname(path))
        try:
            kw = {}
            if key == "qgc_plan":
                kw["firmware"] = self.fw_cb.currentText().lower()
            if key == "dji_kmz":
                tpl = QSettings().value("dmp/dji_template", "")
                if self.mission.drone.needs_dji_template or tpl:
                    tpl, _ = QFileDialog.getOpenFileName(
                        self, "Waypoint file created by DJI Fly / Pilot 2 for this aircraft (Cancel = none)",
                        tpl or QSettings().value("dmp/lastdir", ""), "DJI waypoint mission (*.kmz)")
                    if tpl:
                        QSettings().setValue("dmp/dji_template", tpl)
                        kw["template"] = tpl
            if key in ("brief", "brief_pdf"):
                kw["kp_interval"] = self.kp_int.value() or None
                bg = self._brief_background()
                if bg is not None:
                    kw["background"] = bg
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            try:
                files = exporters.export(self.mission, key, path, self.to_wgs84(),
                                         per_flight=self.per_flight.isChecked(), **kw)
            finally:
                QApplication.restoreOverrideCursor()
            QMessageBox.information(self, "Export", "Wrote %d file(s):\n\n%s"
                                    % (len(files), "\n".join(files)))
        except Exception as e:
            QMessageBox.critical(self, "Export failed", "%s\n\n%s" % (e, traceback.format_exc()))

    def _tile_basemap(self, key):
        from qgis.core import QgsApplication
        xf = L.transform(WGS84, self.work_crs)

        def from84(lon, lat):
            p = xf.transform(QgsPointXY(lon, lat))
            return p.x(), p.y()
        try:
            cache = os.path.join(QgsApplication.qgisSettingsDirPath(), "cache", "dmp_tiles")
        except Exception:
            cache = None
        return core.basemap.TileBasemap(key, self.to_wgs84(), from84, cache_dir=cache,
                                        log=self._warn)

    def _brief_background(self):
        """Callable (extent, size) -> background for the brief maps, or None."""
        key = self.brief_bg.currentData() or "none"
        if key == "none":
            return None
        if key != "canvas":
            return self._tile_basemap(key)
        osm = []

        def fn(extent, size):
            bg = self._render_background(extent, size)
            if bg is not None:
                return bg
            if not osm:                          # nothing visible under the mission -> OpenStreetMap
                osm.append(self._tile_basemap("osm"))
            return osm[0](extent, size)
        return fn

    def _render_background(self, extent, size):
        """Visible non-plugin layers rendered for an extent in the planning CRS
        (core.basemap.Background with their attribution), or None."""
        from qgis.core import (QgsMapSettings, QgsMapRendererParallelJob, QgsRectangle)
        from qgis.PyQt.QtCore import QSize, QBuffer, QByteArray, QIODevice
        from qgis.PyQt.QtGui import QColor
        layers = [l for l in self.canvas.layers() if not l.customProperty("dmp_role")]
        if not layers:
            return None
        ms = QgsMapSettings()
        ms.setLayers(layers)
        ms.setDestinationCrs(self.work_crs)
        ms.setExtent(QgsRectangle(*extent))
        ms.setOutputSize(QSize(int(size[0] * 2), int(size[1] * 2)))   # 2x for print sharpness
        ms.setBackgroundColor(QColor(252, 252, 251))
        job = QgsMapRendererParallelJob(ms)
        job.start()
        job.waitForFinished()
        img = job.renderedImage()
        ba = QByteArray()
        buf = QBuffer(ba)
        buf.open(QIODevice.OpenModeFlag.WriteOnly if hasattr(QIODevice, "OpenModeFlag") else QIODevice.WriteOnly)
        img.save(buf, "JPG", 82)            # photos compress far better as JPEG
        credits = []
        for lyr in layers:
            a = ""
            try:
                a = lyr.serverProperties().attribution()
            except Exception:
                try:
                    a = lyr.attribution()
                except Exception:
                    a = ""
            a = a or ("\u00a9 OpenStreetMap contributors" if "openstreetmap" in
                      (lyr.source() + lyr.name()).lower() else "")
            if a and a not in credits:
                credits.append(a)
        return core.basemap.Background(data=bytes(ba), attribution=" | ".join(credits)[:140], opacity=0.9)

    def upload(self):
        if not self._need_mission():
            return
        UploadDialog(self).exec()


class UploadDialog(QDialog):
    def __init__(self, dock):
        super().__init__(dock)
        self.dock = dock
        m = dock.mission
        self.setWindowTitle("Upload mission to drone")
        self.setMinimumWidth(520)
        lay = QVBoxLayout(self)
        platform = m.drone.platform

        g = QGroupBox("Method")
        gl = QVBoxLayout(g)
        self.r_mav = QRadioButton("Live upload over MAVLink (ArduPilot / PX4 – USB, telemetry radio, Wi-Fi)")
        self.r_dev = QRadioButton("Copy to controller / SD card / USB stick "
                                  "(DJI Pilot 2, Litchi, Autel, any app that imports files)")
        bg = QButtonGroup(self); bg.addButton(self.r_mav); bg.addButton(self.r_dev)
        gl.addWidget(self.r_mav); gl.addWidget(self.r_dev)
        (self.r_mav if platform == "mavlink" else self.r_dev).setChecked(True)
        lay.addWidget(g)

        self.flight = QComboBox()
        for f in m.flights:
            self.flight.addItem("Flight %d  (%.1f min, %d waypoints)"
                                % (f.index + 1, f.duration_s / 60, len(f.waypoints)), f.index)
        fl = QFormLayout()
        fl.addRow("Flight / battery", self.flight)
        lay.addLayout(fl)

        self.mav_box = QGroupBox("MAVLink connection")
        f = QFormLayout(self.mav_box)
        self.conn = QComboBox(); self.conn.setEditable(True)
        self.conn.addItems(["udpin:0.0.0.0:14550", "COM3", "/dev/ttyUSB0", "/dev/ttyACM0",
                            "tcp:127.0.0.1:5760", "udpout:192.168.4.1:14550"])
        last = QSettings().value("dmp/mavconn", "")
        if last:
            self.conn.setCurrentText(last)
        self.baud = QComboBox(); self.baud.addItems(["57600", "115200", "921600", "38400"])
        f.addRow("Connection", self.conn)
        f.addRow("Baud (serial)", self.baud)
        if not uploader.pymavlink_available():
            n = QLabel("pymavlink is not installed. Open the OSGeo4W Shell and run:\n"
                       "    python -m pip install pymavlink\nthen restart QGIS.")
            n.setStyleSheet("color: #b45309")
            f.addRow(n)
        lay.addWidget(self.mav_box)

        self.dev_box = QGroupBox("Device folder")
        f = QFormLayout(self.dev_box)
        self.dev_fmt = QComboBox()
        for k, (label, ext) in exporters.FORMATS.items():
            self.dev_fmt.addItem(label, k)
        self.dev_fmt.setCurrentIndex(self.dev_fmt.findData(
            exporters.DEFAULT_FORMAT_FOR_PLATFORM.get(platform, "kml")))
        self.dest = QComboBox(); self.dest.setEditable(True)
        self.dest.addItems(uploader.find_removable_drives())
        b = QToolButton(); b.setText("…")
        b.clicked.connect(self._browse)
        row = QHBoxLayout(); row.addWidget(self.dest, 1); row.addWidget(b)
        f.addRow("File format", self.dev_fmt)
        f.addRow("Destination", row)
        self.replace_cb = QCheckBox("Replace a DJI Fly / Pilot 2 mission file (keeps a .bak copy)")
        self.replace_le = QLineEdit()
        self.replace_le.setPlaceholderText("…/waypoint/<id>/<id>.kmz  on the phone / RC")
        rb = QToolButton(); rb.setText("…")
        rb.clicked.connect(self._browse_replace)
        rrow = QHBoxLayout(); rrow.addWidget(self.replace_le, 1); rrow.addWidget(rb)
        f.addRow(self.replace_cb)
        f.addRow("Mission file", rrow)
        self.replace_cb.setChecked(platform == "dji_fly")
        fly_hint = QLabel("DJI Fly: first create any waypoint flight in DJI Fly and save it. Connect the phone / "
                          "RC by USB (Android: File Transfer; iOS: Trust This Computer) and pick its newest KMZ "
                          "in Android/data/dji.go.v5/files/waypoint (Android) or DJI Fly/wayline_mission (iOS). "
                          "It is overwritten with this flight using the aircraft codes from that file; reopen "
                          "DJI Fly to load it.")
        fly_hint.setWordWrap(True); fly_hint.setStyleSheet("color: gray")
        f.addRow(fly_hint)
        hint = QLabel(uploader.DJI_PILOT2_HINT + "\nLitchi: import the CSV at flylitchi.com/hub "
                      "(syncs to the app).\nAutel: import the KML in Autel Explorer / Smart Controller.")
        hint.setWordWrap(True); hint.setStyleSheet("color: gray")
        f.addRow(hint)
        lay.addWidget(self.dev_box)

        self.log = QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumHeight(140)
        lay.addWidget(self.log)
        bb = QDialogButtonBox()
        self.go = bb.addButton("Upload", QDialogButtonBox.ButtonRole.AcceptRole)
        bb.addButton(QDialogButtonBox.StandardButton.Close)
        self.go.clicked.connect(self.run)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)
        self.r_mav.toggled.connect(self._mode)
        self._mode()

    def _mode(self):
        self.mav_box.setVisible(self.r_mav.isChecked())
        self.dev_box.setVisible(self.r_dev.isChecked())

    def _browse(self):
        d = QFileDialog.getExistingDirectory(self, "Controller / SD card folder")
        if d:
            self.dest.setCurrentText(d)

    def _browse_replace(self):
        f, _ = QFileDialog.getOpenFileName(self, "DJI Fly / Pilot 2 mission file to replace",
                                           self.dest.currentText(), "DJI waypoint mission (*.kmz)")
        if f:
            self.replace_le.setText(f)
            self.replace_cb.setChecked(True)

    def _log(self, s):
        self.log.appendPlainText(s)
        QApplication.processEvents()

    def run(self):
        m = self.dock.mission
        k = self.flight.currentData()
        try:
            QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
            if self.r_mav.isChecked():
                QSettings().setValue("dmp/mavconn", self.conn.currentText())
                n = uploader.upload_mavlink(m, self.dock.to_wgs84(), self.conn.currentText().strip(),
                                            int(self.baud.currentText()), k, log=self._log)
                self._log("Done – %d items on the vehicle. Verify in your GCS before flight." % n)
            elif self.replace_cb.isChecked():
                target = self.replace_le.text().strip()
                if not target or not os.path.isfile(target):
                    raise ValueError("Pick the DJI Fly / Pilot 2 mission file (.kmz) to replace.")
                tmp = tempfile.mkdtemp(prefix="dmp_")
                new = os.path.join(tmp, os.path.basename(target))
                exporters.write_dji_kmz(m, new, self.dock.to_wgs84(), flight_index=k, template=target)
                import shutil
                bak = target + ".bak"
                if not os.path.exists(bak):
                    shutil.copy2(target, bak)
                shutil.copyfile(new, target)
                self._log("Replaced %s with flight %d (original kept as %s)." % (target, k + 1, os.path.basename(bak)))
                self._log("Reopen DJI Fly / Pilot 2 and check the route before flying.")
                if m.params.photo_trigger == "timed":
                    self._log("Photo trigger is 'Timed shot': set the camera to Timed %.0f s and start it "
                              "before the mission." % m.drone.camera.min_trigger_interval_s)
            else:
                key = self.dev_fmt.currentData()
                tmp = tempfile.mkdtemp(prefix="dmp_")
                base = os.path.join(tmp, "mission_flight%02d%s" % (k + 1, exporters.FORMATS[key][1]))
                writer_kw = {"firmware": self.dock.fw_cb.currentText().lower()} if key == "qgc_plan" else {}
                fn = {"dji_kmz": exporters.write_dji_kmz, "qgc_plan": exporters.write_qgc_plan,
                      "mp_waypoints": exporters.write_mp_waypoints,
                      "litchi_csv": exporters.write_litchi_csv, "csv": exporters.write_csv}.get(key)
                if fn:
                    res = fn(m, base, self.dock.to_wgs84(), flight_index=k, **writer_kw)
                    files = res if isinstance(res, list) else [base]
                else:
                    files = exporters.export(m, key, base, self.dock.to_wgs84())
                out = uploader.copy_to_device(files, self.dest.currentText().strip())
                for f in out:
                    self._log("Copied → %s" % f)
        except Exception as e:
            self._log("ERROR: %s" % e)
        finally:
            QApplication.restoreOverrideCursor()


def _gtype_geom(g):
    t = g.type()
    name = (getattr(t, "name", None) or str(t)).lower()
    if "polygon" in name or t == 2:
        return "polygon"
    if "line" in name or t == 1:
        return "line"
    return "point"
