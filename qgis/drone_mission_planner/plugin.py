"""QGIS plugin entry: toolbar button + menu that opens the planner dock."""
import os

from qgis.PyQt.QtCore import Qt
from qgis.PyQt.QtGui import QIcon
from qgis.PyQt.QtWidgets import QAction

MENU = "&Drone Mission Planner"


class DroneMissionPlannerPlugin:
    def __init__(self, iface):
        self.iface = iface
        self.dock = None
        self.action = None

    def initGui(self):
        icon = QIcon(os.path.join(os.path.dirname(__file__), "icon.png"))
        self.action = QAction(icon, "Drone Mission Planner", self.iface.mainWindow())
        self.action.setCheckable(True)
        self.action.setToolTip("Plan drone mapping missions")
        self.action.triggered.connect(self.toggle)
        self.iface.addToolBarIcon(self.action)
        self.iface.addPluginToMenu(MENU, self.action)

    def unload(self):
        if self.dock:
            self.iface.removeDockWidget(self.dock)
            self.dock.deleteLater()
            self.dock = None
        self.iface.removePluginMenu(MENU, self.action)
        self.iface.removeToolBarIcon(self.action)

    def toggle(self, checked):
        if self.dock is None:
            from .dock import MissionDock
            self.dock = MissionDock(self.iface, self.iface.mainWindow())
            self.dock.visibilityChanged.connect(self.action.setChecked)
            self.iface.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock)
        self.dock.setVisible(checked)
        if checked:
            self.dock.raise_()
