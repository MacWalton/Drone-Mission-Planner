"""Minimal real QGIS interface for headless tests (qgis.testing.mocked breaks CRS parsing)."""
from qgis.PyQt.QtWidgets import QMainWindow, QAction
from qgis.gui import QgsMapCanvas, QgsMessageBar


class FakeIface:
    def __init__(self):
        self.mw = QMainWindow()
        self.canvas = QgsMapCanvas(self.mw)
        self.mw.setCentralWidget(self.canvas)
        self.bar = QgsMessageBar(self.mw)
        self._vertex, self._pan = QAction(self.mw), QAction(self.mw)
        self.active = None

    def mapCanvas(self): return self.canvas
    def mainWindow(self): return self.mw
    def messageBar(self): return self.bar
    def addToolBarIcon(self, a): pass
    def removeToolBarIcon(self, a): pass
    def addPluginToMenu(self, m, a): pass
    def removePluginMenu(self, m, a): pass
    def addDockWidget(self, area, w): self.mw.addDockWidget(area, w)
    def removeDockWidget(self, w): self.mw.removeDockWidget(w)
    def setActiveLayer(self, l): self.active = l
    def actionVertexTool(self): return self._vertex
    def actionPan(self): return self._pan
