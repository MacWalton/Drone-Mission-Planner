import os
import sys

_here = os.path.dirname(__file__)
if _here not in sys.path:
    sys.path.insert(0, _here)   # makes the bundled dronemapper_core importable


def classFactory(iface):
    from .plugin import DroneMissionPlannerPlugin
    return DroneMissionPlannerPlugin(iface)
