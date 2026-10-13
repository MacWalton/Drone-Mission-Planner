# SPDX-License-Identifier: GPL-2.0-only
import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
# Always use the dronemapper_core bundled with this plugin: drop any copy already imported
# (an older plugin version before an update / Plugin Reloader, or another folder on the path).
for _n in [n for n in list(sys.modules) if n == "dronemapper_core" or n.startswith("dronemapper_core.")]:
    del sys.modules[_n]
while _here in sys.path:
    sys.path.remove(_here)
sys.path.insert(0, _here)


def classFactory(iface):
    from .plugin import DroneMissionPlannerPlugin
    return DroneMissionPlannerPlugin(iface)
