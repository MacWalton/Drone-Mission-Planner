# SPDX-License-Identifier: GPL-2.0-only
"""Bundle the shared engine into both plugins and build the install zips.

    python tools/build.py          -> dist/drone_mission_planner_qgis.zip
                                      dist/DroneMissionPlanner_ArcGISPro.zip

The plugins ship with their own copy of dronemapper_core (QGIS and ArcGIS Pro
load them from their own folder), so the copies are generated here and are
not tracked in git. Run this before the QGIS / ArcGIS tests too.
"""
import os
import shutil
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLUGINS = [("qgis", "drone_mission_planner", "drone_mission_planner_qgis.zip"),
           ("arcgis", "DroneMissionPlanner", "DroneMissionPlanner_ArcGISPro.zip")]


def _ignore(_, names):
    return [n for n in names if n == "__pycache__" or n.endswith(".pyc")]


def bundle():
    core = os.path.join(ROOT, "dronemapper_core")
    for parent, name, _ in PLUGINS:
        d = os.path.join(ROOT, parent, name)
        shutil.rmtree(os.path.join(d, "dronemapper_core"), ignore_errors=True)
        shutil.copytree(core, os.path.join(d, "dronemapper_core"), ignore=_ignore)
        for f in ("README.md", "LICENSE"):
            shutil.copy2(os.path.join(ROOT, f), os.path.join(d, f))


def package():
    dist = os.path.join(ROOT, "dist")
    os.makedirs(dist, exist_ok=True)
    out = []
    for parent, name, zname in PLUGINS:
        base = os.path.join(ROOT, parent)
        path = os.path.join(dist, zname)
        with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
            for dp, dns, fns in os.walk(os.path.join(base, name)):
                dns[:] = [x for x in dns if x != "__pycache__"]
                for f in fns:
                    if not f.endswith(".pyc"):
                        full = os.path.join(dp, f)
                        z.write(full, os.path.relpath(full, base))
        out.append(path)
    return out


if __name__ == "__main__":
    bundle()
    for p in package():
        print("built", os.path.relpath(p, ROOT))
