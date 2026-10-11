# SPDX-License-Identifier: GPL-2.0-only
"""Drone mission planning core - shared by the QGIS plugin and ArcGIS toolbox.

Pure Python 3 (no GIS dependencies). pymavlink is optional (live upload).
"""
import math

from .drones import Camera, Drone, DRONES, CAMERAS, get_drone, drone_names
from .performance import Conditions, EnduranceModel, air_density, PayloadItem
from .planner import (MissionParams, Mission, plan_mission, AREA, ROUTE, suggest_stations,
                      route_layout, prepare_centreline, suggest_area_stations,
                      default_mission_name, safe_filename)
from .logistics import LogisticsParams, build_schedule, clock
from . import photogrammetry, geometry, exporters, uploader, logistics, dem, terrain, units, obstacles, basemap
from .obstacles import Obstacle, ObstacleSet

__version__ = "1.7.0"


def utm_epsg(lon, lat):
    """EPSG code of the WGS84 / UTM zone containing lon/lat."""
    zone = int(math.floor((lon + 180.0) / 6.0)) % 60 + 1
    # Norway / Svalbard exceptions
    if 56 <= lat < 64 and 3 <= lon < 12:
        zone = 32
    if 72 <= lat < 84 and 0 <= lon < 42:
        zone = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
    return (32600 if lat >= 0 else 32700) + zone
