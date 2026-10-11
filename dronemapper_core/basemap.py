# SPDX-License-Identifier: GPL-2.0-only
"""Online basemap tiles under the field-brief maps.

Standard XYZ (Web Mercator, 256 px) tiles are fetched for the map extent and
placed in the SVG one by one, each with an affine transform from its corner
coordinates in the planning CRS. No image decoding or resampling is needed,
so PNG (OpenStreetMap) and JPEG (Esri imagery) tiles both work in plain Python
inside QGIS and ArcGIS Pro.

Respect each provider's terms: the attribution is printed on every map, and
tiles are cached so the same tile is downloaded once per session.
"""
import math
import os
import hashlib
import urllib.request

UA = "DroneMissionPlanner/1.8 (QGIS/ArcGIS field-brief map)"

SOURCES = {
    "osm": {"name": "OpenStreetMap",
            "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
            "attribution": "© OpenStreetMap contributors", "max_zoom": 19, "opacity": 0.8},
    "esri_imagery": {"name": "Esri World Imagery",
                     "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                     "attribution": "Imagery © Esri, Maxar, Earthstar Geographics, GIS User Community",
                     "max_zoom": 19, "opacity": 0.95},
    "esri_topo": {"name": "Esri World Topographic",
                  "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}",
                  "attribution": "Basemap © Esri and contributors", "max_zoom": 19, "opacity": 0.8},
    "esri_streets": {"name": "Esri World Street Map",
                     "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}",
                     "attribution": "Basemap © Esri and contributors", "max_zoom": 19, "opacity": 0.8},
    "esri_gray": {"name": "Esri Light Gray Canvas",
                  "url": "https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Light_Gray_Base/MapServer/tile/{z}/{y}/{x}",
                  "attribution": "Basemap © Esri and contributors", "max_zoom": 16, "opacity": 0.9},
}

# basemap / layer names (ArcGIS Pro basemap gallery, QGIS XYZ names) -> source key
_NAME_HINTS = [("imagery", "esri_imagery"), ("satellite", "esri_imagery"), ("aerial", "esri_imagery"),
               ("hybrid", "esri_imagery"), ("openstreetmap", "osm"), ("osm", "osm"),
               ("topo", "esri_topo"), ("street", "esri_streets"), ("navigation", "esri_streets"),
               ("gray", "esri_gray"), ("grey", "esri_gray"), ("canvas", "esri_gray")]


def source_for_name(name, default="osm"):
    """Best matching tile source for a basemap/layer name (e.g. 'World Imagery')."""
    n = (name or "").lower()
    for hint, key in _NAME_HINTS:
        if hint in n:
            return key
    return default


class Background(object):
    """What svg_map() draws under the mission: one image for the whole map
    (data = PNG/JPEG bytes) or positioned tiles, plus an attribution line."""

    def __init__(self, data=None, tiles=None, attribution="", opacity=0.9):
        self.data = data
        self.tiles = tiles or []      # [(bytes, (tl, tr, bl)) corners in planning CRS]
        self.attribution = attribution
        self.opacity = opacity

    def __bool__(self):
        return bool(self.data) or bool(self.tiles)


def tile_lonlat(x, y, z):
    n = 2.0 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def lonlat_tile(lon, lat, z):
    n = 2.0 ** z
    lat = max(-85.0511, min(85.0511, lat))
    x = (lon + 180.0) / 360.0 * n
    r = math.radians(lat)
    y = (1 - math.log(math.tan(r) + 1 / math.cos(r)) / math.pi) / 2 * n
    return x, y


def choose_zoom(lat, m_per_px, max_zoom=19, scale=2.0):
    """Zoom whose tile resolution matches the map (scale 2 = sharp on print)."""
    res0 = 156543.03392 * math.cos(math.radians(lat))
    z = int(math.ceil(math.log(res0 * scale / max(m_per_px, 0.01), 2)))
    return max(1, min(max_zoom, z))


class TileBasemap(object):
    """Callable (extent, (w, h)) -> Background | None for the brief writer.

    to_wgs84(x, y) -> (lon, lat); from_wgs84(lon, lat) -> (x, y) in the
    planning CRS. fetch(url) -> bytes can be swapped (tests, proxies)."""

    def __init__(self, source, to_wgs84, from_wgs84, cache_dir=None, max_tiles=20, timeout=10,
                 log=None, fetch=None):
        self.key = source if source in SOURCES else source_for_name(source)
        self.src = SOURCES[self.key]
        self.to84, self.from84 = to_wgs84, from_wgs84
        self.cache_dir = cache_dir
        self.max_tiles = max_tiles
        self.timeout = timeout
        self.log = log or (lambda *_: None)
        self.fetch = fetch or self._http
        self.mem = {}
        self.failed = False
        self.errors = 0

    @property
    def attribution(self):
        return self.src["attribution"]

    def _http(self, url):
        req = urllib.request.Request(url, headers={"User-Agent": UA})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return r.read()

    def tile(self, z, x, y):
        k = (z, x, y)
        if k in self.mem:
            return self.mem[k]
        url = self.src["url"].format(z=z, x=x, y=y)
        path = None
        if self.cache_dir:
            path = os.path.join(self.cache_dir, self.key, hashlib.md5(url.encode()).hexdigest() + ".tile")
            if os.path.exists(path):
                with open(path, "rb") as fh:
                    data = fh.read()
                self.mem[k] = data
                return data
        data = self.fetch(url)
        if not data or not (data[:4] == b"\x89PNG" or data[:2] == b"\xff\xd8"):
            raise IOError("not an image tile: %s" % url)
        self.mem[k] = data
        if path:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "wb") as fh:
                    fh.write(data)
            except OSError:
                pass
        return data

    def tile_range(self, extent, width):
        x0, y0, x1, y1 = extent
        samples = [(x0 + (x1 - x0) * i / 4, y0 + (y1 - y0) * j / 4) for i in range(5) for j in range(5)]
        ll = [self.to84(*p) for p in samples]
        lons, lats = [p[0] for p in ll], [p[1] for p in ll]
        lat_c = (min(lats) + max(lats)) / 2
        z = choose_zoom(lat_c, (x1 - x0) / max(width, 1), self.src["max_zoom"])
        while True:
            tx0, ty0 = lonlat_tile(min(lons), max(lats), z)
            tx1, ty1 = lonlat_tile(max(lons), min(lats), z)
            xs = range(int(math.floor(tx0)), int(math.floor(tx1)) + 1)
            ys = range(int(math.floor(ty0)), int(math.floor(ty1)) + 1)
            if len(xs) * len(ys) <= self.max_tiles or z <= 1:
                return z, xs, ys
            z -= 1

    def __call__(self, extent, size):
        if self.failed:
            return None
        z, xs, ys = self.tile_range(extent, size[0])
        n = 2 ** z
        tiles, errs = [], []
        corner = {}

        def C(i, j):                                  # shared corners -> seamless edges
            if (i, j) not in corner:
                corner[(i, j)] = self.from84(*tile_lonlat(i, j, z))
            return corner[(i, j)]

        for ty in ys:
            for tx in xs:
                if not 0 <= ty < n:
                    continue
                try:
                    data = self.tile(z, tx % n, ty)
                except Exception as e:               # offline, blocked, 404 ...
                    errs.append(e)
                    if len(errs) >= 3 and not tiles:
                        break
                    continue
                tiles.append((data, (C(tx, ty), C(tx + 1, ty), C(tx, ty + 1))))
            if len(errs) >= 3 and not tiles:
                break
        if errs:
            self.errors += len(errs)
            if not tiles:
                self.failed = True                   # don't retry for every section map
                self.log("Basemap (%s) not available: %s - brief maps drawn without it."
                         % (self.src["name"], errs[0]))
                return None
        return Background(tiles=tiles, attribution=self.attribution, opacity=self.src["opacity"])
