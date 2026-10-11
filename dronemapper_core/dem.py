"""Elevation data: free global DEM download and fast sampling.

Sources (no account or key needed):

* Copernicus GLO-30 (ESA / Copernicus, 30 m, global). A *surface* model (DSM):
  it includes trees and buildings, which is what you want for clearance
  checks. Read with GDAL straight from the public cloud-optimised GeoTIFFs,
  fetching only the window around the mission. Needs GDAL (QGIS always has
  it). Ocean tiles do not exist and are filled with 0 m.
  Attribution: "Copernicus DEM GLO-30 (c) DLR e.V. 2010-2014 and (c) Airbus
  Defence and Space GmbH 2014-2018, provided under COPERNICUS by the European
  Union and ESA."
* AWS Terrain Tiles (Mapzen / Tilezen "terrarium" PNG tiles, Web Mercator,
  zoom <= 15). A blend of SRTM, national LiDAR/DEMs and others - mostly a
  *terrain* model. Pure Python (urllib + zlib), so it also works inside
  ArcGIS Pro, which has no GDAL. Attribution: see
  https://github.com/tilezen/joerd/blob/master/docs/attribution.md

Sampling uses GridDEM: the raster held in memory with bilinear interpolation.
"""
import math
import os
import struct
import time
import urllib.error
import urllib.request
import zlib

COPERNICUS_URL = "https://copernicus-dem-30m.s3.amazonaws.com/{name}/{name}.tif"
TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
USER_AGENT = "DroneMissionPlanner/1.2 (QGIS/ArcGIS plugin)"
WEBMERC_HALF = 20037508.342789244

SOURCES = {
    "copernicus": "Copernicus GLO-30 (30 m surface model, needs GDAL)",
    "terrarium": "AWS Terrain Tiles (~10-30 m terrain model)",
}
ATTRIBUTION = {
    "copernicus": "Copernicus DEM GLO-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space "
                  "GmbH 2014-2018, provided under COPERNICUS by the European Union and ESA.",
    "terrarium": "Terrain Tiles (Mapzen/Tilezen) - sources incl. SRTM, GMTED, ETOPO1 and national "
                 "elevation models; see github.com/tilezen/joerd/blob/master/docs/attribution.md",
}


class DEMError(Exception):
    pass


# ------------------------------------------------------------------ bbox
def bbox_lonlat(points_lonlat, buffer_m=500.0):
    """(lon_min, lat_min, lon_max, lat_max) around points plus a buffer."""
    lons = [p[0] for p in points_lonlat]
    lats = [p[1] for p in points_lonlat]
    lat_c = (min(lats) + max(lats)) / 2.0
    dlat = buffer_m / 111320.0
    dlon = buffer_m / (111320.0 * max(math.cos(math.radians(lat_c)), 0.05))
    return (max(-180.0, min(lons) - dlon), max(-85.0, min(lats) - dlat),
            min(180.0, max(lons) + dlon), min(85.0, max(lats) + dlat))


# ------------------------------------------------------------ HTTP helper
def _get(url, timeout=30, retries=2):
    """Bytes of a URL, or None if it doesn't exist (404/403 = no tile)."""
    last = None
    for attempt in range(retries + 1):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code in (403, 404):
                return None
            last = e
        except Exception as e:          # network hiccup: retry
            last = e
        time.sleep(0.5 * (attempt + 1))
    raise DEMError("Could not download %s: %s" % (url, last))


# ------------------------------------------------------------ PNG decode
def decode_png(data):
    """Decode an 8-bit RGB/RGBA non-interlaced PNG -> (width, height, channels, bytes).
    Uses Pillow when available, otherwise a pure-Python decoder."""
    try:
        from PIL import Image
        import io
        im = Image.open(io.BytesIO(data))
        im = im.convert("RGB")
        return im.width, im.height, 3, im.tobytes()
    except ImportError:
        pass
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise DEMError("Not a PNG tile")
    pos, idat, w = 8, [], None
    while pos < len(data):
        length, ctype = struct.unpack(">I4s", data[pos:pos + 8])
        chunk = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if ctype == b"IHDR":
            w, h, depth, color, _, _, interlace = struct.unpack(">IIBBBBB", chunk)
            if depth != 8 or color not in (2, 6) or interlace:
                raise DEMError("Unsupported PNG layout (depth %d colour %d)" % (depth, color))
            bpp = 3 if color == 2 else 4
        elif ctype == b"IDAT":
            idat.append(chunk)
        elif ctype == b"IEND":
            break
    raw = zlib.decompress(b"".join(idat))
    stride = w * bpp
    out = bytearray(h * stride)
    prev = bytearray(stride)
    for r in range(h):
        f = raw[r * (stride + 1)]
        line = bytearray(raw[r * (stride + 1) + 1:(r + 1) * (stride + 1)])
        if f == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 255
        elif f == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 255
        elif f == 3:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 255
        elif f == 4:
            for i in range(stride):
                a = line[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 255
        out[r * stride:(r + 1) * stride] = line
        prev = line
    if bpp == 4:                      # drop alpha
        rgb = bytearray(w * h * 3)
        rgb[0::3], rgb[1::3], rgb[2::3] = out[0::4], out[1::4], out[2::4]
        return w, h, 3, bytes(rgb)
    return w, h, 3, bytes(out)


# ------------------------------------------------------- terrain tiles
def _tile_xy(lon, lat, z):
    n = 2 ** z
    lat = max(min(lat, 85.0511), -85.0511)
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def terrarium_zoom(bbox, target_m=15.0, max_tiles=300):
    lat_c = (bbox[1] + bbox[3]) / 2.0
    z = 15
    while z > 0:
        px = 2 * WEBMERC_HALF / (2 ** z) / 256.0 * math.cos(math.radians(lat_c))
        if px >= target_m * 0.75:
            break
        z -= 1
    while z > 0:
        x0, y0 = _tile_xy(bbox[0], bbox[3], z)
        x1, y1 = _tile_xy(bbox[2], bbox[1], z)
        if (int(x1) - int(x0) + 1) * (int(y1) - int(y0) + 1) <= max_tiles:
            break
        z -= 1
    return z


def fetch_terrarium(bbox, zoom=None, progress=None, cancelled=None, url=None):
    """Download Terrain Tiles covering bbox (lon/lat).

    Returns (rows, gt, epsg, info): rows = list of row lists (metres AMSL),
    gt = GDAL-style geotransform in EPSG:3857.
    progress(done, total) and cancelled() -> bool are optional callbacks.
    """
    url = url or TERRARIUM_URL
    z = zoom or terrarium_zoom(bbox)
    fx0, fy0 = _tile_xy(bbox[0], bbox[3], z)
    fx1, fy1 = _tile_xy(bbox[2], bbox[1], z)
    tx0, ty0, tx1, ty1 = int(fx0), int(fy0), int(fx1), int(fy1)
    nx, ny = tx1 - tx0 + 1, ty1 - ty0 + 1
    total = nx * ny
    W, H = nx * 256, ny * 256
    try:
        import numpy as np
        grid = np.zeros((H, W), dtype="float32")
    except ImportError:
        np = None
        grid = [[0.0] * W for _ in range(H)]
    missing = 0
    done = 0
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            if cancelled and cancelled():
                raise DEMError("Download cancelled")
            data = _get(url.format(z=z, x=tx, y=ty))
            done += 1
            if progress:
                progress(done, total)
            if data is None:
                missing += 1
                continue
            w, h, ch, px = decode_png(data)
            ox, oy = (tx - tx0) * 256, (ty - ty0) * 256
            if np is not None:
                a = np.frombuffer(px, dtype=np.uint8).reshape(h, w, 3).astype("float32")
                grid[oy:oy + h, ox:ox + w] = a[:, :, 0] * 256.0 + a[:, :, 1] + a[:, :, 2] / 256.0 - 32768.0
            else:
                for r in range(h):
                    row = grid[oy + r]
                    base = r * w * 3
                    for c in range(w):
                        k = base + c * 3
                        row[ox + c] = px[k] * 256.0 + px[k + 1] + px[k + 2] / 256.0 - 32768.0
    size = 2 * WEBMERC_HALF / (2 ** z)
    px_m = size / 256.0
    gt = (-WEBMERC_HALF + tx0 * size, px_m, 0.0, WEBMERC_HALF - ty0 * size, 0.0, -px_m)
    info = {"source": "terrarium", "zoom": z, "tiles": total, "missing_tiles": missing,
            "pixel_m": px_m, "attribution": ATTRIBUTION["terrarium"]}
    return grid, gt, 3857, info


# ----------------------------------------------------------- copernicus
def copernicus_names(bbox):
    names = []
    for lat in range(int(math.floor(bbox[1])), int(math.floor(bbox[3])) + 1):
        for lon in range(int(math.floor(bbox[0])), int(math.floor(bbox[2])) + 1):
            ns = "N%02d_00" % lat if lat >= 0 else "S%02d_00" % -lat
            ew = "E%03d_00" % lon if lon >= 0 else "W%03d_00" % -lon
            names.append("Copernicus_DSM_COG_10_%s_%s_DEM" % (ns, ew))
    return names


def fetch_copernicus(bbox, out_path, progress=None, cancelled=None, url=None):
    """Clip Copernicus GLO-30 to bbox (lon/lat) into a GeoTIFF (EPSG:4326).
    Reads only the needed window from the cloud-optimised tiles. Needs GDAL."""
    try:
        from osgeo import gdal
    except ImportError:
        raise DEMError("Copernicus download needs GDAL (available in QGIS). In ArcGIS Pro use "
                       "'AWS Terrain Tiles' instead.")
    url = url or COPERNICUS_URL
    gdal.UseExceptions()
    for k, v in (("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR"),
                 ("CPL_VSIL_CURL_ALLOWED_EXTENSIONS", ".tif"),
                 ("GDAL_HTTP_USERAGENT", USER_AGENT), ("GDAL_HTTP_MAX_RETRY", "3"),
                 ("GDAL_HTTP_RETRY_DELAY", "1")):
        gdal.SetConfigOption(k, v)
    names = copernicus_names(bbox)
    if len(names) > 16:
        raise DEMError("Area spans %d one-degree tiles - too large for one download." % len(names))
    found, missing = [], 0
    for i, n in enumerate(names):
        if cancelled and cancelled():
            raise DEMError("Download cancelled")
        path = "/vsicurl/" + url.format(name=n)
        try:
            ds = gdal.Open(path)
        except RuntimeError:
            ds = None
        if ds is None:
            missing += 1
        else:
            found.append(path)
            ds = None
        if progress:
            progress(i + 1, len(names) + 1)
    if not found:
        raise DEMError("No Copernicus land tiles cover this area (open sea, or a tile not "
                       "released in GLO-30). Try 'AWS Terrain Tiles'.")
    vrt = gdal.BuildVRT("/vsimem/dmp_cop.vrt", found)
    out = gdal.Warp(out_path, vrt, outputBounds=bbox, dstSRS="EPSG:4326", format="GTiff",
                    resampleAlg="bilinear", warpOptions=["INIT_DEST=0"],
                    creationOptions=["COMPRESS=DEFLATE", "TILED=YES"])
    if out is None:
        raise DEMError("GDAL could not write %s" % out_path)
    out = None
    vrt = None
    gdal.Unlink("/vsimem/dmp_cop.vrt")
    if progress:
        progress(len(names) + 1, len(names) + 1)
    return {"source": "copernicus", "tiles": len(names), "missing_tiles": missing,
            "pixel_m": 30.0, "attribution": ATTRIBUTION["copernicus"], "path": out_path}


def crop_grid(grid, gt, bbox_native):
    """Crop a north-up grid to a bbox in its own CRS -> (array, gt)."""
    import numpy as np
    a = np.asarray(grid, dtype="float32")
    x0, y0, x1, y1 = bbox_native
    c0 = max(0, int((x0 - gt[0]) / gt[1]))
    c1 = min(a.shape[1], int(math.ceil((x1 - gt[0]) / gt[1])))
    r0 = max(0, int((gt[3] - y1) / -gt[5]))
    r1 = min(a.shape[0], int(math.ceil((gt[3] - y0) / -gt[5])))
    return a[r0:r1, c0:c1], (gt[0] + c0 * gt[1], gt[1], 0.0, gt[3] + r0 * gt[5], 0.0, gt[5])


def write_geotiff(grid, gt, epsg, out_path, crop_bbox_native=None):
    """Write a grid (numpy array or row lists) to GeoTIFF with GDAL."""
    from osgeo import gdal, osr
    import numpy as np
    a = np.asarray(grid, dtype="float32")
    if crop_bbox_native:
        x0, y0, x1, y1 = crop_bbox_native
        c0 = max(0, int((x0 - gt[0]) / gt[1]))
        c1 = min(a.shape[1], int(math.ceil((x1 - gt[0]) / gt[1])))
        r0 = max(0, int((gt[3] - y1) / -gt[5]))
        r1 = min(a.shape[0], int(math.ceil((gt[3] - y0) / -gt[5])))
        a = a[r0:r1, c0:c1]
        gt = (gt[0] + c0 * gt[1], gt[1], 0.0, gt[3] + r0 * gt[5], 0.0, gt[5])
    drv = gdal.GetDriverByName("GTiff")
    ds = drv.Create(out_path, a.shape[1], a.shape[0], 1, gdal.GDT_Float32,
                    options=["COMPRESS=DEFLATE", "TILED=YES"])
    ds.SetGeoTransform(gt)
    sr = osr.SpatialReference()
    sr.ImportFromEPSG(int(epsg))
    ds.SetProjection(sr.ExportToWkt())
    ds.GetRasterBand(1).WriteArray(a)
    ds.FlushCache()
    ds = None
    return out_path


def webmerc_bbox(bbox):
    """lon/lat bbox -> EPSG:3857 bbox."""
    def fwd(lon, lat):
        x = lon * WEBMERC_HALF / 180.0
        y = math.log(math.tan((90 + lat) * math.pi / 360.0)) * WEBMERC_HALF / math.pi
        return x, y
    x0, y0 = fwd(bbox[0], bbox[1])
    x1, y1 = fwd(bbox[2], bbox[3])
    return (x0, y0, x1, y1)


# ------------------------------------------------------------- sampling
class GridDEM(object):
    """In-memory raster with bilinear sampling in its own CRS.

    grid: numpy 2D array (rows from the top), gt: GDAL geotransform (north-up),
    to_native: optional f(x, y) -> (x, y) from the caller's CRS.
    """

    def __init__(self, grid, gt, nodata=None, to_native=None):
        import numpy as np
        self.a = np.asarray(grid, dtype="float64")
        self.gt = gt
        self.nodata = nodata
        self.to_native = to_native
        self.rows, self.cols = self.a.shape

    def __call__(self, x, y):
        return self.sample(x, y)

    def sample(self, x, y):
        if self.to_native:
            x, y = self.to_native(x, y)
        gt = self.gt
        c = (x - gt[0]) / gt[1] - 0.5
        r = (y - gt[3]) / gt[5] - 0.5
        if c < -0.5 or r < -0.5 or c > self.cols - 0.5 or r > self.rows - 0.5:
            raise ValueError("outside DEM")
        c0 = min(max(int(math.floor(c)), 0), self.cols - 1)
        r0 = min(max(int(math.floor(r)), 0), self.rows - 1)
        c1, r1 = min(c0 + 1, self.cols - 1), min(r0 + 1, self.rows - 1)
        fc, fr = min(max(c - c0, 0.0), 1.0), min(max(r - r0, 0.0), 1.0)
        v = [self.a[r0, c0], self.a[r0, c1], self.a[r1, c0], self.a[r1, c1]]
        if self.nodata is not None and any(abs(q - self.nodata) < 1e-6 or q != q for q in v):
            q = self.a[int(round(r)) if 0 <= round(r) < self.rows else r0,
                       int(round(c)) if 0 <= round(c) < self.cols else c0]
            if (self.nodata is not None and abs(q - self.nodata) < 1e-6) or q != q:
                raise ValueError("nodata")
            return float(q)
        top = v[0] * (1 - fc) + v[1] * fc
        bot = v[2] * (1 - fc) + v[3] * fc
        return float(top * (1 - fr) + bot * fr)
