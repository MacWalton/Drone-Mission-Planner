import os, sys, math, threading, time, io, http.server, socketserver, tempfile
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
import numpy as np
from osgeo import gdal, osr
gdal.UseExceptions()
from dronemapper_core import dem as D
ROOT = REPO + "/tests/demsrv"
def truth(lon, lat):      # analytic terrain: hill + slope, metres
    return 50 + 3000*(lon-115.70) + 120*math.exp(-(((lon-115.75)/0.01)**2 + ((lat+32.40)/0.01)**2))

# ---- range-capable static server (GDAL /vsicurl needs Range)
class H(http.server.SimpleHTTPRequestHandler):
    def __init__(s, *a, **k): super().__init__(*a, directory=ROOT, **k)
    def log_message(s, *a): pass
    def send_head(s):
        path = s.translate_path(s.path)
        if not os.path.isfile(path):
            s.send_error(403 if "terrarium" in s.path or "Copernicus" in s.path else 404); return None
        rng = s.headers.get("Range"); size = os.path.getsize(path); f = open(path, "rb")
        if rng:
            a, b = rng.split("=")[1].split("-"); a = int(a); b = int(b) if b else size - 1; b = min(b, size - 1)
            f.seek(a); s.send_response(206); s.send_header("Content-Range", "bytes %d-%d/%d" % (a, b, size))
            s.send_header("Content-Length", str(b - a + 1)); s.send_header("Accept-Ranges", "bytes"); s.end_headers()
            s._len = b - a + 1; return f
        s.send_response(200); s.send_header("Content-Length", str(size)); s.send_header("Accept-Ranges", "bytes"); s.end_headers(); return f
    def copyfile(s, src, dst):
        n = getattr(s, "_len", None)
        dst.write(src.read(n) if n else src.read())
srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), H); port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
base = "http://127.0.0.1:%d" % port

# ---- 1. terrarium tiles for the bbox, encoded like the real service; one tile deleted = "sea"
bbox = (115.725, -32.42, 115.775, -32.38)
z = D.terrarium_zoom(bbox); print("auto zoom", z)
from PIL import Image
fx0, fy0 = D._tile_xy(bbox[0], bbox[3], z); fx1, fy1 = D._tile_xy(bbox[2], bbox[1], z)
size = 2*D.WEBMERC_HALF/2**z
def merc_inv(x, y): return x/D.WEBMERC_HALF*180, math.degrees(2*math.atan(math.exp(y/D.WEBMERC_HALF*math.pi)) - math.pi/2)
tiles = []
for ty in range(int(fy0), int(fy1)+1):
    for tx in range(int(fx0), int(fx1)+1):
        arr = np.zeros((256, 256, 3), np.uint8)
        for r in range(256):
            for c in range(256):
                x = -D.WEBMERC_HALF + tx*size + (c+0.5)*size/256; y = D.WEBMERC_HALF - ty*size - (r+0.5)*size/256
                v = truth(*merc_inv(x, y)) + 32768
                R = int(v // 256); G = int(v - R*256); B = int(round((v - R*256 - G)*256)) % 256
                arr[r, c] = (R, G, B)
        d = os.path.join(ROOT, "terrarium", str(z), str(tx)); os.makedirs(d, exist_ok=True)
        Image.fromarray(arr, "RGB").save(os.path.join(d, "%d.png" % ty), optimize=True)
        tiles.append((tx, ty))
os.remove(os.path.join(ROOT, "terrarium", str(z), str(tiles[0][0]), "%d.png" % tiles[0][1]))  # a missing (sea) tile
print("served %d tiles (1 missing)" % len(tiles))

# pure-python decoder == Pillow on every filter type
raw = open(os.path.join(ROOT, "terrarium", str(z), str(tiles[1][0]), "%d.png" % tiles[1][1]), "rb").read()
import builtins; real_import = builtins.__import__
def no_pil(name, *a, **k):
    if name == "PIL": raise ImportError
    return real_import(name, *a, **k)
builtins.__import__ = no_pil; t0 = time.time(); w, h, ch, pure = D.decode_png(raw); t_pure = time.time()-t0; builtins.__import__ = real_import
_, _, _, pil = D.decode_png(raw)
print("pure-python PNG decode identical to Pillow:", pure == pil, "(%.2f s/tile)" % t_pure)
# also an RGBA PNG with every filter type, via a manual encoder
def enc_png(arr, filt_cycle=(0,1,2,3,4)):
    import zlib, struct
    h, w, ch = arr.shape; bpp = ch; rows = []
    prev = np.zeros(w*ch, np.int32)
    for r in range(h):
        line = arr[r].reshape(-1).astype(np.int32); f = filt_cycle[r % len(filt_cycle)]
        left = np.concatenate([np.zeros(bpp, np.int32), line[:-bpp]]); upl = np.concatenate([np.zeros(bpp, np.int32), prev[:-bpp]])
        if f == 0: o = line
        elif f == 1: o = line - left
        elif f == 2: o = line - prev
        elif f == 3: o = line - (left + prev)//2
        else:
            p = left + prev - upl; pa, pb, pc = abs(p-left), abs(p-prev), abs(p-upl)
            pr = np.where((pa <= pb) & (pa <= pc), left, np.where(pb <= pc, prev, upl)); o = line - pr
        rows.append(bytes([f]) + bytes((o % 256).astype(np.uint8))); prev = line
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t+d) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6 if ch == 4 else 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(b"".join(rows))) + chunk(b"IEND", b"")
rgba = np.random.RandomState(1).randint(0, 256, (40, 37, 4)).astype(np.uint8)
blob = enc_png(rgba)
builtins.__import__ = no_pil; _, _, _, mine = D.decode_png(blob); builtins.__import__ = real_import
print("RGBA all-filters decode correct:", mine == bytes(rgba[:, :, :3].reshape(-1)), "| Pillow agrees:", D.decode_png(blob)[3] == mine)

# fetch + sample
grid, gt, epsg, info = D.fetch_terrarium(bbox, url=base + "/terrarium/{z}/{x}/{y}.png", progress=lambda a, b: None)
print("terrarium:", info["tiles"], "tiles, missing", info["missing_tiles"], "pixel %.1f m" % info["pixel_m"], "grid", grid.shape)
def to3857(lon, lat): return lon*D.WEBMERC_HALF/180, math.log(math.tan((90+lat)*math.pi/360))*D.WEBMERC_HALF/math.pi
g = D.GridDEM(grid, gt, to_native=to3857)
vals = [(g(lon, lat), truth(lon, lat)) for lon in np.linspace(115.735, 115.77, 15) for lat in np.linspace(-32.415, -32.385, 15)]
land = [abs(a - b) for a, b in vals if a != 0.0]; sea = sum(1 for a, b in vals if a == 0.0)
print("terrarium sample error on land: max %.2f m, mean %.3f m  (%d points in the missing 'sea' tile = 0 m)" % (max(land), sum(land)/len(land), sea))
out = os.path.join(tempfile.mkdtemp(), "t.tif"); D.write_geotiff(grid, gt, 3857, out, D.webmerc_bbox(bbox))
ds = gdal.Open(out); print("terrarium GeoTIFF:", ds.RasterXSize, "x", ds.RasterYSize, osr.SpatialReference(ds.GetProjection()).GetAuthorityCode(None))

# ---- 2. Copernicus: one land COG named per convention; neighbour tile absent (ocean)
names = D.copernicus_names((115.7, -32.5, 115.8, -32.3)); print("copernicus names:", names)
n = names[0]; d = os.path.join(ROOT, n); os.makedirs(d, exist_ok=True)
N = 3600; lon0, lat_top = 115.0, -32.0 + 0.0           # tile S33 E115 covers lat -33..-32
src = gdal.GetDriverByName("MEM").Create("", N, N, 1, gdal.GDT_Float32)
px = 1.0/N; src.SetGeoTransform((lon0, px, 0, lat_top, 0, -px)); s = osr.SpatialReference(); s.ImportFromEPSG(4326); src.SetProjection(s.ExportToWkt())
lons = lon0 + (np.arange(N)+0.5)*px; lats = lat_top - (np.arange(N)+0.5)*px
LON, LAT = np.meshgrid(lons, lats)
src.GetRasterBand(1).WriteArray((50 + 3000*(LON-115.70) + 120*np.exp(-(((LON-115.75)/0.01)**2 + ((LAT+32.40)/0.01)**2))).astype(np.float32))
gdal.Translate(os.path.join(d, n + ".tif"), src, format="COG", creationOptions=["COMPRESS=DEFLATE"])
cb = (115.98, -32.45, 116.02, -32.35)    # straddles E115 (land) and E116 (missing -> sea 0)
out2 = os.path.join(tempfile.mkdtemp(), "c.tif")
t0 = time.time()
info2 = D.fetch_copernicus(cb, out2, url=base + "/{name}/{name}.tif")
print("copernicus:", info2["tiles"], "tiles, missing", info2["missing_tiles"], "(%.1f s)" % (time.time()-t0))
ds = gdal.Open(out2); a = ds.ReadAsArray(); gt2 = ds.GetGeoTransform()
g2 = D.GridDEM(a, gt2)
print("copernicus sample at 115.99 = %.1f (truth %.1f); at 116.01 (sea) = %.1f" % (g2(115.99, -32.40), truth(115.99, -32.40), g2(116.01, -32.40)))
try: g2(117, -32.4)
except ValueError as e: print("outside raises:", e)
try: D.fetch_copernicus((150.1, -40.1, 150.2, -40.0), out2, url=base + "/{name}/{name}.tif")
except D.DEMError as e: print("all-sea area:", str(e)[:70])
srv.shutdown()
