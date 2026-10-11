import sys, os, re, tempfile, zlib, struct
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, basemap
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
ti = Transformer.from_crs(4326, 32750, always_xy=True); from84 = lambda lo, la: ti.transform(lo, la)

def png(r, g, b):
    raw = b"".join(b"\x00" + bytes([r, g, b]) * 256 for _ in range(256))
    ch = lambda tag, d: struct.pack(">I", len(d)) + tag + d + struct.pack(">I", zlib.crc32(tag + d) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", 256, 256, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(raw)) + ch(b"IEND", b"")
urls = []
def fake(url):
    urls.append(url); z, x, y = map(int, re.findall(r"/(\d+)/(\d+)/(\d+)\.png", url)[0])
    return png((x * 40) % 255, (y * 40) % 255, 180)
x0, y0 = 381000, 6426000
blk = [[(x0, y0), (x0 + 600, y0), (x0 + 600, y0 + 400), (x0, y0 + 400)]]
m = plan_mission(MissionParams(height_m=80), get_drone("DJI Mini 4 Pro"), Conditions(), blk, (x0 - 20, y0 - 20))
cache = tempfile.mkdtemp()
bm = basemap.TileBasemap("osm", to84, from84, cache_dir=cache, fetch=fake)
out = tempfile.mkdtemp(); f = os.path.join(out, "brief.html")
exporters.export(m, "brief", f, to84, background=bm)
h = open(f).read()
print("tiles fetched", len(urls), "zoom", urls[0].split("/")[3], "| <image> in brief", h.count("<image"), "| attribution", "OpenStreetMap contributors" in h, "| size KB", len(h)//1024)
assert h.count("<image") == len(urls) and "OpenStreetMap contributors" in h
# second run served from cache
bm2 = basemap.TileBasemap("osm", to84, from84, cache_dir=cache, fetch=lambda u: 1/0)
assert bm2(*[basemap and __import__("dronemapper_core.briefmap", fromlist=["x"]).map_view(m, 900)[0], (900, 600)]) is not None
# tiles cover the view: check corners of tile mosaic vs extent
from dronemapper_core.briefmap import map_view
ext, H = map_view(m, 900)
bg = bm(ext, (900, H))
xs = [c[0] for _, cs in bg.tiles for c in cs]; ys = [c[1] for _, cs in bg.tiles for c in cs]
print("mosaic covers extent:", min(xs) <= ext[0], max(xs) >= ext[2], min(ys) <= ext[1], max(ys) >= ext[3])
# offline: no crash, map still drawn, only tried a few tiles then stopped
calls = []
def down(u): calls.append(u); raise IOError("blocked")
logs = []
off = basemap.TileBasemap("esri_imagery", to84, from84, fetch=down, log=logs.append)
exporters.export(m, "brief", f, to84, background=off)
h = open(f).read()
print("offline: calls", len(calls), "map drawn", "<svg" in h, "image", h.count("<image"), "|", logs[0][:70])
print("name hints:", [basemap.source_for_name(n) for n in ("World Imagery", "Topographic", "OpenStreetMap", "Light Gray Canvas", "Streets (Night)", "whatever")])
