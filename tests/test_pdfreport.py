import sys, os, re, io, zlib, struct, tempfile, subprocess, shutil
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, basemap, terrain as TR
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
ti = Transformer.from_crs(4326, 32750, always_xy=True); from84 = lambda lo, la: ti.transform(lo, la)
x0, y0 = 381000, 6426000
out = tempfile.mkdtemp()

def chunk(tag, d):
    return struct.pack(">I", len(d)) + tag + d + struct.pack(">I", zlib.crc32(tag + d) & 0xffffffff)

def png(kind, x):
    """kind: 'pal' (OSM style), 'rgb', 'rgba'."""
    W = 256
    if kind == "pal":
        raw = b"".join(b"\x00" + bytes([(i // 32 + x) % 4 for i in range(W)]) for _ in range(W))
        return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, W, 8, 3, 0, 0, 0)) + \
            chunk(b"PLTE", bytes([242, 239, 233, 170, 211, 223, 200, 230, 200, 255, 255, 255])) + \
            chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")
    n = 3 if kind == "rgb" else 4
    raw = b"".join(b"\x01" + bytes([120, 150 + x % 50, 90] + ([255] if n == 4 else [])) + b"\x00" * (n * (W - 1)) for _ in range(W))
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", W, W, 8, 2 if n == 3 else 6, 0, 0, 0)) + \
        chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")

kinds = ["pal", "rgb", "rgba"]
try:
    from PIL import Image
    def jpg(x):
        b = io.BytesIO(); Image.new("RGB", (256, 256), (90, 130 + x % 60, 80)).save(b, "JPEG"); return b.getvalue()
    kinds.append("jpg")
except ImportError:
    jpg = None
d = get_drone("DJI Matrice 4E")
blk = [[(x0, y0), (x0 + 900, y0), (x0 + 900, y0 + 600), (x0, y0 + 600)]]
ground = lambda x, y: 20 + 0.02 * (x - x0)
m = plan_mission(MissionParams(gsd_cm=2.0, mission_name="PDF test – Lot 7 (stockpile)", terrain_follow=True),
                 d, Conditions(wind_speed_ms=7.2, wind_from_deg=200, wind_unit="kn",
                               payload_items=[PayloadItem("Spotlight", 0.1, 15)]),
                 blk, (x0 - 30, y0 - 30), ground_elevation=ground)
TR.analyse(m, ground, 30, 120)
for kind in kinds:
    bm = basemap.TileBasemap("osm", to84, from84,
                             fetch=lambda u, k=kind: jpg(len(u)) if k == "jpg" else png(k, len(u)))
    f = os.path.join(out, "report_%s.pdf" % kind)
    files = exporters.export(m, "brief_pdf", f, to84, background=bm)
    data = open(f, "rb").read()
    assert data[:8] == b"%PDF-1.4" and data.rstrip().endswith(b"%%EOF")
    if shutil.which("qpdf"):
        r = subprocess.run(["qpdf", "--check", f], capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr
    txt = ""
    try:
        import pypdf
        rd = pypdf.PdfReader(f); txt = "\n".join(p.extract_text() for p in rd.pages)
        npages = len(rd.pages)
    except ImportError:
        npages = data.count(b"/Type /Page ")
    print("  %-4s %d pages %4d KB images %d | title %s | wind kn %s | flights rows %s | attribution %s" % (
        kind, npages, len(data) // 1024, data.count(b"/Subtype /Image"),
        "PDF test" in txt, "kn (3.7 m/s)" in txt or "14 kn" in txt, "S1 -> S1" in txt,
        "OpenStreetMap contributors" in txt))
    assert data.count(b"/Subtype /Image") >= 1
    if txt:
        assert "PDF test" in txt and "Flights & battery plan" in txt and "OpenStreetMap" in txt
# no background, route with sections
road = [(x0 + i * 120, y0 + 2000 + 200 * __import__("math").sin(i / 6.0)) for i in range(50)]
mp = MissionParams(mission_type=ROUTE, gsd_cm=2.0, left_width_m=30, right_width_m=30, max_range_m=1000)
sts = suggest_stations(mp, d, Conditions(), road)
mr = plan_mission(mp, d, Conditions(), road, sts[0], stations=sts[1:])
f = os.path.join(out, "route.pdf"); exporters.export(mr, "brief_pdf", f, to84, kp_interval=1000)
print("  route: %d KB, sections %d" % (os.path.getsize(f) // 1024, len(mr.stations)))
if shutil.which("pdftoppm"):
    subprocess.run(["pdftoppm", "-r", "40", "-png", "-f", "1", "-l", "1", os.path.join(out, "report_pal.pdf"),
                    os.path.join(REPO, "tests", "out", "pdf_pal")], check=True)
print("PDF report OK")
