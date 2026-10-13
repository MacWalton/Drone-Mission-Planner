# SPDX-License-Identifier: GPL-2.0-only
"""Mission report / field brief as PDF - pure Python, no dependencies.

Uses the PDF standard fonts (Helvetica, Courier), so the file opens in any PDF
reader and prints on A4. Maps and the elevation profile are drawn as vector
graphics from the same SVG the HTML brief uses (a small SVG-subset renderer),
with basemap tiles / rendered backgrounds embedded as JPEG or PNG images.
"""
import base64
import hashlib
import math
import re
import struct
import time
import zlib
import xml.etree.ElementTree as ET

# --------------------------------------------------------------- fonts
# Glyph widths (1/1000 em) for WinAnsi codes 32..255, from the Adobe core-font metrics.
_HW = [278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278, 556, 556, 556, 556, 556, 556,
       556, 556, 556, 556, 278, 278, 584, 584, 584, 556, 1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667,
       556, 833, 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556, 333, 556,
       556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556, 556, 556, 333, 500, 278, 556, 500, 722,
       500, 500, 500, 334, 260, 334, 584, 556, 556, 556, 222, 556, 333, 1000, 556, 556, 556, 1000, 556, 556, 556, 556,
       556, 556, 556, 222, 222, 333, 333, 350, 556, 1000, 556, 1000, 556, 556, 556, 556, 556, 556, 278, 556, 556, 556,
       556, 556, 556, 556, 556, 737, 556, 556, 556, 556, 737, 556, 400, 584, 333, 333, 556, 556, 537, 278, 556, 333,
       556, 556, 834, 834, 834, 556, 556, 556, 556, 556, 667, 667, 556, 556, 556, 667, 556, 556, 556, 556, 556, 556,
       556, 556, 556, 556, 556, 556, 778, 584, 556, 556, 556, 556, 722, 556, 556, 611, 556, 556, 556, 556, 556, 556,
       556, 500, 556, 556, 556, 556, 556, 278, 556, 556, 556, 556, 556, 556, 556, 556, 556, 584, 556, 556, 556, 556,
       556, 556, 556, 556]
_HBW = [278, 333, 474, 556, 556, 889, 722, 238, 333, 333, 389, 584, 278, 333, 278, 278, 556, 556, 556, 556, 556, 556,
        556, 556, 556, 556, 333, 333, 584, 584, 584, 611, 975, 722, 722, 722, 722, 667, 611, 778, 722, 278, 556, 722,
        611, 833, 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 333, 278, 333, 584, 556, 333, 556,
        611, 556, 611, 556, 333, 611, 611, 278, 278, 556, 278, 889, 611, 611, 611, 611, 389, 556, 333, 611, 556, 778,
        556, 556, 500, 389, 280, 389, 584, 556, 556, 556, 278, 556, 500, 1000, 556, 556, 556, 1000, 556, 556, 556, 556,
        556, 556, 556, 278, 278, 500, 500, 350, 556, 1000, 556, 1000, 556, 556, 556, 556, 556, 556, 278, 556, 556, 556,
        556, 556, 556, 556, 556, 737, 556, 556, 556, 556, 737, 556, 400, 584, 333, 333, 556, 611, 556, 278, 556, 333,
        556, 556, 834, 834, 834, 556, 556, 556, 556, 556, 722, 722, 556, 556, 556, 667, 556, 556, 556, 556, 556, 556,
        556, 556, 556, 556, 556, 556, 778, 584, 556, 556, 556, 556, 722, 556, 556, 611, 556, 556, 556, 556, 556, 556,
        556, 556, 556, 556, 556, 556, 556, 278, 556, 556, 556, 611, 556, 611, 556, 556, 611, 584, 556, 556, 611, 556,
        611, 556, 556, 556]
FONTS = {"F1": "Helvetica", "F2": "Helvetica-Bold", "F3": "Courier"}
_SUBST = {"→": "->", "←": "<-", "≤": "<=", "≥": ">=", "−": "-", " ": " ",
          " ": " ", "☑": "[x]", "☐": "[ ]", "▲": "^", "▼": "v", "✓": "v"}


def _enc(s):
    s = "".join(_SUBST.get(ch, ch) for ch in str(s))
    b = s.encode("cp1252", "replace")
    return b.replace(b"\\", b"\\\\").replace(b"(", b"\\(").replace(b")", b"\\)")


def text_width(s, size, font="F1"):
    if font == "F3":
        return 0.6 * size * len(s)
    tab = _HBW if font == "F2" else _HW
    w = 0
    for ch in "".join(_SUBST.get(c, c) for c in str(s)).encode("cp1252", "replace"):
        w += tab[ch - 32] if 32 <= ch <= 255 else 556
    return w * size / 1000.0


def wrap(s, size, width, font="F1"):
    """Greedy word wrap -> list of lines."""
    out = []
    for para in str(s).split("\n"):
        line = ""
        for word in para.split(" "):
            cand = word if not line else line + " " + word
            if text_width(cand, size, font) <= width or not line:
                line = cand
                while text_width(line, size, font) > width and len(line) > 1:   # very long word
                    k = len(line)
                    while k > 1 and text_width(line[:k], size, font) > width:
                        k -= 1
                    out.append(line[:k])
                    line = line[k:]
            else:
                out.append(line)
                line = word
        out.append(line)
    return out


# --------------------------------------------------------------- colours
_NAMED = {"white": (1, 1, 1), "black": (0, 0, 0), "none": None}


def colour(v):
    if v is None:
        return None
    v = v.strip()
    if v in _NAMED:
        return _NAMED[v]
    if v.startswith("#"):
        h = v[1:]
        if len(h) == 3:
            h = "".join(c * 2 for c in h)
        try:
            return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
        except ValueError:
            return None
    m = re.match(r"rgba?\(([^)]*)\)", v)
    if m:
        parts = [float(x) for x in m.group(1).split(",")[:3]]
        return tuple(p / 255.0 for p in parts)
    return None


# --------------------------------------------------------------- images
def _jpeg_info(data):
    i = 2
    while i < len(data) - 9:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack(">HH", data[i + 5:i + 9])
            return w, h, data[i + 9]
        i += 2 + struct.unpack(">H", data[i + 2:i + 4])[0]
    raise ValueError("bad JPEG")


def image_object(data):
    """(dict entries, stream bytes) for a PDF image XObject from JPEG / PNG bytes."""
    if data[:2] == b"\xff\xd8":
        w, h, nc = _jpeg_info(data)
        cs = {1: "/DeviceGray", 3: "/DeviceRGB", 4: "/DeviceCMYK"}.get(nc, "/DeviceRGB")
        extra = " /Decode [1 0 1 0 1 0 1 0]" if nc == 4 else ""
        return ("/Width %d /Height %d /ColorSpace %s /BitsPerComponent 8 /Filter /DCTDecode%s"
                % (w, h, cs, extra)), data
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        pos, idat, plte = 8, [], b""
        w = h = depth = ctype = inter = None
        while pos < len(data):
            ln, typ = struct.unpack(">I4s", data[pos:pos + 8])
            chunk = data[pos + 8:pos + 8 + ln]
            pos += 12 + ln
            if typ == b"IHDR":
                w, h, depth, ctype, _, _, inter = struct.unpack(">IIBBBBB", chunk)
            elif typ == b"PLTE":
                plte = chunk
            elif typ == b"IDAT":
                idat.append(chunk)
            elif typ == b"IEND":
                break
        if depth == 8 and not inter and ctype in (0, 2, 3):    # pass the zlib stream straight through
            colors = 3 if ctype == 2 else 1
            if ctype == 3:
                cs = "[/Indexed /DeviceRGB %d <%s>]" % (len(plte) // 3 - 1, plte.hex())
            else:
                cs = "/DeviceRGB" if ctype == 2 else "/DeviceGray"
            return ("/Width %d /Height %d /ColorSpace %s /BitsPerComponent 8 /Filter /FlateDecode "
                    "/DecodeParms << /Predictor 15 /Colors %d /BitsPerComponent 8 /Columns %d >>"
                    % (w, h, cs, colors, w)), b"".join(idat)
        from .dem import decode_png                              # RGBA etc. -> plain RGB
        w, h, _, rgb = decode_png(data)
        return ("/Width %d /Height %d /ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /FlateDecode"
                % (w, h)), zlib.compress(rgb, 6)
    raise ValueError("unsupported image")


# --------------------------------------------------------------- document
class PDF(object):
    W, H = 595.28, 841.89                     # A4 portrait, points

    def __init__(self, title=""):
        self.title = title
        self.pages = []                       # list of op lists
        self.images = {}                      # sha1 -> (name, dict, stream)
        self.alphas = {}                      # alpha -> gs name
        self.ops = None

    def new_page(self):
        self.ops = []
        self.pages.append(self.ops)

    # -- drawing primitives (PDF coordinates: origin bottom-left)
    def op(self, s):
        self.ops.append(s)

    def alpha(self, a):
        a = round(max(0.0, min(1.0, a)), 2)
        if a >= 0.999:
            return None
        if a not in self.alphas:
            self.alphas[a] = "GS%d" % len(self.alphas)
        return self.alphas[a]

    def image_name(self, data):
        k = hashlib.sha1(data).hexdigest()
        if k not in self.images:
            d, stream = image_object(data)
            self.images[k] = ("Im%d" % len(self.images), d, stream)
        return self.images[k][0]

    # -- output
    def save(self, path):
        objs = []

        def add(body):
            objs.append(body)
            return len(objs)
        cat = add(None)
        pages_id = add(None)
        font_ids = {k: add(("<< /Type /Font /Subtype /Type1 /BaseFont /%s /Encoding /WinAnsiEncoding >>"
                            % v).encode()) for k, v in FONTS.items()}
        img_ids = {}
        for name, d, stream in self.images.values():
            img_ids[name] = add(b"<< /Type /XObject /Subtype /Image " + d.encode() +
                                b" /Length %d >>\nstream\n" % len(stream) + stream + b"\nendstream")
        gs = " ".join("/%s << /ca %.2f /CA %.2f >>" % (n, a, a) for a, n in self.alphas.items())
        res = ("<< /Font << %s >> /XObject << %s >> /ExtGState << %s >> >>"
               % (" ".join("/%s %d 0 R" % (k, v) for k, v in font_ids.items()),
                  " ".join("/%s %d 0 R" % (k, v) for k, v in img_ids.items()), gs)).encode()
        res_id = add(res)
        kids = []
        for ops in self.pages:
            raw = b"\n".join(
                o if isinstance(o, bytes) else o.encode("latin-1") for o in ops)
            comp = zlib.compress(raw, 6)
            cid = add(b"<< /Length %d /Filter /FlateDecode >>\nstream\n" % len(comp) + comp + b"\nendstream")
            kids.append(add(("<< /Type /Page /Parent %d 0 R /MediaBox [0 0 %.2f %.2f] /Resources %d 0 R "
                             "/Contents %d 0 R >>" % (pages_id, self.W, self.H, res_id, cid)).encode()))
        objs[pages_id - 1] = ("<< /Type /Pages /Kids [%s] /Count %d >>"
                              % (" ".join("%d 0 R" % k for k in kids), len(kids))).encode()
        objs[cat - 1] = ("<< /Type /Catalog /Pages %d 0 R >>" % pages_id).encode()
        info = add(b"<< /Title (" + _enc(self.title) + b") /Producer (Drone Mission Planner) "
                   b"/CreationDate (D:" + time.strftime("%Y%m%d%H%M%S").encode() + b") >>")
        out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
        offs = []
        for i, body in enumerate(objs):
            offs.append(len(out))
            out += b"%d 0 obj\n" % (i + 1) + body + b"\nendobj\n"
        x = len(out)
        out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
        out += b"".join(b"%010d 00000 n \n" % o for o in offs)
        out += b"trailer\n<< /Size %d /Root %d 0 R /Info %d 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (
            len(objs) + 1, cat, info, x)
        with open(path, "wb") as fh:
            fh.write(bytes(out))


# --------------------------------------------------------------- SVG -> PDF
_ENT = {"&middot;": "·", "&ndash;": "–", "&mdash;": "—", "&deg;": "°", "&rarr;": "→",
        "&nbsp;": " ", "&times;": "×", "&copy;": "©"}


def _f(v, d=0.0):
    try:
        return float(re.sub(r"[a-z%]+$", "", str(v).strip()))
    except (TypeError, ValueError):
        return d


def _pts(s):
    nums = [float(n) for n in re.findall(r"-?\d*\.?\d+(?:e-?\d+)?", s or "")]
    return list(zip(nums[0::2], nums[1::2]))


def _style(el, inherit):
    st = dict(inherit)
    for k in ("fill", "stroke", "stroke-width", "stroke-dasharray", "fill-opacity", "stroke-opacity",
              "font-size", "font-weight", "text-anchor", "stroke-linecap"):
        if el.get(k) is not None:
            st[k] = el.get(k)
    if el.get("style"):
        for part in el.get("style").split(";"):
            if ":" in part:
                k, v = part.split(":", 1)
                st[k.strip()] = v.strip()
    st["opacity"] = inherit.get("opacity", 1.0) * _f(el.get("opacity"), 1.0)
    st["class"] = el.get("class", "")
    return st


def draw_svg(pdf, svg, x, y_top, width, max_height=None):
    """Draw an SVG string with its top-left at (x, y_top) [PDF points, y measured
    from the top of the page], scaled to `width`. Returns the drawn height."""
    for k, v in _ENT.items():
        svg = svg.replace(k, v)
    root = ET.fromstring(svg)
    vb = [float(v) for v in (root.get("viewBox") or "0 0 100 100").split()]
    vw, vh = vb[2], vb[3]
    s = width / vw
    if max_height and vh * s > max_height:
        s = max_height / vh
    h = vh * s
    pdf.op("q 1 0 0 1 %.2f %.2f cm %.4f 0 0 %.4f 0 0 cm" % (x, pdf.H - y_top, s, -s))
    pdf.op("0 0 %.2f %.2f re W n" % (vw, vh))
    bg = root.get("style") or ""
    m = re.search(r"background:\s*(#[0-9a-fA-F]{3,6})", bg)
    if m:
        c = colour(m.group(1))
        pdf.op("%.3f %.3f %.3f rg 0 0 %.2f %.2f re f" % (c + (vw, vh)))
    _children(pdf, root, {"fill": "#000", "stroke": "none", "opacity": 1.0})
    pdf.op("Q")
    return h


def _tag(el):
    return el.tag.split("}")[-1]


def _children(pdf, el, st):
    for ch in el:
        t = _tag(ch)
        if t in ("style", "defs", "title", "pattern"):
            continue
        cs = _style(ch, st)
        if t == "g":
            _children(pdf, ch, cs)
        else:
            try:
                _draw(pdf, t, ch, cs)
            except Exception:
                continue                     # never lose the report over one shape


def _paint(pdf, st, path_ops, closed=False):
    fill = st.get("fill", "#000")
    hatch = fill.startswith("url(")
    fc = colour("#8a8984") if hatch else colour(fill)
    sc = colour(st.get("stroke", "none"))
    op = st["opacity"]
    pdf.op("q")
    if fc is not None:
        a = pdf.alpha(op * (0.3 if hatch else _f(st.get("fill-opacity"), 1.0)))
        if a:
            pdf.op("/%s gs" % a)
        pdf.op("%.3f %.3f %.3f rg" % fc)
        pdf.op(path_ops + (" h" if closed else "") + " f")
        pdf.op("Q q")
    if sc is not None:
        a = pdf.alpha(op * _f(st.get("stroke-opacity"), 1.0))
        if a:
            pdf.op("/%s gs" % a)
        pdf.op("%.3f %.3f %.3f RG %.2f w" % (sc + (_f(st.get("stroke-width"), 1.0),)))
        if st.get("stroke-linecap") == "round":
            pdf.op("1 J 1 j")
        da = st.get("stroke-dasharray")
        if da and da != "none":
            pdf.op("[%s] 0 d" % " ".join("%.2f" % float(v) for v in re.split(r"[ ,]+", da.strip()) if v))
        pdf.op(path_ops + (" h" if closed else "") + " S")
    pdf.op("Q")


def _poly(points):
    return " ".join(("%.2f %.2f %s" % (px, py, "m" if i == 0 else "l")) for i, (px, py) in enumerate(points))


def _circle(cx, cy, r):
    k = 0.5523 * r
    return ("%.2f %.2f m %.2f %.2f %.2f %.2f %.2f %.2f c %.2f %.2f %.2f %.2f %.2f %.2f c "
            "%.2f %.2f %.2f %.2f %.2f %.2f c %.2f %.2f %.2f %.2f %.2f %.2f c h"
            % (cx + r, cy, cx + r, cy + k, cx + k, cy + r, cx, cy + r, cx - k, cy + r, cx - r, cy + k, cx - r, cy,
               cx - r, cy - k, cx - k, cy - r, cx, cy - r, cx + k, cy - r, cx + r, cy - k, cx + r, cy))


def _draw(pdf, t, el, st):
    if t in ("polygon", "polyline"):
        pts = _pts(el.get("points"))
        if len(pts) >= 2:
            if t == "polyline":
                st = dict(st, fill="none") if st.get("fill") in (None, "#000") and el.get("fill") is None else st
            _paint(pdf, st, _poly(pts), closed=(t == "polygon"))
    elif t == "line":
        _paint(pdf, dict(st, fill="none"), "%.2f %.2f m %.2f %.2f l" % tuple(
            _f(el.get(k)) for k in ("x1", "y1", "x2", "y2")))
    elif t == "path":
        ops = []
        for cmd, a, b in re.findall(r"([MLml])\s*(-?[\d.]+)[ ,]+(-?[\d.]+)", el.get("d", "")):
            ops.append("%s %s %s" % (a, b, "m" if cmd in "Mm" else "l"))
        if ops:
            _paint(pdf, st, " ".join(ops))
    elif t == "circle":
        _paint(pdf, st, _circle(_f(el.get("cx")), _f(el.get("cy")), _f(el.get("r"))))
    elif t == "rect":
        _paint(pdf, st, "%.2f %.2f %.2f %.2f re" % tuple(_f(el.get(k)) for k in ("x", "y", "width", "height")))
    elif t == "text":
        _text(pdf, el, st)
    elif t == "image":
        _image(pdf, el, st)


def _text(pdf, el, st):
    s = "".join(el.itertext()).strip()
    if not s:
        return
    size = _f(st.get("font-size"), 12)
    bold = str(st.get("font-weight", "")).strip() in ("bold", "600", "700", "800")
    font = "F2" if bold else "F1"
    x, y = _f(el.get("x")), _f(el.get("y"))
    w = text_width(s, size, font)
    anchor = st.get("text-anchor", "start")
    if anchor == "middle":
        x -= w / 2
    elif anchor == "end":
        x -= w
    fc = colour(st.get("fill", "#000")) or (0, 0, 0)
    pdf.op("q")
    m = re.match(r"rotate\(\s*(-?[\d.]+)[ ,]+(-?[\d.]+)[ ,]+(-?[\d.]+)\s*\)", el.get("transform", ""))
    if m:
        ang, cx, cy = (float(v) for v in m.groups())
        r = math.radians(ang)
        pdf.op("1 0 0 1 %.2f %.2f cm %.4f %.4f %.4f %.4f 0 0 cm 1 0 0 1 %.2f %.2f cm"
               % (cx, cy, math.cos(r), math.sin(r), -math.sin(r), math.cos(r), -cx, -cy))
    a = pdf.alpha(st["opacity"])
    if a:
        pdf.op("/%s gs" % a)
    txt = b"(" + _enc(s) + b")"
    if "lbl" in st.get("class", ""):                 # white halo for labels on busy maps
        pdf.op(b"BT /%s %.2f Tf 1 0 0 -1 %.2f %.2f Tm 1 Tr 1 1 1 RG %.2f w 1 j " % (
            font.encode(), size, x, y, size * 0.28) + txt + b" Tj ET")
    pdf.op(b"BT /%s %.2f Tf 1 0 0 -1 %.2f %.2f Tm 0 Tr %.3f %.3f %.3f rg " % (
        (font.encode(), size, x, y) + tuple(fc)) + txt + b" Tj ET")
    pdf.op("Q")


def _image(pdf, el, st):
    href = el.get("href") or el.get("{http://www.w3.org/1999/xlink}href") or ""
    if not href.startswith("data:"):
        return
    data = base64.b64decode(href.split(",", 1)[1])
    name = pdf.image_name(data)
    w, h = _f(el.get("width")), _f(el.get("height"))
    x, y = _f(el.get("x")), _f(el.get("y"))
    pdf.op("q")
    a = pdf.alpha(st["opacity"])
    if a:
        pdf.op("/%s gs" % a)
    m = re.match(r"matrix\(([^)]*)\)", el.get("transform", ""))
    if m:
        pdf.op("%s cm" % " ".join("%.5f" % float(v) for v in re.split(r"[ ,]+", m.group(1).strip())))
    pdf.op("%.3f 0 0 %.3f %.3f %.3f cm /%s Do" % (w, -h, x, y + h, name))
    pdf.op("Q")


# --------------------------------------------------------------- layout
TEAL, INK, MUTED, LINE, HEAD = (0.055, 0.455, 0.565), (0.12, 0.16, 0.22), (0.42, 0.45, 0.50), \
    (0.82, 0.84, 0.86), (0.925, 0.996, 1.0)
WARN = (0.706, 0.325, 0.035)


class Report(object):
    M = 36.0                                  # page margin (pt)

    def __init__(self, title, footer):
        self.pdf = PDF(title)
        self.footer = footer
        self.Wc = PDF.W - 2 * self.M
        self.y = None
        self.page()

    @property
    def bottom(self):
        return PDF.H - self.M - 14

    def page(self):
        self.pdf.new_page()
        self.y = self.M

    def need(self, h):
        if self.y + h > self.bottom:
            self.page()

    def text(self, x, y, s, size=9, font="F1", col=INK):
        self.pdf.op(b"BT /%s %.2f Tf %.3f %.3f %.3f rg %.2f %.2f Td (" % (
            (font.encode(), size) + tuple(col) + (x, PDF.H - y)) + _enc(s) + b") Tj ET")

    def rect(self, x, y, w, h, fill=None, stroke=None, lw=0.6):
        ops = []
        if fill:
            ops.append("%.3f %.3f %.3f rg" % fill)
        if stroke:
            ops.append("%.3f %.3f %.3f RG %.2f w" % (stroke + (lw,)))
        ops.append("%.2f %.2f %.2f %.2f re %s" % (x, PDF.H - y - h, w, h,
                                                 "B" if fill and stroke else "f" if fill else "S"))
        self.pdf.op(" ".join(ops))

    def hline(self, x0, x1, y, col=TEAL, lw=1.4):
        self.pdf.op("%.3f %.3f %.3f RG %.2f w %.2f %.2f m %.2f %.2f l S" % (col + (lw, x0, PDF.H - y, x1, PDF.H - y)))

    # -- blocks
    def title(self, t, sub):
        for ln in wrap(t, 18, self.Wc, "F2"):
            self.y += 19
            self.text(self.M, self.y, ln, 18, "F2")
        self.y += 14
        self.text(self.M, self.y, sub, 9.5, "F1", MUTED)
        self.y += 6

    def h2(self, t, keep=110):
        self.need(26 + keep)
        self.y += 20
        self.text(self.M, self.y, t, 12, "F2")
        self.y += 5
        self.hline(self.M, self.M + self.Wc, self.y)
        self.y += 8

    def cards(self, items, cols=4, gap=5):
        w = (self.Wc - gap * (cols - 1)) / cols
        for r in range(0, len(items), cols):
            row = items[r:r + cols]
            lines = [wrap(v, 9.5, w - 10, "F2") for _, v in row]
            h = 20 + 11 * max(len(l) for l in lines)
            self.need(h + gap)
            for i, ((k, _), ln) in enumerate(zip(row, lines)):
                x = self.M + i * (w + gap)
                self.rect(x, self.y, w, h, stroke=LINE)
                self.text(x + 5, self.y + 10, k, 7.5, "F1", MUTED)
                for j, l in enumerate(ln):
                    self.text(x + 5, self.y + 21 + 11 * j, l, 9.5, "F2")
            self.y += h + gap

    def table(self, head, rows, widths, size=8, bold_first=False, total=None, checkbox=False):
        tot = float(sum(widths))
        ws = [self.Wc * w / tot for w in widths]
        pad, lh = 3.5, size + 2.2

        def row_h(cells, font):
            return max(len(wrap(c, size, w - 2 * pad, font)) for c, w in zip(cells, ws)) * lh + 2 * pad

        def draw(cells, fill=None, font="F1", first_bold=False, box=False):
            h = row_h(cells, font)
            if self.y + h > self.bottom:
                self.page()
                draw(head, HEAD, "F2")
            x = self.M
            for i, (c, w) in enumerate(zip(cells, ws)):
                self.rect(x, self.y, w, h, fill=fill, stroke=LINE)
                f = "F2" if (font == "F2" or (first_bold and i == 0)) else "F1"
                for j, ln in enumerate(wrap(c, size, w - 2 * pad, f)):
                    self.text(x + pad, self.y + pad + size * 0.85 + j * lh, ln, size, f)
                if box and i == len(cells) - 1 and not c:
                    self.rect(x + pad, self.y + pad, size, size, stroke=INK, lw=0.7)
                x += w
            self.y += h
        self.need(row_h(head, "F2") + 2 * (lh + 2 * pad))
        draw(head, HEAD, "F2")
        for r in rows:
            draw(list(r) + ([""] if checkbox else []), None, "F1", bold_first, checkbox)
        if total:
            draw(total, HEAD, "F2")
        self.y += 4

    def para(self, s, size=8.5, col=INK, font="F1", indent=0):
        for ln in wrap(s, size, self.Wc - indent, font):
            self.need(size + 3)
            self.y += size + 2.5
            self.text(self.M + indent, self.y, ln, size, font, col)

    def pre(self, s, size=7.5):
        maxc = int((self.Wc) / (0.6 * size))
        for raw in s.split("\n"):
            for k in range(0, max(1, len(raw)), maxc):
                self.need(size + 2.5)
                self.y += size + 2
                self.text(self.M, self.y, raw[k:k + maxc], size, "F3")

    def bullets(self, items, size=8.5, col=WARN):
        for it in items:
            lines = wrap(it, size, self.Wc - 12)
            self.need(len(lines) * (size + 2.5) + 2)
            self.y += size + 2.5
            self.text(self.M + 2, self.y, "•", size, "F1", col)
            for j, ln in enumerate(lines):
                if j:
                    self.y += size + 2.5
                self.text(self.M + 12, self.y, ln, size, "F1", col)
            self.y += 2

    def svg(self, svg, width=None, x=None, max_h=None):
        width = width or self.Wc
        for k, v in _ENT.items():
            svg = svg.replace(k, v)
        vb = [float(v) for v in re.search(r'viewBox="([^"]+)"', svg).group(1).split()]
        h = vb[3] * width / vb[2]
        limit = max_h or (self.bottom - self.M - 10)
        if h > limit:
            width *= limit / h
            h = limit
        self.need(h + 4)
        draw_svg(self.pdf, svg, self.M if x is None else x, self.y, width)
        self.y += h + 4
        return h

    def finish(self, path):
        n = len(self.pdf.pages)
        for i, ops in enumerate(self.pdf.pages):
            self.pdf.ops = ops
            label = "%s  ·  page %d of %d" % (self.footer, i + 1, n)
            self.text(self.M, PDF.H - self.M + 4, label, 7, "F1", MUTED)
        self.pdf.save(path)


# --------------------------------------------------------------- the report
def _legend(r, mission):
    from .briefmap import flight_colours, CATEGORICAL, ORDINAL, INK2
    items = []
    n = len(mission.flights)
    if n <= len(CATEGORICAL):
        items += [("line", colour(c), "Flight %d" % (i + 1)) for i, c in enumerate(flight_colours(n))]
    else:
        items.append(("ramp", None, "Flights 1 → %d (numbered on the map)" % n))
    items.append(("dash", colour(INK2), "Transit to / from launch station"))
    from .briefmap import LAUNCH, COVERAGE
    items.append(("dot", colour(LAUNCH), "Take-off / launch station"))
    if getattr(mission, "coverage", None) and mission.coverage.get("rings"):
        items.append(("cov", colour(COVERAGE), "Image coverage (footprint extent)"))
    if getattr(mission, "obstacles", None):
        items.append(("box", colour("#8a8984"), "Obstacle"))
    if getattr(mission, "terrain", None) is not None and mission.terrain.issues:
        items.append(("tri", colour("#d03b3b"), "Terrain / obstacle warning"))
    asp = getattr(mission, "airspace", None)
    if asp is not None:
        from .briefmap import AIRSPACE_COLOURS
        for cat, lab in (("controlled", "Controlled airspace"), ("restricted", "Restricted / prohibited"),
                         ("danger", "Danger / military"), ("uncontrolled", "Uncontrolled airspace")):
            if any(h.airspace.category == cat and h.ring_xy and h.status != "above" for h in asp.hits):
                items.append(("dash", colour(AIRSPACE_COLOURS[cat]), lab))
    x, size = r.M, 7.5
    r.need(14)
    r.y += 9
    for kind, col, label in items:
        w = 22 + text_width(label, size) + 12
        if x + w > r.M + r.Wc:
            x = r.M
            r.y += 11
        yb = PDF.H - r.y + 2.5
        if kind in ("line", "dash"):
            r.pdf.op("q %.3f %.3f %.3f RG %.1f w %s %.2f %.2f m %.2f %.2f l S Q"
                     % (col + (2.2 if kind == "line" else 1.2, "[3 2] 0 d" if kind == "dash" else "",
                               x, yb, x + 16, yb)))
        elif kind == "ramp":
            for k, c in enumerate(ORDINAL):
                r.pdf.op("%.3f %.3f %.3f rg %.2f %.2f 2 4 re f" % (colour(c) + (x + k * 2, yb - 2)))
        elif kind == "dot":
            r.pdf.op("%.3f %.3f %.3f rg %s f" % (col + (_circle(x + 8, yb, 3.6),)))
        elif kind == "cov":
            r.pdf.op("q 0.25 0.25 0.28 RG 3.4 w %.2f %.2f m %.2f %.2f l S %.3f %.3f %.3f RG 2 w %.2f %.2f m %.2f %.2f l S Q"
                     % ((x, yb, x + 16, yb) + col + (x, yb, x + 16, yb)))
        elif kind == "tri":
            r.pdf.op("%.3f %.3f %.3f rg %.2f %.2f m %.2f %.2f l %.2f %.2f l h f"
                     % (col + (x + 4, yb - 3, x + 12, yb - 3, x + 8, yb + 4)))
        else:
            r.pdf.op("%.3f %.3f %.3f rg %.2f %.2f 12 7 re f" % (col + (x + 2, yb - 3.5)))
        r.text(x + 20, r.y + 3, label, size, "F1", INK)
        x += w
    r.y += 8
    r.text(r.M, r.y + 3, "Grid north. Map and plan in the mission's planning coordinate system.", 7, "F1", MUTED)
    r.y += 6


def write_report_pdf(mission, path, to_wgs84, background=None, kp_interval=None):
    from .exporters import brief_content, STATION_HEAD, FLIGHT_HEAD, TERRAIN_HEAD, AIRSPACE_HEAD, mission_title
    from .briefmap import svg_map, map_view, section_views
    m = mission
    d = brief_content(m, to_wgs84, path)
    r = Report(d["title"], "%s · Drone Mission Planner" % d["title"])
    r.title(d["title"], "Drone mission field brief · " + d["sub"])
    r.h2("Summary", 60)
    r.cards(d["summary"])

    fn = background if callable(background) else None
    try:
        view = map_view(m, 900)
        bg = fn(view[0], (900, view[1])) if fn else background
        map_h = min(view[1] * r.Wc / 900.0, 560.0)
        r.h2("Mission map", map_h + 8)              # heading moves with the map
        r.svg(svg_map(m, 900, bg, kp_interval, mission_title(m), view), max_h=560)
        _legend(r, m)
        if m.centreline and len(m.stations) > 1:
            secs = section_views(m, 440, fn)
            if secs:
                r.h2("Section maps", 200)
                gap = 10
                w = (r.Wc - gap) / 2
                for k in range(0, len(secs), 2):
                    pair = secs[k:k + 2]
                    hs = []
                    for cap, svg in pair:
                        vb = [float(v) for v in re.search(r'viewBox="([^"]+)"', svg).group(1).split()]
                        hs.append(vb[3] * w / vb[2])
                    r.need(max(hs) + 16)
                    y0 = r.y
                    for i, (cap, svg) in enumerate(pair):
                        x = r.M + i * (w + gap)
                        r.text(x, y0 + 9, cap, 8.5, "F2")
                        draw_svg(r.pdf, svg, x, y0 + 13, w)
                    r.y = y0 + 13 + max(hs) + 8
    except Exception as e:                       # never lose the report because of the map
        r.para("Map could not be drawn: %s" % e, col=WARN)

    if d["payload"]:
        p = d["payload"]
        r.h2("Payload", 60)
        r.table(["Item", "Weight", "Power"], p["rows"], [5, 3, 2], total=p["total"])
        r.para(p["note"], 7.5, MUTED)
    if d.get("airspace"):
        a = d["airspace"]
        r.h2("Airspace", 40 + 30 * min(len(a["rows"]), 4))
        r.para(a["summary"], 10, font="F2", col=WARN if not a["summary"].startswith(("Uncontrolled", "No airspace"))
               else INK)
        r.y += 3
        if a["rows"]:
            r.table(AIRSPACE_HEAD, a["rows"], [1.2, 2.4, 1.6, 2.0, 0.9, 4.5], size=7.5)
        r.para(a["note"], 7.5, MUTED)
    r.h2("Launch stations", 50)
    r.table(STATION_HEAD, d["stations"], [2, 5, 2.5], bold_first=True)
    r.h2("Flights & battery plan", 60)
    r.table(FLIGHT_HEAD, d["flights"], [0.6, 1.9, 0.6, 1.0, 1.0, 0.8, 1.1, 1.1, 1.3, 1.6, 3.4, 0.9], size=7.5,
            checkbox=True)
    if d["terrain"]:
        t = d["terrain"]
        r.h2("Terrain (DEM)", 60)
        r.pre(t["text"])
        r.y += 4
        if t["rows"]:
            r.table(TERRAIN_HEAD, t["rows"], [3, 1, 1.4, 1.8, 1.8, 1.8], size=7.5)
        if getattr(m, "profile", None):
            from .terrain import svg_profile
            sp = svg_profile(m.profile, unit=getattr(m.params, "height_unit", "m"))
            if sp:
                r.svg(sp)
    r.h2("Warnings & checks", 30)
    r.bullets(d["warnings"])
    r.bullets([d["check"]], col=INK)
    r.finish(path)
    return path
