# SPDX-License-Identifier: GPL-2.0-only
"""Mission overview map for the field brief, as inline SVG (no dependencies).

Draws the boundary / corridor, every flight (survey lines solid, transits to
and from the launch station dashed), flight numbers, launch stations, final
landing, terrain warnings, route chainage, a scale bar and a grid-north arrow.
An optional raster background (e.g. aerial imagery rendered by QGIS for the
exact same extent) can be placed underneath.

Colour: <= 8 flights -> categorical slots in fixed order; more -> one ordinal
blue ramp in flight order (light = early, dark = late). Every flight is also
numbered on the map, so identity never depends on colour alone.
"""
import base64
import math
from xml.sax.saxutils import escape

from . import geometry as G

CATEGORICAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
ORDINAL = ["#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6", "#256abf", "#1c5cab",
           "#184f95", "#104281", "#0d366b"]               # blue 250 -> 700
CRITICAL, SERIOUS = "#d03b3b", "#ec835a"                 # status: low clearance / above limit
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8a8984"
SURFACE = "#fcfcfb"


def flight_colours(n):
    if n <= len(CATEGORICAL):
        return CATEGORICAL[:n]
    return [ORDINAL[min(len(ORDINAL) - 1, int(i * len(ORDINAL) / n))] for i in range(n)]


def map_view(mission, width=900, pad_frac=0.05, focus=None, aspect_range=(0.3, 1.15)):
    """Extent (x0, y0, x1, y1) in the planning CRS and pixel height for the map.
    focus = list of points to frame instead of the whole mission.
    Callers that render a background must render exactly this extent."""
    if focus:
        pts = list(focus)
    else:
        pts = [p for ring in mission.boundary for p in ring]
        pts += [(w.x, w.y) for f in mission.flights for w in f.waypoints]
        pts += [tuple(s) for s in mission.stations] + [tuple(mission.landing)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    w, h = max(x1 - x0, 1.0), max(y1 - y0, 1.0)
    pad = max(w, h) * pad_frac
    x0, x1, y0, y1 = x0 - pad, x1 + pad, y0 - pad, y1 + pad
    w, h = x1 - x0, y1 - y0
    aspect = min(max(h / w, aspect_range[0]), aspect_range[1])   # keep the page readable
    height = int(round(width * aspect))
    want_h = w * height / width
    if want_h > h:
        d = (want_h - h) / 2
        y0, y1 = y0 - d, y1 + d
    else:
        want_w = h * width / height
        d = (want_w - w) / 2
        x0, x1 = x0 - d, x1 + d
    return (x0, y0, x1, y1), height


def _nice(v):
    e = 10 ** math.floor(math.log10(v))
    for m in (1, 2, 5, 10):
        if m * e >= v:
            return m * e
    return 10 * e


def svg_map(mission, width=900, background=None, kp_interval=None, title=None, view=None,
            highlight=None):
    """SVG markup. background = PNG/JPEG bytes rendered for the same view, or a
    basemap.Background (whole image or positioned tiles + attribution).
    view = (extent, height) from map_view(); highlight = set of flight indexes
    to emphasise (others are faded) for section maps."""
    (x0, y0, x1, y1), H = view or map_view(mission, width)
    W = width
    sx = W / (x1 - x0)

    def X(x):
        return (x - x0) * sx

    def Y(y):
        return (y1 - y) * sx

    def pts(seq):
        return " ".join("%.1f,%.1f" % (X(p[0]), Y(p[1])) for p in seq)

    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="100%%" '
           'style="max-width:%dpx;display:block;background:%s;font-family:-apple-system,Segoe UI,'
           'Roboto,sans-serif" role="img" aria-label="%s">'
           % (W, H, W, SURFACE, escape(title or "Mission map"))]
    out.append('<style>.lbl{paint-order:stroke;stroke:#fff;stroke-width:3px;stroke-linejoin:round}'
               '.fl:hover .ln{stroke-width:4}</style>')
    bg_attr = ""
    if not background:
        background = None
    if background is not None and not isinstance(background, (bytes, bytearray)):
        bg_attr = getattr(background, "attribution", "") or ""
        op = getattr(background, "opacity", 0.9)
        if getattr(background, "tiles", None):
            out.append('<g opacity="%.2f">' % op)
            for data, (tl, tr, bl) in background.tiles:
                ax, ay, bx_, by_, cx_, cy_ = X(tl[0]), Y(tl[1]), X(tr[0]), Y(tr[1]), X(bl[0]), Y(bl[1])
                k = 255.4                          # tiny overlap hides anti-aliased seams
                out.append('<image href="data:%s;base64,%s" width="256" height="256" '
                           'preserveAspectRatio="none" transform="matrix(%.5f %.5f %.5f %.5f %.2f %.2f)"/>'
                           % ("image/jpeg" if data[:2] == b"\xff\xd8" else "image/png",
                              base64.b64encode(data).decode(),
                              (bx_ - ax) / k, (by_ - ay) / k, (cx_ - ax) / k, (cy_ - ay) / k, ax, ay))
            out.append('</g>')
            background = None
        else:
            background = getattr(background, "data", None)
    else:
        op = 0.9
    if background:
        out.append('<image href="data:%s;base64,%s" x="0" y="0" width="%d" height="%d" '
                   'preserveAspectRatio="none" opacity="%.2f"/>'
                   % ("image/jpeg" if background[:2] == b"\xff\xd8" else "image/png",
                      base64.b64encode(background).decode(), W, H, op))
    background = background or bool(bg_attr) or None
    # boundary / corridor
    for k, ring in enumerate(mission.boundary):
        out.append('<polygon points="%s" fill="%s" fill-opacity="%s" stroke="%s" stroke-width="1.5"/>'
                   % (pts(ring), "#eda100" if k == 0 else SURFACE, "0.10" if k == 0 else "0.8", INK2))
    if mission.centreline:
        out.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.5" '
                   'stroke-dasharray="6 4"/>' % (pts(mission.centreline), INK2))
    obs = getattr(mission, "obstacles", None)
    if obs:
        out.append('<defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" '
                   'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="6" stroke="%s" '
                   'stroke-width="1.5"/></pattern></defs>' % INK2)
        for o in obs.obs[:3000]:
            tip = "<title>%s %.0f m</title>" % (escape(o.name or "Obstacle"), o.height_m)
            if o.kind == "polygon":
                out.append('<polygon points="%s" fill="url(#hatch)" fill-opacity="0.7" stroke="%s" '
                           'stroke-width="1">%s</polygon>' % (pts(o.coords[0]), INK2, tip))
            elif o.kind == "line":
                out.append('<polyline points="%s" fill="none" stroke="%s" stroke-width="2.5" '
                           'stroke-dasharray="1 4" stroke-linecap="round">%s</polyline>' % (pts(o.coords), INK, tip))
            else:
                cx, cy = X(o.coords[0][0]), Y(o.coords[0][1])
                out.append('<g>%s<circle cx="%.1f" cy="%.1f" r="4" fill="#fff" stroke="%s" stroke-width="2"/>'
                           '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.5"/></g>'
                           % (tip, cx, cy, INK, cx - 3, cy - 3, cx + 3, cy + 3, INK))
    # flights
    cols = flight_colours(len(mission.flights))
    sched = {r.flight: r for r in (mission.schedule.rows if mission.schedule else [])}
    labels = []
    for f, col in zip(mission.flights, cols):
        w = f.waypoints
        faded = highlight is not None and f.index not in highlight
        tip = "Flight %d: %s to %s, %.1f min, %.0f%% battery, %d photos" % (
            f.index + 1, mission.label(f.station_start), mission.label(f.station_end),
            f.duration_s / 60, f.battery_fraction * 100, f.photos)
        if f.index + 1 in sched:
            r = sched[f.index + 1]
            from .logistics import clock
            t0 = mission.schedule.params.start_time_min
            tip += ", %s-%s, battery set %d" % (clock(t0 + r.start_min), clock(t0 + r.end_min), r.battery_set)
        out.append('<g class="fl"%s><title>%s</title>' % (' opacity="0.25"' if faded else "", escape(tip)))
        survey, transit = [], []
        for a, b in zip(w[:-1], w[1:]):
            (survey if a.photo_dist > 0 else transit).append(((a.x, a.y), (b.x, b.y)))
        if highlight is not None:                # section map: skip what is outside the frame
            mx, my = (x1 - x0) * 0.1, (y1 - y0) * 0.1
            inside = lambda sg: not (max(sg[0][0], sg[1][0]) < x0 - mx or min(sg[0][0], sg[1][0]) > x1 + mx or
                                     max(sg[0][1], sg[1][1]) < y0 - my or min(sg[0][1], sg[1][1]) > y1 + my)
            survey = [sg for sg in survey if inside(sg)]
            transit = [sg for sg in transit if inside(sg)]
        for a, b in transit:
            out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.5" '
                       'stroke-dasharray="4 3" opacity="0.85"/>' % (X(a[0]), Y(a[1]), X(b[0]), Y(b[1]), col))
        if survey:
            d = " ".join("M%.1f %.1fL%.1f %.1f" % (X(a[0]), Y(a[1]), X(b[0]), Y(b[1])) for a, b in survey)
            if background:                       # 2px surface ring keeps lines legible on imagery
                out.append('<path d="%s" fill="none" stroke="#fff" stroke-width="4" stroke-opacity="0.85" '
                           'stroke-linecap="round"/>' % d)
            out.append('<path class="ln" d="%s" fill="none" stroke="%s" stroke-width="2" '
                       'stroke-linecap="round"/>' % (d, col))
        out.append('</g>')
        # label position: along the survey part, avoiding other labels
        segs = survey or transit
        # only label flights that are emphasised and actually inside this view
        if segs and highlight is not None:
            segs = [sg for sg in segs if x0 <= sg[0][0] <= x1 and y0 <= sg[0][1] <= y1] if not faded else []
        if segs:
            lens = [G.dist(a, b) for a, b in segs]
            tot = sum(lens) or 1.0
            pick = None
            for frac in (0.5, 0.3, 0.7, 0.15, 0.85, 0.4, 0.6):
                run, target = 0.0, frac * tot
                for (a, b), L in zip(segs, lens):
                    if run + L >= target:
                        t = (target - run) / max(L, 1e-9)
                        q = (X(a[0] + (b[0] - a[0]) * t), Y(a[1] + (b[1] - a[1]) * t))
                        break
                    run += L
                if all(math.hypot(q[0] - p[0], q[1] - p[1]) > 20 for p in labels):
                    pick = q
                    break
            pick = pick or q
            labels.append(pick)
            out.append('<circle cx="%.1f" cy="%.1f" r="9" fill="#fff" stroke="%s" stroke-width="2"/>'
                       '<text x="%.1f" y="%.1f" font-size="10" font-weight="700" text-anchor="middle" '
                       'fill="%s">%d</text>' % (pick[0], pick[1], col, pick[0], pick[1] + 3.5, INK, f.index + 1))
    # chainage
    if mission.centreline:
        L = G.line_length(mission.centreline)
        step = kp_interval or _nice(L / 8.0)
        if highlight is not None and not kp_interval:       # section map: local spacing
            step = _nice((x1 - x0) / 5.0)
        # step so labels are >= ~70 px apart at this scale
        step = max(step, _nice(70.0 / sx))
        for x, y, c in mission.chainage_markers(step):
            cx, cy = X(x), Y(y)
            if not (0 <= cx <= W and 0 <= cy <= H):
                continue
            out.append('<circle cx="%.1f" cy="%.1f" r="2.5" fill="%s"/>' % (cx, cy, INK2))
            for dx, dy in ((5, -7), (5, 14), (-48, -7), (-48, 14)):
                lx, ly = cx + dx, cy + dy
                if all(math.hypot(lx + 20 - p[0], ly - 4 - p[1]) > 24 for p in labels) and 0 < lx < W - 45:
                    out.append('<text class="lbl" x="%.1f" y="%.1f" font-size="10" fill="%s">KP %g</text>'
                               % (lx, ly, INK2, round(c / 1000.0, 3)))
                    labels.append((lx + 20, ly - 4))
                    break
    # terrain warnings
    rep = getattr(mission, "terrain", None)
    if rep is not None:
        for i in rep.issues[:200]:
            cx, cy = X(i.x), Y(i.y)
            if i.kind in ("low", "obstacle"):
                out.append('<polygon points="%.1f,%.1f %.1f,%.1f %.1f,%.1f" fill="%s" stroke="#fff" '
                           'stroke-width="1"><title>Low clearance: %.0f m (flight %d)</title></polygon>'
                           % (cx - 6, cy - 5, cx + 6, cy - 5, cx, cy + 6, CRITICAL, i.value_m, i.flight))
            else:
                out.append('<polygon points="%.1f,%.1f %.1f,%.1f %.1f,%.1f" fill="%s" stroke="#fff" '
                           'stroke-width="1"><title>%.0f m above ground (flight %d)</title></polygon>'
                           % (cx - 6, cy + 5, cx + 6, cy + 5, cx, cy - 6, SERIOUS, i.value_m, i.flight))
    # stations & landing
    for st in mission.stations:
        cx, cy = X(st[0]), Y(st[1])
        n = sum(1 for f in mission.flights if tuple(f.station_start) == tuple(st))
        out.append('<polygon points="%.1f,%.1f %.1f,%.1f %.1f,%.1f" fill="%s" stroke="#fff" stroke-width="1.5">'
                   '<title>%s: %d flight(s)</title></polygon><text class="lbl" x="%.1f" y="%.1f" '
                   'font-size="12" font-weight="700" fill="%s">%s</text>'
                   % (cx, cy - 9, cx - 8, cy + 6, cx + 8, cy + 6, INK, mission.label(st), n,
                      cx + 10, cy + 4, INK, mission.label(st)))
    if tuple(mission.landing) not in [tuple(s) for s in mission.stations]:
        cx, cy = X(mission.landing[0]), Y(mission.landing[1])
        out.append('<rect x="%.1f" y="%.1f" width="12" height="12" fill="%s" stroke="#fff" stroke-width="1.5"/>'
                   '<text class="lbl" x="%.1f" y="%.1f" font-size="12" font-weight="700" fill="%s">Land</text>'
                   % (cx - 6, cy - 6, INK2, cx + 10, cy + 4, INK))
    # scale bar (bottom left) & north arrow (top right)
    target = (x1 - x0) * 0.22
    sb = _nice(target)
    if sb > target * 1.6:
        sb /= 2
    px = sb * sx
    lab = ("%g km" % (sb / 1000)) if sb >= 1000 else ("%g m" % sb)
    bx, by = 14, H - 18
    out.append('<rect x="%d" y="%d" width="%.1f" height="22" fill="#fff" fill-opacity="0.8" rx="3"/>'
               % (bx - 6, by - 14, px + 50))
    out.append('<rect x="%d" y="%d" width="%.1f" height="5" fill="%s"/><rect x="%d" y="%d" width="%.1f" '
               'height="5" fill="#fff" stroke="%s" stroke-width="1"/>'
               % (bx, by, px / 2, INK, bx + px / 2, by, px / 2, INK))
    out.append('<text x="%.1f" y="%d" font-size="11" fill="%s">%s</text>' % (bx + px + 6, by + 6, INK, lab))
    nx, ny = W - 26, 18
    out.append('<g><circle cx="%d" cy="%d" r="15" fill="#fff" fill-opacity="0.8"/>'
               '<polygon points="%d,%d %d,%d %d,%d" fill="%s"/><text x="%d" y="%d" font-size="10" '
               'font-weight="700" text-anchor="middle" fill="%s">N</text></g>'
               % (nx, ny + 8, nx, ny - 6, nx - 6, ny + 10, nx + 6, ny + 10, INK, nx, ny + 21, INK))
    if bg_attr:                                   # provider attribution, bottom right
        tw = 5.6 * len(bg_attr) + 10
        out.append('<rect x="%.1f" y="%d" width="%.1f" height="15" fill="#fff" fill-opacity="0.8"/>'
                   '<text x="%d" y="%d" font-size="9.5" text-anchor="end" fill="%s">%s</text>'
                   % (W - tw, H - 15, tw, W - 5, H - 4, INK2, escape(bg_attr)))
    out.append("</svg>")
    return "".join(out)


def legend_html(mission):
    """HTML legend that wraps under the map (identity is also on the map labels)."""
    n = len(mission.flights)
    cols = flight_colours(n)
    sw = lambda c, dash=False: ('<span style="display:inline-block;width:22px;height:0;border-top:%s %s;'
                                'vertical-align:middle;margin-right:5px"></span>'
                                % ("2px dashed" if dash else "3px solid", c))
    items = []
    if n <= len(CATEGORICAL):
        items += ["%sFlight %d" % (sw(c), i + 1) for i, c in enumerate(cols)]
    else:
        grad = "linear-gradient(90deg,%s,%s)" % (ORDINAL[0], ORDINAL[-1])
        items.append('<span style="display:inline-block;width:60px;height:6px;background:%s;'
                     'vertical-align:middle;margin-right:5px;border-radius:3px"></span>Flights 1 &rarr; %d '
                     '(numbered on the map)' % (grad, n))
    items.append("%sTransit to / from launch station" % sw(INK2, True))
    items.append('<span style="color:%s;margin-right:4px">&#9650;</span>Launch station' % INK)
    if getattr(mission, "obstacles", None):
        items.append('<span style="display:inline-block;width:14px;height:10px;margin-right:5px;'
                     'background:repeating-linear-gradient(45deg,%s 0 1.5px,transparent 1.5px 5px);'
                     'border:1px solid %s;vertical-align:middle"></span>Obstacle (hover for height)' % (INK2, INK2))
    if getattr(mission, "terrain", None) is not None and mission.terrain.issues:
        items.append('<span style="color:%s;margin-right:4px">&#9660;</span>Low clearance' % CRITICAL)
        if any(i.kind == "high" for i in mission.terrain.issues):
            items.append('<span style="color:%s;margin-right:4px">&#9650;</span>Above height limit' % SERIOUS)
    return ('<div style="display:flex;flex-wrap:wrap;gap:6px 16px;font-size:12px;color:%s;margin-top:6px">%s'
            '</div><div style="font-size:11px;color:%s;margin-top:2px">Grid north. Hover a flight for '
            'details.</div>' % (INK2, "".join("<span>%s</span>" % i for i in items), MUTED))


def section_views(mission, width=440, background_fn=None):
    """[(caption, svg)] - one zoomed map per launch station (routes with several
    stations): that station's flights emphasised, the rest faded.
    background_fn(extent, (w, h)) -> background or None."""
    out = []
    for k, st in enumerate(mission.stations):
        idx = {f.index for f in mission.flights if tuple(f.station_start) == tuple(st)}
        if not idx:
            continue
        focus = [tuple(st)] + [(w.x, w.y) for f in mission.flights if f.index in idx
                                for w in f.waypoints]
        view = map_view(mission, width, 0.08, focus, (0.55, 0.9))
        bg = background_fn(view[0], (width, view[1])) if background_fn else None
        names = ", ".join(str(i + 1) for i in sorted(idx))
        kp = [mission.flights[i].chainage for i in sorted(idx) if mission.flights[i].chainage]
        kp_txt = (" \u00b7 KP %.2f\u2013%.2f" % (min(a for a, _ in kp) / 1000, max(b for _, b in kp) / 1000)) if kp else ""
        cap = "%s \u00b7 flight%s %s%s" % (mission.label(st), "s" if len(idx) > 1 else "", names, kp_txt)
        out.append((cap, svg_map(mission, width, bg, None, "%s section" % mission.label(st), view, idx)))
    return out


def section_maps(mission, width=440, background_fn=None):
    """HTML block of section maps (see section_views)."""
    out = ['<div class="sec"><div style="font-weight:600;font-size:13px;margin:0 0 4px">%s</div>%s</div>'
           % (escape(cap), svg) for cap, svg in section_views(mission, width, background_fn)]
    if not out:
        return ""
    return ('<h2>Section maps</h2><div style="display:grid;grid-template-columns:repeat(auto-fill,'
            'minmax(%dpx,1fr));gap:14px">%s</div>' % (min(width, 420), "".join(out)))
