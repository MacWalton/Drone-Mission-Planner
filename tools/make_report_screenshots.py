# SPDX-License-Identifier: GPL-2.0-only
"""Build two sample field briefs (mission reports) and screenshot their sections.

    python tools/make_report_screenshots.py      (needs pyproj + playwright/Chromium)

Writes docs/screenshots/report_*.png and docs/examples/sample_*_brief.html.
Sample data: a synthetic site near Perth, WA (MGA/UTM 50S) - not a real survey.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from pyproj import Transformer                                    # noqa: E402
from dronemapper_core import *                                    # noqa: E402,F401,F403
from dronemapper_core import exporters, terrain as TR             # noqa: E402

SHOTS = os.path.join(ROOT, "docs", "screenshots")
EX = os.path.join(ROOT, "docs", "examples")
t = Transformer.from_crs(32750, 4326, always_xy=True)
to84 = lambda x, y: t.transform(x, y)                             # noqa: E731
x0, y0 = 381000, 6426000


def area_brief():
    d = get_drone("DJI Matrice 4E")
    blk = [[(x0, y0), (x0 + 1500, y0), (x0 + 1500, y0 + 700), (x0 + 1100, y0 + 1000), (x0, y0 + 1000)]]

    def ground(x, y):                                             # gentle ridge rising to the east
        return 22 + 0.012 * (x - x0) + 14 * math.exp(-((x - (x0 + 800)) / 260.0) ** 2)
    obs = [Obstacle("line", [(x0 - 150, y0 + 420), (x0 + 1650, y0 + 470)], 28, "33 kV powerline"),
           Obstacle("polygon", [[(x0 + 950, y0 + 120), (x0 + 1040, y0 + 120), (x0 + 1040, y0 + 190),
                                 (x0 + 950, y0 + 190)]], 14, "Machinery shed"),
           Obstacle("polygon", [[(x0 + 150, y0 + 720), (x0 + 380, y0 + 720), (x0 + 380, y0 + 930),
                                 (x0 + 150, y0 + 930)]], 22, "Tree stand")]
    cond = Conditions(wind_speed_ms=6, wind_from_deg=220, temperature_c=31, site_elevation_m=22,
                      payload_items=[PayloadItem("RTK module", 0.11, 3), PayloadItem("Spotlight", 0.09, 15)])
    mp = MissionParams(gsd_cm=2.0, front_overlap_pct=75, side_overlap_pct=65, min_agl_m=45,
                       mission_name="Lot 12 Stockpile & Paddock Survey", terrain_follow=True,
                       rth_height_m=60, max_range_m=900)
    sts = suggest_area_stations(blk, mp.max_range_m)
    m = plan_mission(mp, d, cond, blk, sts[0], stations=sts[1:],
                     ground_elevation=ground, obstacles=obs,
                     logistics=LogisticsParams(battery_sets=4, chargers=2, start_time_min=7 * 60 + 30,
                                               flight_date="2026-10-15", utc_offset_h=8))
    TR.analyse(m, ground, 44.5, 120)
    from dronemapper_core import daylight
    daylight.analyse(m, to84)
    return m


def route_brief():
    d = get_drone("DJI Matrice 350 RTK + P1")
    road = [(x0 + i * 120, y0 + 2000 + 260 * math.sin(i / 7.0)) for i in range(70)]    # ~8.5 km

    def hill(x, y):
        return 30 + 55 * math.exp(-((x - (x0 + 3800)) / 900.0) ** 2) + 0.003 * (x - x0)
    mp = MissionParams(mission_type=ROUTE, gsd_cm=1.1, left_width_m=40, right_width_m=30, terrain_follow=True,
                       max_range_m=1200, mission_name="Gas pipeline KP 0-8.5 corridor survey")
    cond = Conditions(wind_speed_ms=5, wind_from_deg=250, temperature_c=24)
    sts = suggest_stations(mp, d, cond, road)
    m = plan_mission(mp, d, cond, road, sts[0], stations=sts[1:], ground_elevation=hill,
                     logistics=LogisticsParams(battery_sets=6, chargers=3, start_time_min=7 * 60,
                                               flight_date="2026-10-16", utc_offset_h=8))
    TR.analyse(m, hill, 30, 120)
    from dronemapper_core import daylight
    daylight.analyse(m, to84)
    return m


def shoot(html, prefix, sections):
    from playwright.sync_api import sync_playwright
    exe = "/opt/pw-browsers/chromium" if os.path.exists("/opt/pw-browsers/chromium") else None
    with sync_playwright() as p:
        b = p.chromium.launch(executable_path=exe)
        pg = b.new_page(viewport={"width": 1040, "height": 900}, device_scale_factor=2)
        pg.goto("file://" + html)
        for name, first, last in sections:
            top = pg.locator(first).first.bounding_box()
            if last == "+map":
                bot = pg.locator(first + " ~ svg, " + first + " ~ div").first.bounding_box()
                y1 = bot["y"] + bot["height"] + 60
            elif last:
                y1 = pg.locator(last).first.bounding_box()["y"] - 8
            else:
                y1 = pg.evaluate("document.body.scrollHeight")
            y0_ = 0 if first == "body" else top["y"] - 10
            pg.screenshot(path=os.path.join(SHOTS, "%s_%s.png" % (prefix, name)), full_page=True,
                          clip={"x": 0, "y": y0_, "width": 1040, "height": min(y1 - y0_, 1500)})
        b.close()


if __name__ == "__main__":
    os.makedirs(SHOTS, exist_ok=True)
    a = os.path.join(EX, "sample_area_brief.html")
    r = os.path.join(EX, "sample_route_brief.html")
    am, rm = area_brief(), route_brief()
    exporters.export(am, "brief", a, to84)
    exporters.export(rm, "brief", r, to84, kp_interval=1000)
    ap = os.path.join(EX, "sample_area_report.pdf")
    exporters.export(am, "brief_pdf", ap, to84)
    exporters.export(rm, "brief_pdf", os.path.join(EX, "sample_route_report.pdf"), to84, kp_interval=1000)
    import shutil
    import subprocess
    if shutil.which("pdftoppm"):
        subprocess.run(["pdftoppm", "-r", "110", "-png", "-f", "1", "-l", "1", "-singlefile", ap,
                        os.path.join(SHOTS, "report_pdf_page1")], check=True)
    shoot(a, "report_area", [("summary", "body", "h2:has-text('Mission map')"),
                             ("map", "h2:has-text('Mission map')", "h2:has-text('Launch stations')"),
                             ("flights", "h2:has-text('Launch stations')", "h2:has-text('Terrain')"),
                             ("terrain", "h2:has-text('Terrain')", None)])
    shoot(r, "report_route", [("map", "h2:has-text('Mission map')", "h2:has-text('Section maps')"),
                              ("sections", "h2:has-text('Section maps')", "h2:has-text('Launch stations')"),
                              ("profile", "h2:has-text('Terrain')", "h2:has-text('Warnings')")])
    print("\n".join(sorted(f for f in os.listdir(SHOTS) if f.startswith("report_"))))
