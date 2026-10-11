# SPDX-License-Identifier: GPL-2.0-only
"""Flight date, daylight and sun angle - pure Python (NOAA solar position).

For the planned flight date and the clock times of the field schedule:

* civil dawn / sunrise / solar noon / sunset / civil dusk at the site;
* the sun's height (elevation) at the start and end of every flight;
* warnings for flights outside daylight (civil twilight - the usual legal
  limit, e.g. CASA standard operating conditions) and in twilight;
* flights with a low sun (long shadows hurt photogrammetry, default < 30 deg);
* the "best light" window when the sun is at least that high.

Accuracy of the solar position is about a minute for sunrise/sunset, which is
plenty for planning; always confirm against your local rules.
"""
import math
import time
import datetime as _dt
from dataclasses import dataclass, field
from typing import List, Optional

HORIZON = -0.833          # sunrise / sunset (refraction + solar radius)
CIVIL = -6.0              # civil twilight


# ------------------------------------------------------------------ solar position
def _julian(t_utc):
    a = (14 - t_utc.month) // 12
    y = t_utc.year + 4800 - a
    m = t_utc.month + 12 * a - 3
    jdn = t_utc.day + (153 * m + 2) // 5 + 365 * y + y // 4 - y // 100 + y // 400 - 32045
    frac = (t_utc.hour - 12) / 24.0 + t_utc.minute / 1440.0 + t_utc.second / 86400.0
    return jdn + frac


def sun_position(lat, lon, t_utc, refraction=True):
    """(elevation_deg, azimuth_deg) of the sun at lat/lon (deg) for a UTC datetime.
    Elevation includes atmospheric refraction; azimuth is clockwise from north."""
    jc = (_julian(t_utc) - 2451545.0) / 36525.0
    L0 = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360
    M = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
    e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)
    Mr = math.radians(M)
    C = (math.sin(Mr) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) + math.sin(2 * Mr) * (0.019993 - 0.000101 * jc)
         + math.sin(3 * Mr) * 0.000289)
    om = math.radians(125.04 - 1934.136 * jc)
    lam = math.radians(L0 + C - 0.00569 - 0.00478 * math.sin(om))
    eps0 = 23 + (26 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60) / 60
    eps = math.radians(eps0 + 0.00256 * math.cos(om))
    decl = math.asin(math.sin(eps) * math.sin(lam))
    y = math.tan(eps / 2) ** 2
    L0r = math.radians(L0)
    eot = 4 * math.degrees(y * math.sin(2 * L0r) - 2 * e * math.sin(Mr) + 4 * e * y * math.sin(Mr) * math.cos(2 * L0r)
                           - 0.5 * y * y * math.sin(4 * L0r) - 1.25 * e * e * math.sin(2 * Mr))
    mins = t_utc.hour * 60 + t_utc.minute + t_utc.second / 60.0
    tst = (mins + eot + 4 * lon) % 1440
    ha = math.radians(tst / 4 + 180 if tst / 4 < 0 else tst / 4 - 180)
    latr = math.radians(lat)
    cosz = math.sin(latr) * math.sin(decl) + math.cos(latr) * math.cos(decl) * math.cos(ha)
    z = math.acos(max(-1.0, min(1.0, cosz)))
    elev = 90 - math.degrees(z)
    # refraction (NOAA)
    if not refraction:
        r = 0
    elif elev > 85:
        r = 0
    elif elev > 5:
        te = math.tan(math.radians(elev))
        r = 58.1 / te - 0.07 / te ** 3 + 0.000086 / te ** 5
    elif elev > -0.575:
        r = 1735 + elev * (-518.2 + elev * (103.4 + elev * (-12.79 + elev * 0.711)))
    else:
        r = -20.772 / math.tan(math.radians(elev))
    elev += r / 3600.0
    sz = math.sin(z)
    if abs(sz) < 1e-9:
        az = 180.0
    else:
        ca = (math.sin(latr) * math.cos(z) - math.sin(decl)) / (math.cos(latr) * sz)
        a = math.degrees(math.acos(max(-1.0, min(1.0, ca))))
        az = (a + 180) % 360 if math.degrees(ha) > 0 else (540 - a) % 360
    return elev, az


# ------------------------------------------------------------------ helpers
def parse_date(s):
    """'2026-10-15' -> date (None if blank / invalid)."""
    if not s:
        return None
    if isinstance(s, _dt.date):
        return s
    try:
        return _dt.datetime.strptime(str(s).strip()[:10], "%Y-%m-%d").date()
    except ValueError:
        return None


def computer_utc_offset(day):
    """UTC offset (hours) of this computer's time zone on `day` (DST aware)."""
    try:
        t = time.mktime((day.year, day.month, day.day, 12, 0, 0, 0, 0, -1))
        return time.localtime(t).tm_gmtoff / 3600.0
    except Exception:
        return -time.timezone / 3600.0


def fmt_offset(h):
    sign = "+" if h >= 0 else "-"
    h = abs(h)
    return "UTC%s%d%s" % (sign, int(h), (":%02d" % round((h - int(h)) * 60)) if h % 1 else "")


def clock(minutes):
    if minutes is None:
        return "-"
    m = int(round(minutes)) % 1440
    return "%02d:%02d" % (m // 60, m % 60)


def elevation_at(lat, lon, day, local_min, utc_offset_h, refraction=True):
    t = _dt.datetime(day.year, day.month, day.day) + _dt.timedelta(minutes=local_min - utc_offset_h * 60)
    return sun_position(lat, lon, t, refraction)[0]


# ------------------------------------------------------------------ the day
@dataclass
class FlightLight:
    flight: int
    start_min: float          # local clock minutes after midnight
    end_min: float
    elev_start: float
    elev_end: float
    elev_min: float
    status: str               # ok | low | twilight | night

    @property
    def sun_text(self):
        a, b = sorted((self.elev_start, self.elev_end))
        if round(a) == round(b):
            s = "%.0f\u00b0" % a
        elif a < 0:
            s = "%.0f\u00b0 to %.0f\u00b0" % (a, b)
        else:
            s = "%.0f\u2013%.0f\u00b0" % (a, b)
        return s + {"ok": "", "low": " low sun", "twilight": " TWILIGHT", "night": " NIGHT"}[self.status]


@dataclass
class DaylightReport:
    date: _dt.date
    utc_offset_h: float
    lat: float
    lon: float
    min_sun_deg: float
    dawn: Optional[float]
    sunrise: Optional[float]
    noon: float
    sunset: Optional[float]
    dusk: Optional[float]
    max_elev: float
    best: Optional[tuple]                 # (start_min, end_min) with sun >= min_sun_deg
    flights: List[FlightLight] = field(default_factory=list)
    offset_suspect: bool = False

    @property
    def date_text(self):
        return "%s (%s)" % (self.date.strftime("%a %d %b %Y"), fmt_offset(self.utc_offset_h))

    @property
    def daylight_text(self):
        if self.sunrise is None and self.max_elev < HORIZON:
            return "no daylight (polar night)"
        if self.sunrise is None:
            return "sun up all day"
        return "sunrise %s, sunset %s (civil twilight %s–%s)" % (
            clock(self.sunrise), clock(self.sunset), clock(self.dawn), clock(self.dusk))

    @property
    def best_text(self):
        if self.best:
            return "%s–%s (sun ≥ %.0f°)" % (clock(self.best[0]), clock(self.best[1]), self.min_sun_deg)
        return "sun never reaches %.0f° (max %.0f° at %s)" % (self.min_sun_deg, self.max_elev, clock(self.noon))

    def text(self):
        out = ["Flight date & light:",
               "  Date              : %s" % self.date_text,
               "  Daylight          : %s" % self.daylight_text,
               "  Solar noon        : %s, sun %.0f° high" % (clock(self.noon), self.max_elev),
               "  Best light        : %s" % self.best_text]
        for f in self.flights:
            out.append("  Flight %-3d %s-%s  sun %s" % (f.flight, clock(f.start_min), clock(f.end_min), f.sun_text))
        return "\n".join(out)


def day_report(lat, lon, day, utc_offset_h, min_sun_deg=30.0, step=2.0):
    """Sun events for a local calendar day (minutes after local midnight)."""
    samples = []
    m = 0.0
    while m <= 1440:
        samples.append((m, elevation_at(lat, lon, day, m, utc_offset_h, refraction=False)))   # -0.833 includes it
        m += step

    def crossings(level):
        up = down = None
        for (m0, e0), (m1, e1) in zip(samples[:-1], samples[1:]):
            if e0 < level <= e1 and up is None:
                up = m0 + (level - e0) / (e1 - e0) * (m1 - m0)
            if e0 >= level > e1:
                down = m0 + (e0 - level) / (e0 - e1) * (m1 - m0)
        return up, down
    sr, ss = crossings(HORIZON)
    dawn, dusk = crossings(CIVIL)
    b0, b1 = crossings(min_sun_deg)
    noon_m, max_e = max(samples, key=lambda s: s[1])
    best = (b0, b1) if b0 is not None and b1 is not None else None
    return DaylightReport(day, utc_offset_h, lat, lon, min_sun_deg, dawn, sr, noon_m, ss, dusk, max_e, best)


def classify(rep, a, b):
    """Status of a flight from local minute a to b."""
    ea = elevation_at(rep.lat, rep.lon, rep.date, a, rep.utc_offset_h)
    eb = elevation_at(rep.lat, rep.lon, rep.date, b, rep.utc_offset_h)
    mids = [elevation_at(rep.lat, rep.lon, rep.date, a + (b - a) * k / 4.0, rep.utc_offset_h) for k in range(1, 4)]
    emin = min([ea, eb] + mids)
    st = "night" if emin < CIVIL else "twilight" if emin < HORIZON else "low" if emin < rep.min_sun_deg else "ok"
    return FlightLight(0, a, b, ea, eb, emin, st)


def analyse(mission, to_wgs84, min_sun_deg=30.0):
    """Attach a DaylightReport to mission.daylight and add warnings.
    Needs a flight date in the logistics parameters (mission.schedule.params)."""
    sc = mission.schedule
    lp = sc.params if sc else None
    day = parse_date(getattr(lp, "flight_date", "")) if lp else None
    if day is None:
        mission.daylight = None
        return None
    lon, lat = to_wgs84(*mission.stations[0])
    off = getattr(lp, "utc_offset_h", None)
    if off is None:
        off = computer_utc_offset(day)
    rep = day_report(lat, lon, day, off, min_sun_deg)
    rep.offset_suspect = abs(off - lon / 15.0) > 2.5
    t0 = lp.start_time_min
    for r in sc.rows:
        fl = classify(rep, t0 + r.start_min, t0 + r.end_min)
        fl.flight = r.flight
        rep.flights.append(fl)
    mission.daylight = rep
    w = mission.warnings
    night = [f.flight for f in rep.flights if f.status == "night"]
    twi = [f.flight for f in rep.flights if f.status == "twilight"]
    low = [f.flight for f in rep.flights if f.status == "low"]
    lst = lambda xs: ", ".join(str(x) for x in xs[:12]) + (" ..." if len(xs) > 12 else "")
    if night:
        w.insert(0, "DAYLIGHT: flight%s %s outside civil twilight (%s–%s) - night flying needs an approval in "
                    "most jurisdictions. Start earlier/later or split the job over two days."
                 % ("s" if len(night) > 1 else "", lst(night), clock(rep.dawn), clock(rep.dusk)))
    if twi:
        w.insert(0 if not night else 1,
                 "DAYLIGHT: flight%s %s in civil twilight (before sunrise %s / after sunset %s) - check your rules "
                 "(anti-collision lighting) and expect dark, noisy images."
                 % ("s" if len(twi) > 1 else "", lst(twi), clock(rep.sunrise), clock(rep.sunset)))
    if low:
        w.append("LIGHT: flight%s %s with the sun below %.0f° - long shadows reduce mapping quality. Best light %s."
                 % ("s" if len(low) > 1 else "", lst(low), min_sun_deg, rep.best_text))
    if rep.offset_suspect:
        w.append("TIME ZONE: %s looks wrong for this site (longitude suggests about %s) - set the site's time "
                 "zone so sunrise/sunset are right." % (fmt_offset(off), fmt_offset(round(lon / 15.0))))
    return rep
