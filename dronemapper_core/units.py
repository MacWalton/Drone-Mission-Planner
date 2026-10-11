# SPDX-License-Identifier: GPL-2.0-only
"""Height units. Everything is computed and exported in metres (drones expect
metres); feet are only for entering and displaying heights."""

FT_PER_M = 1.0 / 0.3048          # international foot
UNITS = {"m": "m", "ft": "ft"}


def to_m(value, unit):
    if value is None:
        return None
    return float(value) * 0.3048 if unit == "ft" else float(value)


def from_m(metres, unit):
    if metres is None:
        return None
    return float(metres) * FT_PER_M if unit == "ft" else float(metres)


def fmt(metres, unit="m", dec=0, both=False):
    """'100 m' or '328 ft (100 m)' when both=True and unit is feet."""
    if metres is None or metres != metres:
        return "-"
    v = from_m(metres, unit)
    s = "%.*f %s" % (dec, v, unit)
    if both and unit == "ft":
        s += " (%.*f m)" % (dec, metres)
    return s


LB_PER_KG = 1.0 / 0.45359237


def kg(value, unit):
    """Mass in kg from 'kg' or 'lb'."""
    return float(value or 0) * 0.45359237 if unit == "lb" else float(value or 0)


def mass_fmt(kgs, unit="kg", dec=2):
    if unit == "lb":
        return "%.*f lb (%.*f kg)" % (dec, kgs * LB_PER_KG, dec, kgs)
    return "%.*f kg" % (dec, kgs)


# ---- wind speed: computed in m/s, entered and shown in knots by default
WIND_UNITS = {"kn": 0.514444, "m/s": 1.0, "km/h": 1.0 / 3.6}
WIND_LABELS = {"kn": "knots", "m/s": "m/s", "km/h": "km/h"}


def wind_to_ms(value, unit="kn"):
    return float(value or 0) * WIND_UNITS.get(unit, 1.0)


def wind_from_ms(ms, unit="kn"):
    return float(ms or 0) / WIND_UNITS.get(unit, 1.0)


def wind_fmt(ms, unit="kn", both=True):
    """'12 kn (6.2 m/s)' - knots / km/h also show m/s so the numbers can be checked."""
    v = wind_from_ms(ms, unit)
    s = "%.*f %s" % (0 if unit != "m/s" else 1, v, unit)
    if both and unit != "m/s":
        s += " (%.1f m/s)" % ms
    return s
