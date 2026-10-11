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
