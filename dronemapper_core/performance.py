# SPDX-License-Identifier: GPL-2.0-only
"""Flight-endurance model: wind, temperature, altitude and payload.

The model scales the manufacturer's rated endurance (sea level, 15 C,
no wind, no payload) using first-principles relationships:

* Induced (lifting) power  P ~ m^1.5 / sqrt(rho)  (momentum theory)
  -> heavier take-off mass and thinner air (high altitude / hot day)
     both shorten flight time.
* Battery usable capacity falls in the cold (Li-ion/LiPo chemistry).
* Wind changes the AIRSPEED on every leg of a multirotor mission (ground
  speed is held constant) and the GROUND SPEED of a fixed-wing (airspeed
  held constant); both change the energy each leg costs.
* A safety reserve is always kept back.

It is a planning estimate, not a guarantee. Keep the reserve at 20-30 %
and watch live battery telemetry.
"""
import math
from dataclasses import dataclass, field

RHO0 = 1.225          # kg/m3, ISA sea level 15 C


@dataclass
class PayloadItem:
    """Something carried on the aircraft: camera, lights, LiDAR, gimbal, mount..."""
    name: str
    mass_kg: float = 0.0
    power_w: float = 0.0          # drawn from the flight battery (0 if self-powered)


def _items(items):
    """Accept PayloadItem objects, dicts or (name, kg, W) tuples."""
    out = []
    for it in items or []:
        if isinstance(it, PayloadItem):
            out.append(it)
        elif isinstance(it, dict):
            out.append(PayloadItem(str(it.get("name", "")), float(it.get("mass_kg", 0) or 0),
                                   float(it.get("power_w", 0) or 0)))
        else:
            name, kg = it[0], it[1]
            w = it[2] if len(it) > 2 else 0.0
            out.append(PayloadItem(str(name), float(kg or 0), float(w or 0)))
    return out


@dataclass
class Conditions:
    wind_speed_ms: float = 0.0
    wind_from_deg: float = 0.0       # meteorological: direction wind blows FROM
    temperature_c: float = 15.0
    site_elevation_m: float = 0.0    # take-off elevation above mean sea level
    payload_kg: float = 0.0          # extra mass not itemised below
    reserve_pct: float = 25.0
    battery_health_pct: float = 100.0
    payload_items: list = field(default_factory=list)   # PayloadItem / dict / tuple
    wind_unit: str = "kn"            # display only: kn | m/s | km/h (wind_speed_ms is always m/s)

    @property
    def items(self):
        return _items(self.payload_items)

    @property
    def total_payload_kg(self):
        return self.payload_kg + sum(i.mass_kg for i in self.items)

    @property
    def accessory_power_w(self):
        return sum(i.power_w for i in self.items)


def air_density(altitude_amsl_m, temperature_c):
    p = 101325.0 * (1.0 - 2.25577e-5 * altitude_amsl_m) ** 5.25588
    return p / (287.05 * (temperature_c + 273.15))


def battery_temperature_factor(temp_c):
    """Fraction of rated capacity available at a given cell temperature."""
    table = [(-20, 0.55), (-10, 0.70), (0, 0.85), (10, 0.95), (20, 1.0),
             (35, 1.0), (45, 0.95), (55, 0.88)]
    if temp_c <= table[0][0]:
        return table[0][1]
    for (t1, f1), (t2, f2) in zip(table[:-1], table[1:]):
        if temp_c <= t2:
            return f1 + (f2 - f1) * (temp_c - t1) / (t2 - t1)
    return table[-1][1]


def _wind_vector(c):
    """Wind velocity vector (east, north) in m/s - direction it blows TO."""
    to = math.radians((c.wind_from_deg + 180.0) % 360.0)
    return (c.wind_speed_ms * math.sin(to), c.wind_speed_ms * math.cos(to))


class EnduranceModel:
    def __init__(self, drone, conditions, flight_height_m=0.0):
        self.drone = drone
        self.c = conditions
        self.flight_height_m = flight_height_m
        self.payload_kg = conditions.total_payload_kg
        self.accessory_w = conditions.accessory_power_w
        self.mass = drone.empty_mass_kg + self.payload_kg
        self.rho = air_density(conditions.site_elevation_m + flight_height_m,
                               conditions.temperature_c)
        self.v_best = max(3.0, 0.5 * drone.max_speed_ms)   # best-endurance airspeed
        self.warnings = []
        if self.payload_kg > drone.max_payload_kg + 1e-9:
            self.warnings.append(
                "Payload %.2f kg exceeds the %.2f kg limit for this aircraft."
                % (self.payload_kg, drone.max_payload_kg))
        if self.accessory_w > 0 and drone.battery_wh <= 0:
            self.warnings.append("Accessory power is ignored: set the aircraft's battery capacity (Wh).")
        if conditions.wind_speed_ms > drone.max_wind_ms:
            from .units import wind_fmt
            u = getattr(conditions, "wind_unit", "kn")
            self.warnings.append(
                "Wind %s exceeds the aircraft's %s wind resistance."
                % (wind_fmt(conditions.wind_speed_ms, u), wind_fmt(drone.max_wind_ms, u)))
        if conditions.temperature_c < 5:
            self.warnings.append("Cold batteries: pre-warm packs to 20 C+ before take-off.")

    # -- factors ------------------------------------------------------
    @property
    def mass_factor(self):
        """Power multiplier vs. the rated (no-payload) condition."""
        if self.drone.fixed_wing:
            return (self.mass / self.drone.empty_mass_kg) ** 1.5
        return (self.mass / self.drone.empty_mass_kg) ** 1.5

    @property
    def density_factor(self):
        return math.sqrt(RHO0 / self.rho)

    @property
    def _cap(self):
        """Fraction of the rated battery that may be used (cold, health, reserve)."""
        return (battery_temperature_factor(self.c.temperature_c)
                * self.c.battery_health_pct / 100.0
                * (1.0 - self.c.reserve_pct / 100.0))

    @property
    def flight_minutes(self):
        """Usable minutes counting only the power to fly (still air, best speed)."""
        return self.drone.rated_flight_time_min * self._cap / (self.mass_factor * self.density_factor)

    @property
    def usable_wh(self):
        return max(self.drone.battery_wh, 0.0) * self._cap

    @property
    def flight_power_w(self):
        """Average power to fly at best-endurance speed, implied by the battery and rated time."""
        if self.drone.battery_wh <= 0:
            return 0.0
        return self.usable_wh / max(self.flight_minutes / 60.0, 1e-6)

    @property
    def usable_minutes(self):
        """Usable endurance (min) in still air at best speed, after reserve -
        flight power plus the power drawn by accessories (lights, sensors...)."""
        U = self.flight_minutes
        if self.accessory_w <= 0 or self.usable_wh <= 0:
            return U
        return 1.0 / (1.0 / max(U, 1e-6) + self.accessory_w / (60.0 * self.usable_wh))

    def _air_power_ratio(self, airspeed):
        """Power at airspeed relative to power at best-endurance speed."""
        if self.drone.fixed_wing:
            x = airspeed / max(self.v_best, 0.1)
            return 0.5 * (x ** 3 + 1.0 / max(x, 0.3))
        x = airspeed / self.v_best
        # U-shaped multirotor power curve: hover=1.25, best speed=1.0
        return 1.25 - 0.25 * x * (2.0 - x) if x <= 2.2 else 1.25 + 0.5 * (x - 2.0) ** 2

    # -- legs ---------------------------------------------------------
    def leg(self, a, b, speed_ms):
        """Time (s) and battery fraction for a straight leg a->b in metres.

        For multirotors speed_ms is ground speed; for fixed-wing it is airspeed.
        Returns (seconds, battery_fraction, ground_speed).
        """
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return 0.0, 0.0, speed_ms
        ux, uy = dx / L, dy / L
        wx, wy = _wind_vector(self.c)
        if self.drone.fixed_wing:
            along = wx * ux + wy * uy
            cross = -wx * uy + wy * ux
            va = speed_ms
            gs = math.sqrt(max(va * va - cross * cross, 0.01)) + along
            gs = max(gs, 0.5)
            ratio = self._air_power_ratio(va)
        else:
            gs = speed_ms
            ax, ay = gs * ux - wx, gs * uy - wy     # air-relative velocity
            ratio = self._air_power_ratio(math.hypot(ax, ay))
        t = L / gs
        return t, self._fraction(t, ratio), gs

    def hover(self, seconds):
        ratio = 1.0 if self.drone.fixed_wing else self._air_power_ratio(self.c.wind_speed_ms)
        return self._fraction(seconds, ratio)

    def climb(self, height_m, rate_ms=4.0):
        t = abs(height_m) / rate_ms
        k = 1.35 if height_m > 0 else 0.8
        return t, self._fraction(t, k * (1.0 if self.drone.fixed_wing else 1.25))

    def _fraction(self, seconds, ratio):
        minutes = seconds / 60.0
        f = minutes * ratio / max(self.flight_minutes, 1e-6)
        if self.accessory_w > 0 and self.usable_wh > 0:
            f += self.accessory_w * minutes / (60.0 * self.usable_wh)
        return f

    # -- optimisation -------------------------------------------------
    def best_speed(self, headings_deg, v_max, v_min=2.0):
        """Ground speed that minimises energy per metre over the given
        line headings (both directions), capped at v_max."""
        best = (float("inf"), v_min)
        v = v_min
        while v <= v_max + 1e-9:
            cost = 0.0
            for h in headings_deg:
                for hh in (h, h + 180.0):
                    r = math.radians(hh)
                    _, frac, _ = self.leg((0, 0), (math.sin(r) * 100, math.cos(r) * 100), v)
                    cost += frac
            if cost < best[0]:
                best = (cost, v)
            v += 0.5
        return best[1]

    def summary(self):
        return {
            "take_off_mass_kg": round(self.mass, 3),
            "air_density_kg_m3": round(self.rho, 4),
            "battery_temp_factor": round(battery_temperature_factor(self.c.temperature_c), 3),
            "usable_minutes_still_air": round(self.usable_minutes, 1),
            "minutes_without_accessories": round(self.flight_minutes, 1),
            "rated_minutes": self.drone.rated_flight_time_min,
            "payload_kg": round(self.payload_kg, 3),
            "accessory_w": round(self.accessory_w, 1),
            "flight_power_w": round(self.flight_power_w, 0),
        }
