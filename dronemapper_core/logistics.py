# SPDX-License-Identifier: GPL-2.0-only
"""Battery-change logistics: turns a multi-flight mission into a field schedule.

Simulates the day on site: battery sets (a set = what one flight needs, e.g. a
pair for an M350), chargers, swap time, driving between launch stations and
charging. Gives clock times per flight, waiting for charge, total site time
and the number of battery sets that removes waiting.
"""
import math
from dataclasses import dataclass, field
from typing import List

from . import geometry as G


@dataclass
class LogisticsParams:
    battery_sets: int = 4            # charged sets available at the start
    chargers: int = 2                # sets that can charge at the same time
    charge_min: float = 60.0         # time to fully charge one set
    swap_min: float = 4.0            # land -> swap -> pre-flight checks -> launch
    setup_min: float = 15.0          # site setup before the first flight
    drive_kmh: float = 30.0          # moving between launch stations (vehicle / walking)
    relocate_overhead_min: float = 8.0   # pack, move, set up again at a new station
    start_time_min: float = 8 * 60   # clock time of arrival on site (minutes after midnight)
    cooldown_min: float = 5.0        # let a hot pack cool before charging


@dataclass
class ScheduleRow:
    flight: int
    station_from: str
    station_to: str
    battery_set: int
    start_min: float
    end_min: float
    wait_min: float
    relocate_min: float
    note: str = ""


@dataclass
class Schedule:
    rows: List[ScheduleRow]
    total_min: float
    flying_min: float
    waiting_min: float
    relocating_min: float
    recommended_sets: int
    params: LogisticsParams
    cycles_per_set: dict = field(default_factory=dict)

    def table(self):
        lp = self.params
        out = ["Field schedule (%d battery sets, %d charger%s, %.0f min charge):"
               % (lp.battery_sets, lp.chargers, "" if lp.chargers == 1 else "s", lp.charge_min),
               "  Flt  Launch -> Land    Set  Start  End    Wait  Notes"]
        for r in self.rows:
            out.append("  %3d  %-6s -> %-6s  %3d  %s  %s  %4.0f  %s"
                       % (r.flight, r.station_from, r.station_to, r.battery_set,
                          clock(lp.start_time_min + r.start_min), clock(lp.start_time_min + r.end_min),
                          r.wait_min, r.note))
        out.append("  On site %s -> %s  (%.1f h: %.0f min flying, %.0f min waiting for charge, "
                   "%.0f min relocating)"
                   % (clock(lp.start_time_min), clock(lp.start_time_min + self.total_min),
                      self.total_min / 60, self.flying_min, self.waiting_min, self.relocating_min))
        if self.waiting_min > 1 and self.recommended_sets > lp.battery_sets:
            out.append("  Tip: %d battery sets would remove the charge waiting."
                       % self.recommended_sets)
        return "\n".join(out)


def clock(minutes):
    minutes = int(round(minutes)) % (24 * 60)
    return "%02d:%02d" % divmod(minutes, 60)


def _simulate(flights, labels, lp, sets):
    ready = [lp.setup_min] * max(1, sets)          # every set charged at arrival
    chargers = [0.0] * max(1, lp.chargers)
    cycles = {k + 1: 0 for k in range(len(ready))}
    rows = []
    crew_at, crew_free = None, lp.setup_min
    fly = wait = reloc_total = 0.0
    for f in flights:
        s_from, s_to = labels[f.station_start], labels[f.station_end]
        reloc = 0.0
        if crew_at is not None and crew_at != f.station_start:
            km = G.dist(crew_at, f.station_start) / 1000.0
            reloc = lp.relocate_overhead_min + 60.0 * km / max(lp.drive_kmh, 1.0)
        crew_ready = crew_free + (lp.swap_min if rows else 0.0) + reloc
        k = min(range(len(ready)), key=lambda i: ready[i])
        start = max(crew_ready, ready[k])
        end = start + f.duration_s / 60.0
        w = start - crew_ready
        note = []
        if f.station_end != f.station_start:
            note.append("lands at %s - operator moves with the aircraft (keep VLOS)" % s_to)
        if reloc:
            note.append("drive %.0f min" % reloc)
        if w > 0.5:
            note.append("waiting for charge")
        rows.append(ScheduleRow(f.index + 1, s_from, s_to, k + 1, start, end, w, reloc, "; ".join(note)))
        cycles[k + 1] += 1
        # recharge the set that just flew
        c = min(range(len(chargers)), key=lambda i: chargers[i])
        cs = max(end + lp.cooldown_min, chargers[c])
        ready[k] = cs + lp.charge_min
        chargers[c] = ready[k]
        crew_at, crew_free = f.station_end, end
        fly += end - start
        wait += w
        reloc_total += reloc
    total = rows[-1].end_min + 5.0 if rows else 0.0   # + pack up
    return rows, total, fly, wait, reloc_total, cycles


def build_schedule(flights, stations, lp: LogisticsParams, landing=None):
    """flights: Flight objects with station_start / station_end (x, y) tuples."""
    labels = {}
    for i, s in enumerate(stations):
        labels[tuple(s)] = "S%d" % (i + 1)
    if landing is not None and tuple(landing) not in labels:
        labels[tuple(landing)] = "Land"
    for f in flights:                       # any ad-hoc point
        for s in (f.station_start, f.station_end):
            labels.setdefault(tuple(s), "P%d" % len(labels))
    rows, total, fly, wait, reloc, cycles = _simulate(flights, labels, lp, lp.battery_sets)
    rec = lp.battery_sets
    if wait > 1.0:
        for n in range(lp.battery_sets + 1, 41):
            if _simulate(flights, labels, lp, n)[3] <= 1.0:
                rec = n
                break
        else:
            rec = 40
    return Schedule(rows, total, fly, wait, reloc, rec, lp, cycles), labels
