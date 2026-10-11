# SPDX-License-Identifier: GPL-2.0-only
"""Photo positions and image footprints along the flights actually planned.

Photos are placed along each flight's survey legs at the trigger distance
(continuing across the extra waypoints of terrain-following lines and
restarting at every line start); in stop-and-shoot mode the shoot waypoints are
used. Each footprint is the camera's ground coverage at that photo:

* size from the camera and the height above ground there - with a DEM the real
  ground under the photo, otherwise ground level with the launch station;
* turned to the flight direction (portrait setting respected);
* nadir camera assumed (gimbal pitch -90 deg). Oblique shots are flagged.
"""
import math

from . import photogrammetry as P


def photo_positions(mission):
    """[(flight_index, x, y, heading_rad, alt_above_launch, z_ref)]"""
    out = []
    trig = mission.trigger_m
    stop = getattr(mission.params, "photo_trigger", "distance") == "stop"
    for f in mission.flights:
        wps = f.waypoints
        if stop and any(getattr(w, "shoot", False) for w in wps):
            for i, w in enumerate(wps):
                if getattr(w, "shoot", False):
                    nb = wps[i + 1] if i + 1 < len(wps) else wps[i - 1]
                    hd = math.atan2(nb.x - w.x, nb.y - w.y) if (nb.x, nb.y) != (w.x, w.y) else 0.0
                    if nb is wps[i - 1] and i + 1 >= len(wps):
                        hd += math.pi
                    out.append((f.index, w.x, w.y, hd, w.alt, f.z_ref))
            continue
        carry = None                                    # distance to the next photo in this run
        for a, b in zip(wps[:-1], wps[1:]):
            if a.photo_dist <= 0:
                carry = None
                continue
            if a.kind in ("line_start", "resume") or carry is None:
                carry = 0.0
            L = math.hypot(b.x - a.x, b.y - a.y)
            if L <= 1e-9:
                continue
            hd = math.atan2(b.x - a.x, b.y - a.y)
            d = carry
            while d <= L + 1e-6:
                t = d / L
                out.append((f.index, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, hd,
                            a.alt + (b.alt - a.alt) * t, f.z_ref))
                d += trig
            carry = d - L
    return out


def photo_footprints(mission, ground=None):
    """One dict per photo: flight, n, x, y, agl_m, gsd_cm, across_m, along_m, ring.

    ground: f(x, y) -> m AMSL (defaults to the DEM the mission was planned with)."""
    ground = ground or getattr(mission, "ground", None)
    cam = mission.drone.camera
    portrait = bool(getattr(mission.params, "portrait", False))
    out = []
    for n, (fi, x, y, hd, alt, zref) in enumerate(photo_positions(mission)):
        agl = alt
        if ground is not None:
            try:
                g = ground(x, y)
                if g is not None:
                    agl = alt + zref - g
            except Exception:
                pass
        agl = max(agl, 1.0)
        across, along = P.footprint(cam, agl, portrait)
        ux, uy = math.sin(hd), math.cos(hd)              # along track
        vx, vy = uy, -ux                                  # across track (right)
        ha, hc = along / 2.0, across / 2.0
        ring = [(x + ux * ha + vx * hc, y + uy * ha + vy * hc), (x + ux * ha - vx * hc, y + uy * ha - vy * hc),
                (x - ux * ha - vx * hc, y - uy * ha - vy * hc), (x - ux * ha + vx * hc, y - uy * ha + vy * hc)]
        ring.append(ring[0])
        out.append({"flight": fi + 1, "n": n + 1, "x": x, "y": y, "agl_m": agl,
                    "gsd_cm": P.gsd_from_height(cam, agl), "across_m": across, "along_m": along,
                    "heading_deg": math.degrees(hd) % 360.0, "ring": ring})
    return out


def oblique(mission):
    return abs(getattr(mission.params, "gimbal_pitch_deg", -90.0) + 90.0) > 10.0
