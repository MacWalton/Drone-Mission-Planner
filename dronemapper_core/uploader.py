# SPDX-License-Identifier: GPL-2.0-only
"""Sending missions to aircraft.

* MAVLink (ArduPilot, PX4 and anything that speaks MAVLink mission
  protocol): uploaded LIVE over USB/serial telemetry radio, UDP or TCP
  using pymavlink. Install once with:  python -m pip install pymavlink
* DJI Pilot 2 / RC Plus / RC Pro / Smart Controller: DJI does not allow
  third-party desktop software to push missions over the radio link. The
  plugin writes the .kmz straight onto the controller's storage when it is
  connected by USB (or onto an SD card / USB stick) so it appears in
  Pilot 2 > Flight Route > Import. For cloud delivery use DJI FlightHub 2
  (upload the same .kmz under Flight Route Library).
* Litchi: import the CSV in Litchi Mission Hub (flylitchi.com/hub); it
  syncs to the Litchi app on the controller.
* Autel Explorer / Smart Controller: copy the .kml to the controller and
  import it as a mission boundary/route.
"""
import json
import math
import os
import shutil
import time

from .exporters import mavlink_items, FRAME_GLOBAL_REL_ALT


class UploadError(Exception):
    pass


def pymavlink_available():
    try:
        import pymavlink  # noqa: F401
        return True
    except Exception:
        return False


def upload_mavlink(mission, to_wgs84, connection, baud=57600, flight_index=0,
                   timeout=10.0, log=print):
    """Upload one flight of a planned mission over MAVLink. connection examples:
    'COM5', '/dev/ttyUSB0', 'udpin:0.0.0.0:14550', 'udpout:192.168.4.1:14550',
    'tcp:127.0.0.1:5760' (SITL)."""
    from .exporters import home_of
    hx, hy, halt = home_of(mission, flight_index)
    lon0, lat0 = to_wgs84(hx, hy)
    home = (16, 0, 0, 0, 0, 0, lat0, lon0, halt)
    return upload_items(home, mavlink_items(mission, to_wgs84, flight_index),
                        connection, baud, timeout, log, rtl_alt_m=mission.rth_height_m)


def read_mission_file(path):
    """Read a QGroundControl .plan or Mission Planner .waypoints file.
    Returns (home_item_or_None, items) in the tuple form used by upload_items."""
    if path.lower().endswith(".plan"):
        with open(path, encoding="utf-8") as fh:
            plan = json.load(fh)
        ms = plan["mission"]
        hp = ms.get("plannedHomePosition") or [0, 0, 0]
        home = (16, 0, 0, 0, 0, 0, hp[0], hp[1], hp[2])
        items = []
        for it in ms["items"]:
            if it.get("type") != "SimpleItem":
                raise UploadError("Complex items (Survey/Corridor patterns) are not supported - "
                                  "export from this plugin or convert in QGC first.")
            p = [(float("nan") if v is None else v) for v in it["params"]]
            items.append((it["command"], it["frame"], p[0], p[1], p[2], p[3], p[4], p[5], p[6]))
        return home, items
    with open(path, encoding="utf-8") as fh:
        rows = [r.split() for r in fh.read().splitlines() if r.strip()]
    if not rows or not rows[0][0].startswith("QGC"):
        raise UploadError("Not a QGC WPL .waypoints file.")
    home, items = None, []
    for r in rows[1:]:
        seq, cur, frame, cmd = int(r[0]), int(r[1]), int(r[2]), int(r[3])
        p = [float(v) for v in r[4:11]]
        it = (cmd, frame, p[0], p[1], p[2], p[3], p[4], p[5], p[6])
        if seq == 0:
            home = it
        else:
            items.append(it)
    return home, items


def upload_file(path, connection, baud=57600, timeout=10.0, log=print, rtl_alt_m=None):
    home, items = read_mission_file(path)
    return upload_items(home, items, connection, baud, timeout, log, rtl_alt_m)


def _set_rtl_alt(m, mavutil, is_px4, alt_m, log, timeout=4.0):
    """Make the autopilot's own failsafe Return-To-Launch altitude match the plan."""
    name, value = ("RTL_RETURN_ALT", float(alt_m)) if is_px4 else ("RTL_ALT", float(alt_m) * 100.0)
    m.mav.param_set_send(m.target_system, m.target_component, name.encode("ascii"), value,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pid = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
        if pid.strip("\x00") == name:
            ok = abs(msg.param_value - value) < 0.5
            log("Failsafe return altitude %s = %.0f%s (%s)" % (name, msg.param_value,
                                                                 " m" if is_px4 else " cm",
                                                                 "set" if ok else "NOT accepted"))
            return ok
    log("WARNING: no reply setting %s - set the RTL altitude on the aircraft yourself." % name)
    return False


def upload_items(home, items, connection, baud=57600, timeout=10.0, log=print, rtl_alt_m=None):
    try:
        from pymavlink import mavutil
    except ImportError:
        raise UploadError("pymavlink is not installed. In the OSGeo4W Shell / ArcGIS Pro "
                          "Python Command Prompt run:  python -m pip install pymavlink")
    log("Connecting to %s ..." % connection)
    m = mavutil.mavlink_connection(connection, baud=baud, source_system=255)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise UploadError("No heartbeat from the vehicle on %s." % connection)
    log("Heartbeat from system %d component %d" % (m.target_system, m.target_component))

    is_px4 = hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
    items = list(items) if is_px4 else [home or items[0]] + list(items)  # ArduPilot: seq 0 = home

    def make(seq):
        cmd, frame, p1, p2, p3, p4, lat, lon, alt = items[seq]
        p4 = 0.0 if p4 != p4 else p4
        if seq == 0 and not is_px4:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL
        args = [m.target_system, m.target_component, seq, frame, cmd,
                0, 1, float(p1), float(p2), float(p3), float(p4),
                int(round(lat * 1e7)), int(round(lon * 1e7)), float(alt)]
        try:   # MAVLink 2 has a mission_type field, MAVLink 1 does not
            return m.mav.mission_item_int_encode(*(args + [0]))
        except TypeError:
            return m.mav.mission_item_int_encode(*args)

    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    time.sleep(0.5)
    try:
        m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
    except TypeError:
        m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = set()
    deadline = time.time() + timeout
    while True:
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
                           blocking=True, timeout=1.0)
        if msg is None:
            if time.time() > deadline:
                raise UploadError("Timed out waiting for the vehicle (sent %d/%d items)."
                                  % (len(sent), len(items)))
            continue
        deadline = time.time() + timeout
        if msg.get_type() == "MISSION_ACK":
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED and len(sent) >= len(items):
                log("Mission accepted: %d items." % len(items))
                if rtl_alt_m:
                    _set_rtl_alt(m, mavutil, is_px4, rtl_alt_m, log)
                return len(items)
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise UploadError("Vehicle rejected the mission (MAV_MISSION_RESULT %d)." % msg.type)
            continue
        seq = msg.seq
        if 0 <= seq < len(items):
            m.mav.send(make(seq))
            sent.add(seq)
            log("  sent item %d/%d" % (seq + 1, len(items)))


# --------------------------------------------------------- file transfer
DJI_PILOT2_HINT = ("On the DJI RC Plus / RC Pro / Smart Controller open Pilot 2 > Flight Route > "
                   "Import Route (KMZ/KML) and select the file. For cloud upload use "
                   "FlightHub 2 > Flight Route Library > Import.")


def find_removable_drives():
    """Best-effort list of mounted removable volumes (USB/SD/controller)."""
    roots = []
    if os.name == "nt":
        try:
            import ctypes
            bitmask = ctypes.windll.kernel32.GetLogicalDrives()
            for i in range(26):
                if bitmask & (1 << i):
                    d = "%s:\\" % chr(65 + i)
                    if ctypes.windll.kernel32.GetDriveTypeW(d) == 2:   # DRIVE_REMOVABLE
                        roots.append(d)
        except Exception:
            pass
    else:
        for base in ("/media", "/run/media", "/Volumes"):
            if os.path.isdir(base):
                for root, dirs, _ in os.walk(base):
                    for d in dirs:
                        roots.append(os.path.join(root, d))
                    break
    return roots


def copy_to_device(files, dest_dir):
    """Copy mission files to a controller / SD card / USB stick folder."""
    if not os.path.isdir(dest_dir):
        raise UploadError("Destination folder not found: %s" % dest_dir)
    out = []
    for f in files:
        target = os.path.join(dest_dir, os.path.basename(f))
        shutil.copy2(f, target)
        out.append(target)
    return out


def replace_dji_mission(src_kmz, target_kmz, log=print):
    """Overwrite a mission file made by DJI Fly / Pilot 2 with our KMZ, after
    copying the aircraft/payload codes from that file into ours (DJI Fly only
    loads routes made for its aircraft). The original is kept as .bak."""
    import re
    import zipfile
    from .exporters import read_dji_template
    codes = read_dji_template(target_kmz)
    tmp = target_kmz + ".dmp_tmp"
    with zipfile.ZipFile(src_kmz) as zi, zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zo:
        for n in zi.namelist():
            data = zi.read(n)
            if n.endswith((".kml", ".wpml")):
                t = data.decode("utf-8")
                for tag, val in (("droneEnumValue", codes["drone"]), ("droneSubEnumValue", codes["sub"]),
                                 ("payloadEnumValue", codes["payload"])):
                    if val is not None:
                        t = re.sub(r"<wpml:%s>[^<]*</wpml:%s>" % (tag, tag), "<wpml:%s>%s</wpml:%s>" % (tag, val, tag), t)
                data = t.encode("utf-8")
            zo.writestr(n, data)
    bak = target_kmz + ".bak"
    if not os.path.exists(bak):
        shutil.copy2(target_kmz, bak)
    os.replace(tmp, target_kmz)
    log("Replaced %s (aircraft code %s; original kept as %s)." % (target_kmz, codes["drone"], os.path.basename(bak)))
    return target_kmz
