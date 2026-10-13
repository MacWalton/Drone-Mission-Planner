# SPDX-License-Identifier: GPL-2.0-only
"""Mission file writers for the main drone ecosystems.

Format                     | Flies on
---------------------------|---------------------------------------------------
DJI WPML .kmz              | DJI Pilot 2 (M350/M300, M30, Mavic 3E/3T/3M, M4E/T),
                           | DJI FlightHub 2, DJI Fly waypoint missions*
QGroundControl .plan       | PX4 and ArduPilot (QGC), Auterion, Skydio X10 via QGC
Mission Planner .waypoints | ArduPilot (Mission Planner, MAVProxy, APM Planner)
Litchi .csv                | DJI consumer drones via Litchi (Mini/Air/Mavic/Phantom)
KML / GeoJSON / CSV        | Autel Explorer/Smart Controller, UgCS, Google Earth,
                           | any GIS, Pix4Dcapture & DroneDeploy boundary import
* DJI Fly support depends on firmware.

Every writer takes ``to_wgs84(x, y) -> (lon, lat)`` because the planner works
in projected metres.
"""
import csv
import io
import json
import os
import time
import uuid
import zipfile
from xml.sax.saxutils import escape
from . import units

FORMATS = {
    "dji_kmz": ("DJI WPML (.kmz)", ".kmz"),
    "qgc_plan": ("QGroundControl plan (.plan)", ".plan"),
    "mp_waypoints": ("Mission Planner (.waypoints)", ".waypoints"),
    "litchi_csv": ("Litchi mission (.csv)", ".csv"),
    "kml": ("KML route + boundary (.kml)", ".kml"),
    "geojson": ("GeoJSON (.geojson)", ".geojson"),
    "csv": ("Waypoint CSV (.csv)", ".csv"),
    "brief": ("Field brief & battery plan (.html)", ".html"),
    "brief_pdf": ("Mission report / field brief (.pdf)", ".pdf"),
}

DEFAULT_FORMAT_FOR_PLATFORM = {
    "dji_wpml": "dji_kmz",
    "dji_fly": "dji_kmz",
    "litchi": "litchi_csv",
    "mavlink": "qgc_plan",
    "autel": "kml",
    "generic": "kml",
}


def _flight_wps(mission, flight_index=None):
    if flight_index is None:
        return mission.waypoints
    return mission.flights[flight_index].waypoints


def _path_for_flight(path, k, n):
    if n <= 1:
        return path
    root, ext = os.path.splitext(path)
    return "%s_flight%02d%s" % (root, k + 1, ext)


def home_of(mission, flight_index):
    """(lon_lat_fn-ready x, y, alt_amsl) of the flight's launch station."""
    k = flight_index or 0
    f = mission.flights[k]
    x, y = f.station_start
    alt = mission.conditions.site_elevation_m
    if mission.takeoff_elev:                       # DEM present: shift by station height difference
        alt += f.z_ref - mission.takeoff_elev
    return x, y, alt


def export(mission, fmt, path, to_wgs84, per_flight=True, **kw):
    """Write the mission. Returns the list of files written."""
    writer = {
        "dji_kmz": write_dji_kmz, "qgc_plan": write_qgc_plan,
        "mp_waypoints": write_mp_waypoints, "litchi_csv": write_litchi_csv,
        "kml": write_kml, "geojson": write_geojson, "csv": write_csv, "brief": write_brief,
        "brief_pdf": write_brief_pdf,
    }[fmt]
    if fmt in ("kml", "geojson", "brief", "brief_pdf"):
        writer(mission, path, to_wgs84, **kw)
        return [path]
    out = []
    n = len(mission.flights) if per_flight else 1
    for k in range(n):
        p = _path_for_flight(path, k, n)
        res = writer(mission, p, to_wgs84, flight_index=k if per_flight else None, **kw)
        out.extend(res if isinstance(res, list) else [p])
    return out


# ---------------------------------------------------------------- MAVLink
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_DO_MOUNT_CONTROL = 205
FRAME_GLOBAL_REL_ALT = 3
FRAME_MISSION = 2


def mavlink_items(mission, to_wgs84, flight_index=None):
    """Mission as a list of MAVLink items (command, frame, p1..p4, lat, lon, alt).
    Item 0 is NOT home; the caller adds home if its protocol needs it."""
    wps = _flight_wps(mission, flight_index)
    items = []
    cur_trig = None
    tko = wps[0]
    lon, lat = to_wgs84(tko.x, tko.y)
    first_alt = next((w.alt for w in wps if w.kind.startswith("line")), mission.height_m)
    items.append((MAV_CMD_NAV_TAKEOFF, FRAME_GLOBAL_REL_ALT,
                  (15.0 if mission.drone.fixed_wing else 0.0), 0, 0, float("nan"), lat, lon, first_alt))
    items.append((MAV_CMD_DO_CHANGE_SPEED, FRAME_MISSION, 1, mission.speed_ms, -1, 0, 0, 0, 0))
    items.append((MAV_CMD_DO_MOUNT_CONTROL, FRAME_MISSION, mission.params.gimbal_pitch_deg,
                  0, 0, 0, 0, 0, 2))
    for w in wps[1:-1]:
        lon, lat = to_wgs84(w.x, w.y)
        items.append((MAV_CMD_NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT, 0, 0, 0, float("nan"), lat, lon, w.alt))
        trig = w.photo_dist if w.kind != "line_end" else 0.0
        if w.kind == "line_end":
            trig = 0.0
        if trig != cur_trig and (trig > 0 or cur_trig):
            items.append((MAV_CMD_DO_SET_CAM_TRIGG_DIST, FRAME_MISSION, trig, 0, 1 if trig else 0,
                          0, 0, 0, 0))
            cur_trig = trig
    if cur_trig:
        items.append((MAV_CMD_DO_SET_CAM_TRIGG_DIST, FRAME_MISSION, 0, 0, 0, 0, 0, 0, 0))
    # fly home at the planned return height, then land (an RTL command would use the
    # autopilot's own RTL altitude instead of the height set in the plan)
    land = wps[-1]
    lon, lat = to_wgs84(land.x, land.y)
    items.append((MAV_CMD_NAV_WAYPOINT, FRAME_GLOBAL_REL_ALT, 0, 0, 0, float("nan"), lat, lon, land.alt))
    items.append((MAV_CMD_NAV_LAND, FRAME_GLOBAL_REL_ALT, 0, 0, 0, float("nan"), lat, lon, 0))
    return items


def write_qgc_plan(mission, path, to_wgs84, flight_index=None, firmware="px4", **_):
    items = mavlink_items(mission, to_wgs84, flight_index)
    hx, hy, halt = home_of(mission, flight_index)
    lon0, lat0 = to_wgs84(hx, hy)
    out = []
    for i, it in enumerate(items, start=1):
        cmd, frame, p1, p2, p3, p4, lat, lon, alt = it
        params = [p1, p2, p3, None if p4 != p4 else p4, lat, lon, alt]
        out.append({"autoContinue": True, "command": cmd, "doJumpId": i, "frame": frame,
                    "params": params, "type": "SimpleItem",
                    "AMSLAltAboveTerrain": None,
                    "Altitude": alt if frame == FRAME_GLOBAL_REL_ALT else None,
                    "AltitudeMode": 1})
    plan = {
        "fileType": "Plan", "groundStation": "QGroundControl", "version": 1,
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "cruiseSpeed": mission.speed_ms, "hoverSpeed": mission.speed_ms,
            "firmwareType": 3 if firmware == "ardupilot" else 12,
            "globalPlanAltitudeMode": 1,
            "vehicleType": 1 if mission.drone.fixed_wing else 2,
            "plannedHomePosition": [lat0, lon0, halt],
            "items": out, "version": 2,
        },
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(plan, fh, indent=2, allow_nan=False)


def write_mp_waypoints(mission, path, to_wgs84, flight_index=None, **_):
    items = mavlink_items(mission, to_wgs84, flight_index)
    hx, hy, halt = home_of(mission, flight_index)
    lon0, lat0 = to_wgs84(hx, hy)
    rows = ["QGC WPL 110",
            "0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t%.2f\t1"
            % (lat0, lon0, halt)]
    for i, (cmd, frame, p1, p2, p3, p4, lat, lon, alt) in enumerate(items, start=1):
        p4 = 0 if p4 != p4 else p4
        rows.append("%d\t0\t%d\t%d\t%g\t%g\t%g\t%g\t%.8f\t%.8f\t%.2f\t1"
                    % (i, frame, cmd, p1, p2, p3, p4, lat, lon, alt))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(rows) + "\n")


# ----------------------------------------------------------------- Litchi
LITCHI_HEADER = (["latitude", "longitude", "altitude(m)", "heading(deg)", "curvesize(m)",
                  "rotationdir", "gimbalmode", "gimbalpitchangle"]
                 + [f for i in range(1, 16) for f in ("actiontype%d" % i, "actionparam%d" % i)]
                 + ["altitudemode", "speed(m/s)", "poi_latitude", "poi_longitude",
                    "poi_altitude(m)", "poi_altitudemode", "photo_timeinterval",
                    "photo_distinterval"])


def stop_points(mission, wps):
    """For stop-and-shoot: replace every survey leg by one waypoint per photo
    position (shoot=True), keeping transits and line ends."""
    from .planner import Waypoint
    trig = mission.trigger_m
    out = []
    for k, w in enumerate(wps):
        nxt = wps[k + 1] if k + 1 < len(wps) else None
        if w.photo_dist > 0 and nxt is not None:
            L = ((nxt.x - w.x) ** 2 + (nxt.y - w.y) ** 2) ** 0.5
            n = max(1, int(L // trig))
            for j in range(n + 1):
                t = min(1.0, j * trig / L) if L else 0.0
                if j and t >= 1.0 - 1e-9:
                    break
                out.append(Waypoint(w.x + (nxt.x - w.x) * t, w.y + (nxt.y - w.y) * t,
                                    w.alt + (nxt.alt - w.alt) * t, "photo", w.speed, 0.0, w.line_id,
                                    w.flight, True))
        else:
            if out and out[-1].shoot and w.kind == "line_end" and not w.shoot:
                w = Waypoint(w.x, w.y, w.alt, w.kind, w.speed, 0.0, w.line_id, w.flight, True)
            out.append(w)
    return out


def write_litchi_csv(mission, path, to_wgs84, flight_index=None, max_wp=99, **_):
    """Litchi CSV (import at flylitchi.com/hub). Splits at 99 waypoints.
    Photo trigger: distance -> photo_distinterval; timed -> photo_timeinterval;
    stop -> a waypoint per photo with Stay + Take Photo actions."""
    wps = _flight_wps(mission, flight_index)[1:]       # Litchi takes off itself
    mode = getattr(mission.params, "photo_trigger", "distance")
    if mode == "stop":
        wps = stop_points(mission, wps)
    interval = mission.drone.camera.min_trigger_interval_s
    heading = mission.heading_deg
    chunks = [wps[i:i + max_wp] for i in range(0, len(wps), max_wp)] or [[]]
    files = []
    for c, chunk in enumerate(chunks):
        root, ext = os.path.splitext(path)
        p = "%s_part%d%s" % (root, c + 1, ext) if len(chunks) > 1 else path
        with open(p, "w", newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh)
            wr.writerow(LITCHI_HEADER)
            for w in chunk:
                lon, lat = to_wgs84(w.x, w.y)
                row = [round(lat, 8), round(lon, 8), round(max(w.alt, 2.0), 1),
                       round(heading, 1), 0, 0, 2, mission.params.gimbal_pitch_deg]
                if mode == "stop" and w.shoot:       # 0 = stay (ms), 1 = take photo
                    acts = [0, int(mission.params.stop_hover_s * 1000), 1, 0] + [-1, 0] * 13
                else:
                    acts = [-1, 0] * 15
                row += acts
                t_iv = round(interval, 1) if (mode == "timed" and w.photo_dist > 0) else -1
                d_iv = round(w.photo_dist, 1) if (mode == "distance" and w.photo_dist > 0) else -1
                row += [1, round(w.speed, 1), 0, 0, 0, 0, t_iv, d_iv]
                wr.writerow(row)
        files.append(p)
    return files


# --------------------------------------------------------------- DJI WPML
WPML_NS = "http://www.dji.com/wpmz/1.0.2"


def read_dji_template(path):
    """droneEnumValue, droneSubEnumValue, payloadEnumValue from a mission file the
    aircraft's own app created (e.g. DJI Fly / Pilot 2 KMZ)."""
    import re
    if not zipfile.is_zipfile(path):
        raise ValueError("%s is not a DJI waypoint file (.kmz)." % os.path.basename(path))
    with zipfile.ZipFile(path) as z:
        names = [n for n in z.namelist() if n.endswith((".kml", ".wpml"))]
        text = "".join(z.read(n).decode("utf-8", "ignore") for n in names)
    get = lambda tag: (re.search(r"<wpml:%s>\s*(-?\d+)\s*</wpml:%s>" % (tag, tag), text) or [None, None])[1]
    out = {"drone": get("droneEnumValue"), "sub": get("droneSubEnumValue") or "0",
           "payload": get("payloadEnumValue")}
    if out["drone"] is None:
        raise ValueError("No droneEnumValue in %s - is it a DJI waypoint (KMZ) file?" % os.path.basename(path))
    return out


def _wpml_header(mission, finish_action, enums=None):
    d = mission.drone
    enum = [e.strip() for e in (d.drone_enum or "0,0").split(",")]   # "drone,payload[,sub]"
    sub = enum[2] if len(enum) > 2 and enum[2] else "0"
    enum = enum[:2]
    if enums:
        enum = [enums["drone"], enums["payload"] if enums.get("payload") is not None else ""]
        sub = enums.get("sub") or "0"
    now = int(time.time() * 1000)
    return """<wpml:author>Drone Mission Planner</wpml:author>
    <wpml:createTime>{now}</wpml:createTime>
    <wpml:updateTime>{now}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>{fa}</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{tsh:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{gts:.1f}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{de}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>{sub}</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>{pe}</wpml:payloadEnumValue>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>""".format(now=now, fa=finish_action, tsh=min(max(mission.height_m, 20), 1500),
                                    gts=min(mission.drone.max_speed_ms, 15.0),
                                    de=enum[0].strip(), pe=(enum[1].strip() or 0) if len(enum) > 1 else 0,
                                    sub=sub)


def write_dji_kmz(mission, path, to_wgs84, flight_index=None, template=None, **_):
    """DJI WPML KMZ (Pilot 2 / FlightHub 2 / DJI Fly waypoints).
    template: a KMZ made by the aircraft's own app - its drone/payload codes are
    copied so the app accepts the file (needed for DJI Fly).
    Photo trigger: distance -> multipleDistance photos; timed -> no photo actions
    (use the camera's Timed shot); stop -> a waypoint per photo with hover + takePhoto."""
    wps = _flight_wps(mission, flight_index)
    tko, land = wps[0], wps[-1]
    same_home = abs(land.x - tko.x) < 1.0 and abs(land.y - tko.y) < 1.0
    body = wps[1:]          # incl. the point over home at the planned return height
    finish = "goHome" if same_home else "noAction"
    mode = getattr(mission.params, "photo_trigger", "distance")
    if mode == "stop":
        body = stop_points(mission, body)
    enums = read_dji_template(template) if template else None

    placemarks = []
    action_id = 0
    for idx, w in enumerate(body):
        lon, lat = to_wgs84(w.x, w.y)
        actions = ""
        if idx == 0:
            actions += """
        <wpml:actionGroup>
          <wpml:actionGroupId>{g}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:gimbalHeadingYawBase>aircraft</wpml:gimbalHeadingYawBase>
              <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
              <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
              <wpml:gimbalPitchRotateAngle>{pitch:.1f}</wpml:gimbalPitchRotateAngle>
              <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
              <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
              <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
              <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
              <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
              <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>""".format(g=action_id, i=idx, pitch=mission.params.gimbal_pitch_deg)
            action_id += 1
        if mode == "stop" and w.shoot:
            actions += """
        <wpml:actionGroup>
          <wpml:actionGroupId>{g}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{i}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger><wpml:actionTriggerType>reachPoint</wpml:actionTriggerType></wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>hover</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:hoverTime>{h:g}</wpml:hoverTime>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
          <wpml:action>
            <wpml:actionId>1</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>""".format(g=action_id, i=idx, h=max(1.0, mission.params.stop_hover_s))
            action_id += 1
        elif mode == "distance" and w.photo_dist > 0 and idx + 1 < len(body):
            actions += """
        <wpml:actionGroup>
          <wpml:actionGroupId>{g}</wpml:actionGroupId>
          <wpml:actionGroupStartIndex>{i}</wpml:actionGroupStartIndex>
          <wpml:actionGroupEndIndex>{j}</wpml:actionGroupEndIndex>
          <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
          <wpml:actionTrigger>
            <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
            <wpml:actionTriggerParam>{d:.2f}</wpml:actionTriggerParam>
          </wpml:actionTrigger>
          <wpml:action>
            <wpml:actionId>0</wpml:actionId>
            <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
            <wpml:actionActuatorFuncParam>
              <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
              <wpml:useGlobalPayloadLensIndex>1</wpml:useGlobalPayloadLensIndex>
            </wpml:actionActuatorFuncParam>
          </wpml:action>
        </wpml:actionGroup>""".format(g=action_id, i=idx, j=idx + 1, d=w.photo_dist)
            action_id += 1
        placemarks.append("""
      <Placemark>
        <Point><coordinates>{lon:.8f},{lat:.8f}</coordinates></Point>
        <wpml:index>{i}</wpml:index>
        <wpml:executeHeight>{alt:.2f}</wpml:executeHeight>
        <wpml:waypointSpeed>{spd:.2f}</wpml:waypointSpeed>
        <wpml:waypointHeadingParam>
          <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
          <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
          <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
          <wpml:waypointHeadingPathMode>followBadArc</wpml:waypointHeadingPathMode>
        </wpml:waypointHeadingParam>
        <wpml:waypointTurnParam>
          <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
          <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
        </wpml:waypointTurnParam>
        <wpml:useStraightLine>1</wpml:useStraightLine>{act}
      </Placemark>""".format(lon=lon, lat=lat, i=idx, alt=w.alt, spd=w.speed or mission.speed_ms,
                             act=actions))

    header = _wpml_header(mission, finish, enums)
    template = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="{ns}">
  <Document>
    {header}
    <Folder>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>{spd:.2f}</wpml:autoFlightSpeed>
      <wpml:globalHeight>{h:.2f}</wpml:globalHeight>
      <wpml:caliFlightEnable>0</wpml:caliFlightEnable>
      <wpml:gimbalPitchMode>usePointSetting</wpml:gimbalPitchMode>
      <wpml:globalWaypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>0.000000,0.000000,0.000000</wpml:waypointPoiPoint>
        <wpml:waypointHeadingPoiIndex>0</wpml:waypointHeadingPoiIndex>
      </wpml:globalWaypointHeadingParam>
      <wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode>
      <wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>{pms}
    </Folder>
  </Document>
</kml>
""".format(ns=WPML_NS, header=header, spd=mission.speed_ms, h=mission.height_m,
           pms="".join(placemarks))
    waylines = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="{ns}">
  <Document>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>{fa}</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:takeOffSecurityHeight>{tsh:.1f}</wpml:takeOffSecurityHeight>
      <wpml:globalTransitionalSpeed>{gts:.1f}</wpml:globalTransitionalSpeed>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:executeHeightMode>relativeToStartPoint</wpml:executeHeightMode>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:distance>{dist:.1f}</wpml:distance>
      <wpml:duration>{dur:.1f}</wpml:duration>
      <wpml:autoFlightSpeed>{spd:.2f}</wpml:autoFlightSpeed>{pms}
    </Folder>
  </Document>
</kml>
""".format(ns=WPML_NS, fa=finish, tsh=min(max(mission.height_m, 20), 1500),
           gts=min(mission.drone.max_speed_ms, 15.0),
           dist=sum(f.distance_m for f in mission.flights) if flight_index is None
           else mission.flights[flight_index].distance_m,
           dur=mission.total_time_s if flight_index is None
           else mission.flights[flight_index].duration_s,
           spd=mission.speed_ms, pms="".join(placemarks))
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("wpmz/template.kml", template)
        z.writestr("wpmz/waylines.wpml", waylines)


# ------------------------------------------------------------ KML / JSON
def _ring_lonlat(ring, to_wgs84):
    pts = [to_wgs84(x, y) for x, y in ring]
    if pts and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def write_kml(mission, path, to_wgs84, **_):
    def coords(pts, alt=None):
        return " ".join("%.8f,%.8f,%.2f" % (lo, la, alt if alt is not None else 0) for lo, la in pts)

    parts = []
    rings = mission.boundary
    outer = coords(_ring_lonlat(rings[0], to_wgs84))
    inner = "".join("<innerBoundaryIs><LinearRing><coordinates>%s</coordinates></LinearRing></innerBoundaryIs>"
                    % coords(_ring_lonlat(r, to_wgs84)) for r in rings[1:])
    parts.append("""<Placemark><name>Mission boundary</name><styleUrl>#bnd</styleUrl>
<Polygon><outerBoundaryIs><LinearRing><coordinates>%s</coordinates></LinearRing></outerBoundaryIs>%s</Polygon></Placemark>"""
                 % (outer, inner))
    for f in mission.flights:
        pts = " ".join("%.8f,%.8f,%.2f" % (to_wgs84(w.x, w.y) + (w.alt,)) for w in f.waypoints)
        parts.append("""<Placemark><name>Flight %d route</name><styleUrl>#route</styleUrl>
<LineString><altitudeMode>relativeToGround</altitudeMode><coordinates>%s</coordinates></LineString></Placemark>"""
                     % (f.index + 1, pts))
    pts = [(mission.label(st), st) for st in mission.stations]
    if tuple(mission.landing) not in [tuple(st) for st in mission.stations]:
        pts.append(("Landing", mission.landing))
    for name, p in pts:
        lo, la = to_wgs84(*p)
        parts.append("<Placemark><name>%s</name><Point><coordinates>%.8f,%.8f,0</coordinates></Point></Placemark>"
                     % (escape(name), lo, la))
    kml = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>%s</name>
<Style id="bnd"><LineStyle><color>ff00a5ff</color><width>2</width></LineStyle><PolyStyle><color>3300a5ff</color></PolyStyle></Style>
<Style id="route"><LineStyle><color>ffffff00</color><width>2</width></LineStyle></Style>
%s
</Document></kml>
""" % (escape(mission_title(mission, path)), "\n".join(parts))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(kml)


def write_geojson(mission, path, to_wgs84, **_):
    feats = [{"type": "Feature", "properties": {"type": "boundary", "area_ha": mission.area_m2 / 1e4},
              "geometry": {"type": "Polygon",
                           "coordinates": [_ring_lonlat(r, to_wgs84) for r in mission.boundary]}}]
    for f in mission.flights:
        feats.append({"type": "Feature",
                      "properties": {"type": "route", "flight": f.index + 1,
                                     "minutes": round(f.duration_s / 60, 1)},
                      "geometry": {"type": "LineString",
                                   "coordinates": [list(to_wgs84(w.x, w.y)) + [w.alt]
                                                   for w in f.waypoints]}})
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"type": "FeatureCollection", "features": feats}, fh)


def write_csv(mission, path, to_wgs84, flight_index=None, **_):
    with open(path, "w", newline="", encoding="utf-8") as fh:
        wr = csv.writer(fh)
        wr.writerow(["flight", "seq", "kind", "latitude", "longitude", "alt_rel_m",
                     "speed_ms", "photo_dist_m"])
        for i, w in enumerate(_flight_wps(mission, flight_index)):
            lon, lat = to_wgs84(w.x, w.y)
            wr.writerow([w.flight + 1, i, w.kind, "%.8f" % lat, "%.8f" % lon,
                         "%.2f" % w.alt, "%.1f" % w.speed, "%.2f" % w.photo_dist])


# -------------------------------------------------------------- field brief
FORMATS_BRIEF = ("brief", "brief_pdf")


def brief_content(mission, to_wgs84, path=None):
    """Everything the field brief / mission report shows, as plain text (shared by the
    HTML and PDF writers)."""
    from .logistics import clock
    from .units import fmt as _fmt, mass_fmt as _mfmt
    m = mission
    sc = m.schedule
    lp = sc.params if sc else None
    hu = getattr(m.params, "height_unit", "m")
    mu = getattr(m.params, "mass_unit", "kg")
    stations = []
    for st in m.stations + ([m.landing] if tuple(m.landing) not in [tuple(s) for s in m.stations] else []):
        lo, la = to_wgs84(*st)
        n = sum(1 for f in m.flights if tuple(f.station_start) == tuple(st))
        stations.append([m.label(st), "%.6f, %.6f" % (la, lo), "%d" % n])
    flights = []
    srow = {r.flight: r for r in (sc.rows if sc else [])}
    for f in m.flights:
        r = srow.get(f.index + 1)
        kp = ("%.2f\u2013%.2f" % (f.chainage[0] / 1000, f.chainage[1] / 1000)) if f.chainage else ""
        dl = {x.flight: x for x in (getattr(m, "daylight", None).flights if getattr(m, "daylight", None) else [])}
        flights.append(["%d" % (f.index + 1), "%s \u2192 %s" % (m.label(f.station_start), m.label(f.station_end)),
                        "%s" % (r.battery_set if r else ""),
                        clock(lp.start_time_min + r.start_min) if r else "",
                        clock(lp.start_time_min + r.end_min) if r else "", "%.1f" % (f.duration_s / 60),
                        "%.0f%%" % (f.battery_fraction * 100), "%d" % f.photos, kp,
                        dl[f.index + 1].sun_text if (f.index + 1) in dl else "-",
                        (("battery change mid-line. " if f.ends_midline else "") + (r.note if r else "")).strip()])
    h, mm = divmod(int(m.total_time_s // 60), 60)
    summary = [("Aircraft", m.drone.name), ("Mission", m.params.mission_type),
               ("Area", "%.2f ha" % (m.area_m2 / 1e4)), ("Height", _fmt(m.height_m, hu, 0, True)),
               ("Return-to-home height", _fmt(m.rth_height_m, hu, 0, True)),
               ("Battery-change height", _fmt(m.battery_height_m, hu, 0, True)),
               ("GSD", "%.2f cm/px" % m.gsd_cm), ("Speed", "%.1f m/s" % m.speed_ms),
               ("Photos", "%d" % m.photos), ("Image coverage", "%.2f ha" % (m.coverage["area_m2"] / 1e4)),
               ("Flights", "%d" % len(m.flights)),
               ("Flight time", "%dh %02dm" % (h, mm)),
               ("Usable / battery", "%.1f min" % m.endurance["usable_minutes_still_air"]),
               ("Wind", "%s from %.0f\u00b0" % (units.wind_fmt(m.conditions.wind_speed_ms,
                                                               getattr(m.conditions, "wind_unit", "kn")),
                                                m.conditions.wind_from_deg)),
               ("Temp / payload", "%.0f \u00b0C / %s" % (m.conditions.temperature_c,
                                                          _mfmt(m.conditions.total_payload_kg, mu))),
               ("Accessory power", "%.0f W" % m.conditions.accessory_power_w)]
    if m.centreline:
        summary.insert(3, ("Route", "%.2f km, %d sections" % (sum(
            dist_m(a, b) for a, b in zip(m.centreline[:-1], m.centreline[1:])) / 1000, len(m.sections))))
    dlr = getattr(m, "daylight", None)
    if dlr is not None:
        summary[0:0] = [("Flight date", dlr.date_text)]
        summary += [("Daylight", "%s \u2013 %s" % (_dclock(dlr.sunrise), _dclock(dlr.sunset)) if dlr.sunrise is not None
                     else dlr.daylight_text),
                    ("Civil twilight", "%s \u2013 %s" % (_dclock(dlr.dawn), _dclock(dlr.dusk))),
                    ("Best light (sun \u2265 %.0f\u00b0)" % dlr.min_sun_deg,
                     ("%s \u2013 %s" % (_dclock(dlr.best[0]), _dclock(dlr.best[1]))) if dlr.best
                     else "never (max %.0f\u00b0)" % dlr.max_elev)]
    if sc:
        summary += [("On site", "%s \u2013 %s (%.1f h)" % (clock(lp.start_time_min),
                                                         clock(lp.start_time_min + sc.total_min), sc.total_min / 60)),
                    ("Battery sets", "%d (recommended %d)" % (lp.battery_sets, sc.recommended_sets)),
                    ("Charge waiting", "%.0f min" % sc.waiting_min)]
    payload = None
    c = m.conditions
    if c.items or c.payload_kg:
        rows = [[i.name, _mfmt(i.mass_kg, mu), ("%.0f W" % i.power_w) if i.power_w else "-"] for i in c.items]
        if c.payload_kg:
            rows.append(["Other (unlisted)", _mfmt(c.payload_kg, mu), "-"])
        e = m.endurance
        payload = {"rows": rows, "total": ["Total", _mfmt(c.total_payload_kg, mu), "%.0f W" % c.accessory_power_w],
                   "note": "Take-off mass %.2f kg (limit %.2f kg payload). Usable endurance %.1f min per battery; "
                           "%.1f min without accessory power." % (e["take_off_mass_kg"], m.drone.max_payload_kg,
                                                                   e["usable_minutes_still_air"],
                                                                   e.get("minutes_without_accessories", 0))}
    terrain = None
    rep = getattr(m, "terrain", None)
    if rep is not None:
        rows = [["Low clearance" if i.kind == "low" else "Obstacle" if i.kind == "obstacle" else "Above height limit",
                 "%d" % i.flight, ("%.3f" % (i.kp / 1000)) if i.kp is not None else "", "%.0f m" % i.value_m,
                 "%.0f" % i.ground_m, "%.0f" % i.alt_amsl] for i in rep.issues[:40]]
        terrain = {"text": rep.text(hu), "rows": rows}
    sub = ("%s \u00b7 flight date %s \u00b7 prepared %s" % (m.drone.name, dlr.date.strftime("%a %d %b %Y"),
                                                           time.strftime("%d %b %Y")) if dlr is not None
           else "%s, %s" % (m.drone.name, time.strftime("%d %b %Y")))
    airspace = None
    ar = getattr(m, "airspace", None)
    if ar is not None:
        summary.insert(1 if dlr is not None else 0, ("Airspace", ar.summary))
        order = ("inside", "below", "near", "above")
        rows = []
        for hh in sorted(ar.hits, key=lambda q: order.index(q.status)):
            a = hh.airspace
            rows.append([{"inside": "INSIDE", "below": "Below floor", "near": "Nearby", "above": "Above"}[hh.status],
                         a.name, a.label, a.limits_text, ", ".join(str(x) for x in hh.flights[:12]) or "-",
                         hh.message.split(": ", 1)[-1] if hh.status == "inside" else
                         ("%.0f m from the mission" % hh.min_dist_m if hh.status == "near" else hh.message)])
        airspace = {"summary": ar.summary, "rows": rows, "sources": ", ".join(ar.sources) or "-",
                    "note": "Checked against %d airspace(s) from %s along every flight path. Not an authoritative "
                            "source - confirm with an approved airspace app / authority and check NOTAMs before "
                            "flight." % (ar.checked, ", ".join(ar.sources) or "-")}
    return {"title": mission_title(m, path), "sub": sub, "airspace": airspace,
            "summary": summary, "stations": stations, "flights": flights, "payload": payload,
            "terrain": terrain, "warnings": list(m.warnings),
            "check": "Check airspace, NOTAMs, weather and the mission preview in the flight app before each flight."}


STATION_HEAD = ["Station", "Lat, Lon (WGS84)", "Flights launched"]
FLIGHT_HEAD = ["#", "Launch \u2192 land", "Set", "Start", "End", "Min", "Battery", "Photos", "KP (km)", "Sun",
               "Notes", "Done"]


def _dclock(v):
    from .daylight import clock
    return clock(v)
AIRSPACE_HEAD = ["Status", "Airspace", "Class / type", "Limits", "Flights", "What it means"]
TERRAIN_HEAD = ["Issue", "Flight", "KP (km)", "Clearance / AGL", "Ground m AMSL", "Aircraft m AMSL"]


def write_brief(mission, path, to_wgs84, background=None, kp_interval=None, **_):
    """Printable field sheet (HTML): summary, map, stations, battery schedule, checks."""
    m = mission
    esc = escape
    d = brief_content(m, to_wgs84, path)
    tr = lambda cells, tag="td": "<tr>%s</tr>" % "".join("<%s>%s</%s>" % (tag, esc(c), tag) for c in cells)
    st_rows = "".join("<tr><td><b>%s</b></td><td>%s</td><td>%s</td></tr>" % tuple(esc(c) for c in r)
                      for r in d["stations"])
    fl_rows = "".join("<tr>%s<td>&#9744;</td></tr>" % "".join("<td>%s</td>" % esc(c) for c in r)
                      for r in d["flights"])
    warn = "".join("<li>%s</li>" % esc(w) for w in d["warnings"])
    cards = "".join("<div class=c><span>%s</span><b>%s</b></div>" % (esc(k), esc(v)) for k, v in d["summary"])
    payload = ""
    if d["payload"]:
        p = d["payload"]
        payload = ("<h2>Payload</h2><table>%s%s%s</table><div style='color:#6b7280;margin-top:4px'>%s</div>"
                   % (tr(["Item", "Weight", "Power"], "th"), "".join(tr(r) for r in p["rows"]),
                      tr(p["total"], "th"), esc(p["note"])))
    terrain = ""
    if d["terrain"]:
        from .terrain import svg_profile
        t = d["terrain"]
        table = ("<table>%s%s</table>" % (tr(TERRAIN_HEAD, "th"), "".join(tr(r) for r in t["rows"]))) if t["rows"] else ""
        svg = svg_profile(m.profile, unit=getattr(m.params, "height_unit", "m")) if getattr(m, "profile", None) else ""
        terrain = ("<h2>Terrain (DEM)</h2><div style='font-family:monospace;font-size:12px'>%s</div>%s%s"
                   % (esc(t["text"]).replace("\n", "<br>"), table,
                      ("<div style='margin-top:8px'>%s</div>" % svg) if svg else ""))
    html = """<!doctype html><html><head><meta charset="utf-8"><title>Field brief - %(name)s</title>
<style>
h2.map{break-before:auto}svg{break-inside:avoid}
body{font:13px/1.4 -apple-system,Segoe UI,Roboto,sans-serif;margin:24px;color:#1f2937}
h1{font-size:20px;margin:0 0 4px}h2{font-size:15px;margin:22px 0 6px;border-bottom:2px solid #0e7490;padding-bottom:2px}
.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:6px}
.c{border:1px solid #d1d5db;border-radius:6px;padding:6px 8px}.c span{display:block;color:#6b7280;font-size:11px}
table{border-collapse:collapse;width:100%%}th,td{border:1px solid #d1d5db;padding:4px 6px;text-align:left;vertical-align:top}
th{background:#ecfeff}ul{margin:4px 0;padding-left:18px}.w li{color:#b45309}
@media print{body{margin:10mm}h2{break-after:avoid}}
</style></head><body>
<h1>%(name)s</h1><div style="color:#6b7280">Drone mission field brief &middot; %(sub)s</div>
<h2>Summary</h2><div class=g>%(cards)s</div>
<h2>Mission map</h2>%(map)s
%(payload)s
%(airspace)s
<h2>Launch stations</h2><table>%(st_head)s%(st)s</table>
<h2>Flights &amp; battery plan</h2>
<table>%(fl_head)s%(fl)s</table>
%(terrain)s
<h2>Warnings &amp; checks</h2><ul class=w>%(warn)s<li style="color:#1f2937">%(check)s</li></ul>
</body></html>""" % dict(name=esc(d["title"]), cards=cards, sub=esc(d["sub"]),
                         map=_brief_map(m, background, kp_interval), st_head=tr(STATION_HEAD, "th"),
                         st=st_rows, fl_head=tr(FLIGHT_HEAD, "th"), fl=fl_rows, warn=warn,
                         terrain=terrain, payload=payload, check=esc(d["check"]),
                         airspace=(("<h2>Airspace</h2><p><b>%s</b></p><table>%s%s</table><div style='color:#6b7280;"
                                    "margin-top:4px'>%s</div>" % (esc(d["airspace"]["summary"]),
                                                                  tr(AIRSPACE_HEAD, "th"),
                                                                  "".join(tr(r) for r in d["airspace"]["rows"]),
                                                                  esc(d["airspace"]["note"])))
                                   if d["airspace"] else ""))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(html)


def write_brief_pdf(mission, path, to_wgs84, background=None, kp_interval=None, **_):
    """The field brief as a PDF (opens anywhere, prints on A4)."""
    from .pdfreport import write_report_pdf
    write_report_pdf(mission, path, to_wgs84, background=background, kp_interval=kp_interval)


def dist_m(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def mission_title(mission, path=None):
    name = (getattr(mission.params, "mission_name", "") or "").strip()
    if name:
        return name
    return os.path.splitext(os.path.basename(path))[0] if path else "Drone mission"


def _brief_map(m, background=None, kp_interval=None):
    """Overview map (+ zoomed section maps for multi-station routes).
    background: PNG bytes for the overview, or a callable (extent, (w, h)) -> PNG
    bytes used for the overview and every section map."""
    from .briefmap import svg_map, legend_html, section_maps, map_view
    try:
        fn = background if callable(background) else None
        view = map_view(m, 900)
        bg = fn(view[0], (900, view[1])) if fn else background
        html = svg_map(m, 900, bg, kp_interval, mission_title(m), view) + legend_html(m)
        if m.centreline and len(m.stations) > 1:
            html += section_maps(m, 440, fn)
        return html
    except Exception as e:                       # never lose the brief because of the map
        return "<p style='color:#b45309'>Map could not be drawn: %s</p>" % escape(str(e))
