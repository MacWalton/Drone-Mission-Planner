import sys, os, math, json, zipfile, tempfile, threading, time, re
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from dronemapper_core import *
from dronemapper_core import exporters, uploader, units as U, terrain as TR
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84 = lambda x, y: t.transform(x, y)
x0, y0 = 381000, 6426000
d = get_drone("DJI Mavic 3 Enterprise")
blk = [[(x0, y0), (x0 + 1500, y0), (x0 + 1500, y0 + 1500), (x0, y0 + 1500)]]
print("== units:", U.to_m(400, "ft"), U.from_m(121.92, "ft"), U.fmt(100, "ft", 0, True))
mp = MissionParams(height_m=U.to_m(250, "ft"), rth_height_m=U.to_m(350, "ft"), battery_height_m=U.to_m(300, "ft"), height_unit="ft")
m = plan_mission(mp, d, Conditions(), blk, (x0 - 50, y0 - 50))
print("\n".join(l for l in m.report().splitlines() if "ht" in l or "height" in l.lower()))
print("flights:", len(m.flights))
for f in m.flights:
    w = f.waypoints
    out = [round(U.from_m(q.alt, "ft")) for q in w[1:4] if q.kind == "transit"]
    home = [round(U.from_m(q.alt, "ft")) for q in w[-3:] if q.kind in ("transit", "landing")]
    print("  F%d out-transit ft %s | home ft %s | kinds tail %s" % (f.index + 1, out, home, [q.kind for q in w[-3:]]))
last = m.flights[-1].waypoints[-1]; mid = m.flights[0].waypoints[-1]
assert abs(last.alt - U.to_m(350, "ft")) < 0.01 and abs(mid.alt - U.to_m(300, "ft")) < 0.01
print("== battery: all <=100%:", all(f.battery_fraction <= 1.0001 for f in m.flights), [round(f.battery_fraction*100) for f in m.flights])
out = tempfile.mkdtemp()
for fmt in ("qgc_plan", "dji_kmz", "litchi_csv", "brief"):
    exporters.export(m, fmt, os.path.join(out, "h" + exporters.FORMATS[fmt][1]), to84)
plan = json.load(open(os.path.join(out, "h_flight%02d.plan" % len(m.flights))))
items = plan["mission"]["items"]
print("== QGC last items:", [(it["command"], round(it["params"][6], 1)) for it in items[-2:]], "(16=WP at RTH m, 21=LAND)")
z = zipfile.ZipFile(os.path.join(out, "h_flight%02d.kmz" % len(m.flights))); wl = z.read("wpmz/waylines.wpml").decode()
hs = re.findall(r"<wpml:executeHeight>([\d.]+)</wpml:executeHeight>", wl)
print("== DJI last executeHeights (m):", hs[-2:], "| finishAction goHome:", "<wpml:finishAction>goHome</wpml:finishAction>" in wl)
html = open(os.path.join(out, "h.html")).read(); print("== brief shows feet:", "Return-to-home height" in html and "350 ft (107 m)" in html)

print("\n== terrain-follow + DEM: return floor honoured")
ground = lambda x, y: 20 + 80 * math.exp(-((x - (x0 + 700)) / 150.0) ** 2)
m2 = plan_mission(MissionParams(height_m=60, rth_height_m=110, battery_height_m=90, terrain_follow=True), d, Conditions(), blk, (x0 - 50, y0 - 50), ground_elevation=ground)
rep = TR.analyse(m2, ground)
for f in m2.flights[:3]:
    print("  F%d landing alt %.0f m | min home-transit alt %.0f m" % (f.index + 1, f.waypoints[-1].alt, min([q.alt for q in f.waypoints if q.kind == "transit"][-5:] or [0])))
print("  terrain:", rep.text("ft").splitlines()[1], "|", rep.text().splitlines()[1])

print("\n== MAVLink live upload sets failsafe RTL altitude")
from pymavlink import mavutil
got = {}
def vehicle(port, autopilot):
    v = mavutil.mavlink_connection("udpout:127.0.0.1:%d" % port, source_system=1, source_component=1)
    stop = time.time() + 15; count = None; items = {}
    while time.time() < stop:
        v.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, autopilot, 0, 0, 0)
        msg = v.recv_match(blocking=True, timeout=0.3)
        if msg is None: continue
        ty = msg.get_type()
        if ty == "MISSION_COUNT": count = msg.count; v.mav.mission_request_int_send(255, 0, 0)
        elif ty == "MISSION_ITEM_INT":
            items[msg.seq] = msg
            if msg.seq + 1 < count: v.mav.mission_request_int_send(255, 0, msg.seq + 1)
            else: v.mav.mission_ack_send(255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED)
        elif ty == "PARAM_SET":
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            got["param"] = (pid, msg.param_value)
            v.mav.param_value_send(msg.param_id if isinstance(msg.param_id, bytes) else pid.encode(), msg.param_value, msg.param_type, 1, 0); return
for port, ap, name in ((14571, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, "ArduPilot"), (14572, mavutil.mavlink.MAV_AUTOPILOT_PX4, "PX4")):
    got.clear(); th = threading.Thread(target=vehicle, args=(port, ap), daemon=True); th.start(); time.sleep(0.4)
    logs = []
    uploader.upload_mavlink(m, to84, "udpin:127.0.0.1:%d" % port, flight_index=0, log=logs.append); th.join(5)
    print("  %s: vehicle got %s | log: %s" % (name, got.get("param"), [l for l in logs if "return altitude" in l]))
