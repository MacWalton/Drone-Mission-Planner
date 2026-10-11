import sys, threading, time
import os as _os; REPO = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from pymavlink import mavutil
from dronemapper_core import *
from dronemapper_core import uploader
from pyproj import Transformer
t = Transformer.from_crs(32750, 4326, always_xy=True); to84=lambda x,y:t.transform(x,y)
x0,y0=381000,6426000
poly=[[(x0,y0),(x0+400,y0),(x0+400,y0+300),(x0,y0+300)]]
fw = get_drone("ArduPilot / PX4 Fixed-wing (generic)")
print("fixed-wing:", plan_mission(MissionParams(height_m=110), fw, Conditions(), poly, (x0,y0)).speed_ms)
m = plan_mission(MissionParams(height_m=80), get_drone("ArduPilot / PX4 Quad (generic)"), Conditions(), poly, (x0,y0))
got = {}
def vehicle():
    v = mavutil.mavlink_connection("udpout:127.0.0.1:14561", source_system=1, source_component=1)
    stop=time.time()+15; count=None; items={}
    while time.time()<stop:
        v.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,0,0,0)
        msg=v.recv_match(blocking=True,timeout=0.3)
        if msg is None: continue
        ty=msg.get_type()
        if ty=="MISSION_COUNT":
            count=msg.count; v.mav.mission_request_int_send(255,0,0)
        elif ty=="MISSION_ITEM_INT":
            items[msg.seq]=msg
            if msg.seq+1<count: v.mav.mission_request_int_send(255,0,msg.seq+1)
            else:
                v.mav.mission_ack_send(255,0,mavutil.mavlink.MAV_MISSION_ACCEPTED); got["items"]=items; return
th=threading.Thread(target=vehicle,daemon=True); th.start(); time.sleep(0.5)
n = uploader.upload_mavlink(m, to84, "udpin:127.0.0.1:14561", log=lambda s: None)
th.join(5)
print("uploaded", n, "vehicle received", len(got["items"]), "cmds", [got["items"][i].command for i in sorted(got["items"])][:8])
# file-based upload: export .plan and .waypoints then upload each
import tempfile, os, json
from dronemapper_core import exporters
d=tempfile.mkdtemp()
for fmt,ext in (("qgc_plan",".plan"),("mp_waypoints",".waypoints")):
    f=exporters.export(m, fmt, os.path.join(d,"x"+ext), to84)[0]
    h,its=uploader.read_mission_file(f)
    got.clear(); th=threading.Thread(target=vehicle,daemon=True); th.start(); time.sleep(0.5)
    n=uploader.upload_file(f,"udpin:127.0.0.1:14561",log=lambda s:None); th.join(5)
    print(ext, "items", n, "received", len(got["items"]), "lat ok", abs(got["items"][4].x/1e7 - its[3][6])<1e-6)
