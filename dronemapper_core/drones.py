# SPDX-License-Identifier: GPL-2.0-only
"""Drone and camera profiles.

Values are typical manufacturer figures and are only defaults - every field
can be edited in the plugin UI before planning. Always confirm limits against
your own aircraft manual.
"""
from dataclasses import dataclass, field, asdict, replace


@dataclass
class Camera:
    name: str
    sensor_width_mm: float
    sensor_height_mm: float
    focal_length_mm: float
    image_width_px: int
    image_height_px: int
    min_trigger_interval_s: float = 2.0   # fastest sustained photo interval

    @property
    def pixel_pitch_mm(self):
        return self.sensor_width_mm / self.image_width_px


@dataclass
class Drone:
    name: str
    platform: str              # dji_wpml | dji_fly | mavlink | litchi | autel | generic
    camera: Camera
    empty_mass_kg: float       # take-off mass incl. battery, no payload
    max_payload_kg: float
    rated_flight_time_min: float   # hover/cruise endurance, sea level, 15 C, no wind
    max_speed_ms: float
    max_wind_ms: float
    battery_wh: float
    rotor_disk_area_m2: float = 0.12   # total disk area (used for density scaling)
    fixed_wing: bool = False
    max_waypoints: int = 65535
    drone_enum: str = ""       # DJI WPML "droneEnumValue,payloadEnumValue[,droneSubEnumValue]"
    photo_trigger: str = "distance"   # default photo trigger: distance | timed | stop
    notes: str = ""

    def to_dict(self):
        return asdict(self)

    @property
    def needs_dji_template(self):
        """True for DJI aircraft whose WPML aircraft/payload code isn't known (export with a template)."""
        if self.platform == "dji_fly":
            return True
        if self.platform != "dji_wpml":
            return False
        parts = [p.strip() for p in (self.drone_enum or "").split(",")]
        return len(parts) < 2 or not parts[0] or not parts[1]


CAMERAS = {
    "DJI Mavic 3E (4/3 CMOS)": Camera("Mavic 3E", 17.3, 13.0, 12.29, 5280, 3956, 0.7),
    "DJI Phantom 4 RTK": Camera("P4RTK FC6310R", 13.2, 8.8, 8.8, 5472, 3648, 2.0),
    "DJI Zenmuse P1 35mm": Camera("Zenmuse P1 35mm", 35.9, 24.0, 35.0, 8192, 5460, 0.7),
    # ---- DJI Enterprise cameras (DJI spec pages). DJI lists 35 mm-equivalent focal lengths for the
    # built-in cameras; actual focal lengths below are derived from sensor size (24 mm eq. on 4/3 = 12.29 mm).
    "DJI Zenmuse P1 24mm": Camera("Zenmuse P1 24mm", 35.9, 24.0, 24.0, 8192, 5460, 0.7),
    "DJI Zenmuse P1 50mm": Camera("Zenmuse P1 50mm", 35.9, 24.0, 50.0, 8192, 5460, 0.7),
    "DJI Zenmuse L1 (RGB)": Camera("Zenmuse L1 1in 20MP", 13.2, 8.8, 8.8, 5472, 3648, 1.0),
    "DJI Zenmuse L2 (RGB)": Camera("Zenmuse L2 4/3 20MP", 17.3, 13.0, 12.29, 5280, 3956, 0.7),
    "DJI Zenmuse L3 (RGB 100 MP)": Camera("Zenmuse L3 4/3 100MP", 17.3, 11.53, 13.45, 12288, 8192, 1.0),
    "DJI Mavic 3T wide (12 MP)": Camera("Mavic 3T wide 1/2in", 6.4, 4.8, 4.44, 4000, 3000, 2.0),
    "DJI Matrice 30 wide": Camera("M30 wide 1/2in 12MP", 6.4, 4.8, 4.5, 4000, 3000, 2.0),
    "DJI 4/3 20 MP wide (M4E/M4D/M3D)": Camera("4/3 CMOS 20MP wide", 17.3, 13.0, 12.29, 5280, 3956, 0.7),
    "DJI 1/1.3in 48 MP wide (M4T/M4TD/M3TD)": Camera("1/1.3in 48MP wide", 9.6, 7.2, 6.72, 8064, 6048, 0.7),
    # DJI Mini series: 12 MP mode (2 s fastest timed interval). For full resolution set the image
    # size to the 48/50 MP figure and the minimum photo interval to 5 s (DJI spec: timed 48/50 MP >= 5 s).
    "DJI Mini 3 Pro (12 MP)": Camera("Mini 3 Pro 1/1.3in", 9.6, 7.2, 6.72, 4032, 3024, 2.0),
    "DJI Mini 4 Pro (12 MP)": Camera("Mini 4 Pro 1/1.3in", 9.6, 7.2, 6.72, 4032, 3024, 2.0),
    "DJI Mini 5 Pro (12.5 MP)": Camera("Mini 5 Pro 1in", 13.1, 9.83, 9.1, 4096, 3072, 2.0),
    "DJI Air 3": Camera("Air 3 wide", 9.6, 7.2, 6.72, 8064, 6048, 2.0),
    "Autel EVO II Pro": Camera("EVO II Pro 1in", 13.2, 8.8, 10.57, 5472, 3648, 2.0),
    "Sony RX1R II (fixed-wing)": Camera("RX1R II", 35.9, 24.0, 35.0, 7952, 5304, 1.0),
    "Sony a6100 20mm": Camera("a6100 20mm", 23.5, 15.6, 20.0, 6000, 4000, 1.0),
}

DRONES = {
    # ---- DJI Enterprise (specs: enterprise.dji.com spec pages). drone_enum = WPML
    # "droneEnumValue,payloadEnumValue,droneSubEnumValue" from the DJI Cloud API docs. Where DJI has not
    # published a code, it is left blank: export with a route made on the aircraft as the DJI template.
    # Combos with a gimbal camera: mass includes the camera and the flight time is DJI's no-payload time
    # scaled for the extra mass (estimate) - check against your own flights.
    "DJI Matrice 4E": Drone(
        "DJI Matrice 4E", "dji_wpml", CAMERAS["DJI 4/3 20 MP wide (M4E/M4D/M3D)"],
        1.219, 0.2, 49, 15, 12, 99.5, 0.12, drone_enum="99,88,0",
        notes="Mechanical shutter, 0.5 s interval. Pilot 2 route import (KMZ)."),
    "DJI Matrice 4T": Drone(
        "DJI Matrice 4T", "dji_wpml", CAMERAS["DJI 1/1.3in 48 MP wide (M4T/M4TD/M3TD)"],
        1.219, 0.2, 49, 15, 12, 99.5, 0.12, drone_enum="99,89,1",
        notes="Wide camera 48 MP, electronic shutter - keep speed modest to limit rolling-shutter blur."),
    "DJI Matrice 4D (Dock 3)": Drone(
        "DJI Matrice 4D (Dock 3)", "dji_wpml", CAMERAS["DJI 4/3 20 MP wide (M4E/M4D/M3D)"],
        2.09, 0.0, 54, 15, 12, 149.9, 0.15, drone_enum="100,98,0",
        notes="Dock aircraft: import the KMZ into FlightHub 2 > Flight Route Library."),
    "DJI Matrice 4TD (Dock 3)": Drone(
        "DJI Matrice 4TD (Dock 3)", "dji_wpml", CAMERAS["DJI 1/1.3in 48 MP wide (M4T/M4TD/M3TD)"],
        2.09, 0.0, 54, 15, 12, 149.9, 0.15, drone_enum="100,99,1",
        notes="Dock aircraft: import the KMZ into FlightHub 2 > Flight Route Library."),
    "DJI Matrice 3D (Dock 2)": Drone(
        "DJI Matrice 3D (Dock 2)", "dji_wpml", CAMERAS["DJI 4/3 20 MP wide (M4E/M4D/M3D)"],
        1.61, 0.0, 50, 15, 12, 115.2, 0.13, drone_enum="91,80,0",
        notes="Dock aircraft: import the KMZ into FlightHub 2 > Flight Route Library."),
    "DJI Matrice 3TD (Dock 2)": Drone(
        "DJI Matrice 3TD (Dock 2)", "dji_wpml", CAMERAS["DJI 1/1.3in 48 MP wide (M4T/M4TD/M3TD)"],
        1.61, 0.0, 50, 15, 12, 115.2, 0.13, drone_enum="91,81,1",
        notes="Dock aircraft: import the KMZ into FlightHub 2. 48 MP photos need >= 1 s between shots."),
    "DJI Mavic 3 Enterprise": Drone(
        "DJI Mavic 3 Enterprise", "dji_wpml", CAMERAS["DJI Mavic 3E (4/3 CMOS)"],
        0.915, 0.135, 45, 15, 12, 77, 0.10, drone_enum="77,66,0",
        notes="EU C1 battery: 65 Wh, 36 min - edit battery and flight time if you fly that pack."),
    "DJI Mavic 3 Thermal (3T)": Drone(
        "DJI Mavic 3 Thermal (3T)", "dji_wpml", CAMERAS["DJI Mavic 3T wide (12 MP)"],
        0.92, 0.13, 45, 15, 12, 77, 0.10, drone_enum="77,67,1",
        notes="Wide camera planned in 12 MP mode (2 s interval); 48 MP does not support 2 s timed shots."),
    "DJI Mavic 3 Multispectral (3M)": Drone(
        "DJI Mavic 3 Multispectral (3M)", "dji_wpml", CAMERAS["DJI Mavic 3E (4/3 CMOS)"],
        0.951, 0.0, 43, 15, 12, 77, 0.10, drone_enum="77,68,2",
        notes="Planned on the 20 MP RGB camera. Multispectral bands (5 MP, 25 mm eq.) have a coarser GSD - "
              "check overlap for the band cameras."),
    "DJI Matrice 30": Drone(
        "DJI Matrice 30", "dji_wpml", CAMERAS["DJI Matrice 30 wide"],
        3.77, 0.3, 41, 15, 12, 2 * 131.6, 0.35, drone_enum="67,52,0",
        notes="Two TB30 batteries. Wide camera is 12 MP 1/2in - fine for inspection, coarse for survey."),
    "DJI Matrice 30T": Drone(
        "DJI Matrice 30T", "dji_wpml", CAMERAS["DJI Matrice 30 wide"],
        3.77, 0.3, 41, 15, 12, 2 * 131.6, 0.35, drone_enum="67,53,1",
        notes="Two TB30 batteries. Wide camera is 12 MP 1/2in - fine for inspection, coarse for survey."),
    "DJI Matrice 350 RTK + P1": Drone(
        "DJI Matrice 350 RTK + P1", "dji_wpml", CAMERAS["DJI Zenmuse P1 35mm"],
        7.27, 0.16, 46, 15, 12, 2 * 263.2, 0.95, drone_enum="89,50,0",
        notes="P1 35 mm lens; change focal length to 24 or 50 mm for the other lenses. Two TB65 batteries."),
    "DJI Matrice 350 RTK + L2": Drone(
        "DJI Matrice 350 RTK + L2", "dji_wpml", CAMERAS["DJI Zenmuse L2 (RGB)"],
        7.38, 0.05, 45, 15, 12, 2 * 263.2, 0.95, drone_enum="89,,0",
        notes="Planned for the L2 RGB camera. DJI hasn't published the L2 WPML payload code: export with a "
              "Pilot 2 route made with the L2 as the DJI template. LiDAR: plan overlap to your LiDAR spec."),
    "DJI Matrice 350 RTK + L1": Drone(
        "DJI Matrice 350 RTK + L1", "dji_wpml", CAMERAS["DJI Zenmuse L1 (RGB)"],
        7.40, 0.03, 45, 15, 12, 2 * 263.2, 0.95, drone_enum="89,90742,0",
        notes="Planned for the L1 RGB camera."),
    "DJI Matrice 300 RTK + P1": Drone(
        "DJI Matrice 300 RTK + P1", "dji_wpml", CAMERAS["DJI Zenmuse P1 35mm"],
        7.1, 1.9, 46, 15, 15, 2 * 274, 0.95, drone_enum="60,50,0",
        notes="P1 35 mm lens; change focal length for the 24 / 50 mm lenses. Two TB60 batteries."),
    "DJI Matrice 300 RTK + L1": Drone(
        "DJI Matrice 300 RTK + L1", "dji_wpml", CAMERAS["DJI Zenmuse L1 (RGB)"],
        7.23, 1.77, 45, 15, 15, 2 * 274, 0.95, drone_enum="60,90742,0",
        notes="Planned for the L1 RGB camera. Two TB60 batteries."),
    "DJI Matrice 400 + P1": Drone(
        "DJI Matrice 400 + P1", "dji_wpml", CAMERAS["DJI Zenmuse P1 35mm"],
        10.54, 5.2, 59, 15, 12, 977, 1.3, drone_enum="",
        notes="DJI hasn't published the Matrice 400 WPML aircraft code: export with a Pilot 2 route made on "
              "this aircraft as the DJI template. TB100 battery."),
    "DJI Matrice 400 + L2": Drone(
        "DJI Matrice 400 + L2", "dji_wpml", CAMERAS["DJI Zenmuse L2 (RGB)"],
        10.65, 5.1, 59, 15, 12, 977, 1.3, drone_enum="",
        notes="Use a Pilot 2 route made on this aircraft with the L2 as the DJI template (codes not "
              "published). Planned for the L2 RGB camera."),
    "DJI Matrice 400 + L3": Drone(
        "DJI Matrice 400 + L3", "dji_wpml", CAMERAS["DJI Zenmuse L3 (RGB 100 MP)"],
        11.49, 4.25, 52, 15, 12, 977, 1.3, drone_enum="",
        notes="Use a Pilot 2 route made on this aircraft with the L3 as the DJI template (codes not "
              "published). Planned for the L3 100 MP RGB camera (1 s interval); focal length estimated."),
    "DJI Phantom 4 RTK": Drone(
        "DJI Phantom 4 RTK", "dji_wpml", CAMERAS["DJI Phantom 4 RTK"],
        1.391, 0.0, 30, 14, 10, 89.2, 0.17),
    # ---- DJI Mini Pro series (specs: DJI). Masses are the standard battery's take-off mass;
    # Plus batteries make the aircraft heavier than 249 g - check local rules.
    "DJI Mini 3 Pro": Drone(
        "DJI Mini 3 Pro", "litchi", CAMERAS["DJI Mini 3 Pro (12 MP)"],
        0.249, 0.0, 34, 16, 10.7, 18.1, 0.05, max_waypoints=99,
        notes="Fly with Litchi Pilot (DJI MSDK v5 supports the Mini 3 Pro)."),
    "DJI Mini 4 Pro": Drone(
        "DJI Mini 4 Pro", "litchi", CAMERAS["DJI Mini 4 Pro (12 MP)"],
        0.249, 0.0, 34, 16, 10.7, 18.96, 0.05, max_waypoints=99,
        notes="Fly with Litchi Pilot (DJI MSDK supports the Mini 4 Pro with the RC-N2), or DJI Fly "
              "waypoints using the DJI Fly export."),
    "DJI Mini 4 Pro (Plus battery)": Drone(
        "DJI Mini 4 Pro (Plus battery)", "litchi", CAMERAS["DJI Mini 4 Pro (12 MP)"],
        0.249, 0.0, 45, 16, 10.7, 28.4, 0.05, max_waypoints=99,
        notes="Plus battery: heavier than 249 g. Not sold in Europe."),
    "DJI Mini 5 Pro": Drone(
        "DJI Mini 5 Pro", "dji_fly", CAMERAS["DJI Mini 5 Pro (12.5 MP)"],
        0.2499, 0.0, 36, 18, 12, 19.52, 0.06, photo_trigger="timed",
        notes="No DJI SDK yet: fly in DJI Fly waypoints by replacing a DJI Fly mission file "
              "(DJI Fly export). Photos: timed shooting or stop-and-shoot."),
    "DJI Mini 5 Pro (Plus battery)": Drone(
        "DJI Mini 5 Pro (Plus battery)", "dji_fly", CAMERAS["DJI Mini 5 Pro (12.5 MP)"],
        0.2499, 0.0, 52, 18, 12, 33.51, 0.06, photo_trigger="timed",
        notes="Plus battery: heavier than 249 g. Not sold in Europe."),
    "DJI Air 3 (Litchi/WPML)": Drone(
        "DJI Air 3 (Litchi/WPML)", "litchi", CAMERAS["DJI Air 3"],
        0.72, 0.0, 46, 21, 12, 62, 0.08, max_waypoints=99),
    "Autel EVO II Pro": Drone(
        "Autel EVO II Pro", "autel", CAMERAS["Autel EVO II Pro"],
        1.191, 0.0, 40, 20, 12, 82, 0.14),
    "ArduPilot / PX4 Quad (generic)": Drone(
        "ArduPilot / PX4 Quad (generic)", "mavlink", CAMERAS["Sony a6100 20mm"],
        3.5, 1.5, 25, 15, 10, 222, 0.5),
    "ArduPilot / PX4 Fixed-wing (generic)": Drone(
        "ArduPilot / PX4 Fixed-wing (generic)", "mavlink", CAMERAS["Sony RX1R II (fixed-wing)"],
        4.5, 0.6, 75, 25, 12, 222, 0.0, fixed_wing=True),
}


def drone_names():
    return list(DRONES.keys())


ALIASES = {"DJI Mini 4 Pro (Litchi/WPML)": "DJI Mini 4 Pro"}     # names used by v1.0-1.6


def get_drone(name):
    """Return an editable copy of a stock profile."""
    d = DRONES[ALIASES.get(name, name)]
    return replace(d, camera=replace(d.camera))
