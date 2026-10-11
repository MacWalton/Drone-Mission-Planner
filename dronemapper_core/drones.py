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
    drone_enum: str = ""       # DJI WPML droneEnumValue / payloadEnumValue "a,b"
    photo_trigger: str = "distance"   # default photo trigger: distance | timed | stop
    notes: str = ""

    def to_dict(self):
        return asdict(self)


CAMERAS = {
    "DJI Mavic 3E (4/3 CMOS)": Camera("Mavic 3E", 17.3, 13.0, 12.29, 5280, 3956, 0.7),
    "DJI Phantom 4 RTK": Camera("P4RTK FC6310R", 13.2, 8.8, 8.8, 5472, 3648, 2.0),
    "DJI Zenmuse P1 35mm": Camera("Zenmuse P1 35mm", 35.9, 24.0, 35.0, 8192, 5460, 0.7),
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
    "DJI Mavic 3 Enterprise": Drone(
        "DJI Mavic 3 Enterprise", "dji_wpml", CAMERAS["DJI Mavic 3E (4/3 CMOS)"],
        0.915, 0.2, 42, 15, 12, 77, 0.10, drone_enum="77,66"),
    "DJI Matrice 350 RTK + P1": Drone(
        "DJI Matrice 350 RTK + P1", "dji_wpml", CAMERAS["DJI Zenmuse P1 35mm"],
        7.7, 2.7, 42, 23, 12, 2 * 263, 0.95, drone_enum="89,50"),
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
