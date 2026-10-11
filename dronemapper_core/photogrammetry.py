"""Camera / ground-sample-distance maths.

Convention: the image's long side (image_width_px) lies ACROSS the flight
line, which is how most mapping drones are configured. Set
``portrait=True`` to swap it.
"""
import math


def gsd_from_height(camera, height_m):
    """Ground sample distance in cm/px at a given height above ground."""
    return camera.sensor_width_mm * height_m * 100.0 / (camera.focal_length_mm * camera.image_width_px)


def height_from_gsd(camera, gsd_cm):
    """Height above ground (m) that gives the requested GSD (cm/px)."""
    return gsd_cm * camera.focal_length_mm * camera.image_width_px / (camera.sensor_width_mm * 100.0)


def footprint(camera, height_m, portrait=False):
    """(across_track_m, along_track_m) ground footprint of one image."""
    gsd_m = gsd_from_height(camera, height_m) / 100.0
    w, h = camera.image_width_px * gsd_m, camera.image_height_px * gsd_m
    return (h, w) if portrait else (w, h)


def line_spacing(camera, height_m, side_overlap_pct, portrait=False):
    across, _ = footprint(camera, height_m, portrait)
    return across * (1.0 - side_overlap_pct / 100.0)


def trigger_distance(camera, height_m, front_overlap_pct, portrait=False):
    _, along = footprint(camera, height_m, portrait)
    return along * (1.0 - front_overlap_pct / 100.0)


def max_speed_for_camera(camera, height_m, front_overlap_pct, shutter_s=1 / 1000.0,
                         max_blur_px=0.5, portrait=False):
    """Fastest ground speed that the camera can keep up with.

    Limited by (a) the minimum photo interval and (b) motion blur.
    Returns (speed_ms, limiting_reason).
    """
    td = trigger_distance(camera, height_m, front_overlap_pct, portrait)
    v_interval = td / max(camera.min_trigger_interval_s, 0.1)
    gsd_m = gsd_from_height(camera, height_m) / 100.0
    v_blur = max_blur_px * gsd_m / max(shutter_s, 1e-5)
    if v_interval <= v_blur:
        return v_interval, "photo interval"
    return v_blur, "motion blur"


def photos_for_length(length_m, trig_dist_m):
    if trig_dist_m <= 0:
        return 0
    return int(math.floor(length_m / trig_dist_m)) + 1
