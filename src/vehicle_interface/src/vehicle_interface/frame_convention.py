# -*- coding: utf-8 -*-
"""Shared O-XYZ heading and tracked-chassis sign convention.

Convention used by every Qianxun script:
  * heading phi is measured from global +Y;
  * counterclockwise heading/yaw-rate is positive;
  * body +x points forward and body +y points left;
  * e_y is positive on the left side of the reference path;
  * omega = (v_R - v_L) / B.
"""

import math


def wrap_deg(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def wrap_rad(angle_rad):
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def heading_from_delta(dx, dy):
    """Heading from +Y, CCW-positive, for a planar displacement."""
    return wrap_deg(math.degrees(math.atan2(-float(dx), float(dy))))


def tangent(phi):
    """Forward unit vector in O-XYZ for heading phi [rad]."""
    return -math.sin(phi), math.cos(phi)


def left_normal(phi):
    """Left unit normal in O-XYZ for heading phi [rad]."""
    return -math.cos(phi), -math.sin(phi)


def lateral_error_left(x, y, x_ref, y_ref, phi_ref):
    """Signed lateral error: positive means vehicle is left of the path."""
    nx, ny = left_normal(phi_ref)
    return (float(x) - float(x_ref)) * nx + (float(y) - float(y_ref)) * ny


def nmea_cog_to_enu_heading(cog_deg):
    """NMEA COG (north, clockwise+) -> ENU heading (+N, CCW+)."""
    return wrap_deg(-float(cog_deg))


def enu_heading_to_oxyz(head_enu_deg, yaw_enu_to_oxyz_deg):
    """Rotate a +N/CCW ENU heading into the configured O-XYZ axes."""
    return wrap_deg(float(head_enu_deg) - float(yaw_enu_to_oxyz_deg))


def oxyz_heading_to_enu(head_oxyz_deg, yaw_enu_to_oxyz_deg):
    return wrap_deg(float(head_oxyz_deg) + float(yaw_enu_to_oxyz_deg))


def enu_velocity(speed, head_enu_rad):
    """Return east/north velocity for heading measured from +N, CCW+."""
    return -float(speed) * math.sin(head_enu_rad), float(speed) * math.cos(head_enu_rad)


def body_to_tracks(speed, yaw_rate, track_width):
    """v, omega -> left/right track speeds for CCW-positive omega."""
    half = 0.5 * float(track_width)
    return float(speed) - float(yaw_rate) * half, float(speed) + float(yaw_rate) * half


def tracks_to_body(v_left, v_right, track_width):
    """Left/right track speeds -> v, omega for CCW-positive omega."""
    return 0.5 * (float(v_left) + float(v_right)), (float(v_right) - float(v_left)) / float(track_width)
