"""Pure conversions shared by the ROS/ZMQ compatibility bridge."""

import datetime
import math


def rmc_measurement_epoch(utc_text, date_text):
    """Convert NMEA RMC ``hhmmss.sss``/``ddmmyy`` into Unix UTC."""
    try:
        utc_text = str(utc_text).strip()
        date_text = str(date_text).strip()
        if len(utc_text) < 6 or len(date_text) != 6:
            return None
        hour = int(utc_text[0:2])
        minute = int(utc_text[2:4])
        second_value = float(utc_text[4:])
        second = int(second_value)
        microsecond = int(round((second_value - second) * 1e6))
        if microsecond >= 1000000:
            second += 1
            microsecond -= 1000000
        day = int(date_text[0:2])
        month = int(date_text[2:4])
        year_2 = int(date_text[4:6])
        year = 2000 + year_2 if year_2 < 80 else 1900 + year_2
        return datetime.datetime(
            year, month, day, hour, minute, second, microsecond,
            tzinfo=datetime.timezone.utc,
        ).timestamp()
    except (TypeError, ValueError, OverflowError):
        return None


def resolve_gnss_measurement_timestamp(
        rmc_epoch, receive_epoch, fixed_latency_s=0.0,
        previous_epoch=None, max_clock_residual_s=0.30):
    """Choose a ROS-compatible GNSS measurement stamp and audit its clock.

    The NMEA UTC epoch is preferred only when it is monotonic and agrees with
    the host receive clock after the configured receiver latency is removed.
    An inconsistent UTC stamp is replaced with the latency-corrected receive
    stamp so downstream messages never carry a wildly old/future timestamp;
    ``clock_consistent`` remains false so the localization health gate can
    fail closed.
    """
    receive_epoch = float(receive_epoch)
    if not math.isfinite(receive_epoch):
        raise ValueError("receive_epoch must be finite")
    latency_s = max(0.0, float(fixed_latency_s))
    fallback_epoch = receive_epoch - latency_s
    limit_s = max(0.0, float(max_clock_residual_s))

    try:
        parsed_rmc_epoch = float(rmc_epoch)
        rmc_available = math.isfinite(parsed_rmc_epoch)
    except (TypeError, ValueError, OverflowError):
        parsed_rmc_epoch = float("nan")
        rmc_available = False

    clock_offset_s = (
        parsed_rmc_epoch - receive_epoch
        if rmc_available else float("nan")
    )
    clock_residual_s = (
        parsed_rmc_epoch - fallback_epoch
        if rmc_available else float("nan")
    )
    clock_consistent = bool(
        not rmc_available or abs(clock_residual_s) <= limit_s
    )

    if rmc_available and not clock_consistent:
        source = "ipc_receive_latency_clock_mismatch"
        measurement_epoch = fallback_epoch
    elif not rmc_available:
        source = "ipc_receive_latency_missing_rmc"
        measurement_epoch = fallback_epoch
    elif (previous_epoch is not None
          and parsed_rmc_epoch <= float(previous_epoch)):
        source = "ipc_receive_latency_nonmonotonic_rmc"
        measurement_epoch = fallback_epoch
    else:
        source = "rmc_utc"
        measurement_epoch = parsed_rmc_epoch

    return {
        "measurement_epoch": measurement_epoch,
        "source": source,
        "rmc_available": rmc_available,
        "clock_consistent": clock_consistent,
        "clock_offset_s": clock_offset_s,
        "clock_residual_s": clock_residual_s,
    }


def time_alignment_is_valid(error_s, max_abs_error_s):
    """Return whether two sensor samples are sufficiently time-aligned."""
    try:
        error_s = float(error_s)
        max_abs_error_s = float(max_abs_error_s)
    except (TypeError, ValueError, OverflowError):
        return False
    return bool(
        math.isfinite(error_s)
        and math.isfinite(max_abs_error_s)
        and max_abs_error_s >= 0.0
        and abs(error_s) <= max_abs_error_s
    )


def wrap_radians(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def oxyz_heading_to_ros_yaw(head_deg):
    """Convert +Y-origin, CCW-positive O-XYZ heading to ROS +X yaw."""
    return wrap_radians(math.radians(float(head_deg)) + 0.5 * math.pi)


def quaternion_from_rpy(roll, pitch, yaw):
    """Return ROS quaternion (x, y, z, w) from radians."""
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def _flag_is_true(state, key):
    """Treat malformed or missing external status fields as unsafe."""
    try:
        return bool(int(state.get(key, 0)))
    except (TypeError, ValueError, OverflowError):
        return False


def state_is_healthy(state, require_fusion=True, require_position=True,
                     require_control_frame=True, require_known_heading=False):
    if not isinstance(state, dict):
        return False
    if require_fusion and not _flag_is_true(state, "FusionValid"):
        return False
    if require_position and not _flag_is_true(state, "PositionValid"):
        return False
    if require_control_frame and not _flag_is_true(state, "ControlFrameReady"):
        return False
    if require_known_heading and str(state.get("HeadingMode", "UNKNOWN")).upper() == "UNKNOWN":
        return False
    try:
        return all(
            math.isfinite(float(state.get(key, float("nan"))))
            for key in ("UTM_x", "UTM_y", "Head")
        )
    except (TypeError, ValueError, OverflowError):
        return False
