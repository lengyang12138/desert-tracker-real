# -*- coding: utf-8 -*-
"""POS_qianxun.py - pure Qianxun RMC positioning bridge.

This node is a temporary replacement for POS_MPC_xiepo.py while the CHCNAV
INS is unavailable. It reads Qianxun NMEA RMC sentences from a serial port and
publishes the same downstream interface as the slope POS node:

    ZMQ CurGNSS, ROS /fusion_location, ROS /bus/pose, ROS /bus/sensor

Pure GNSS has no altitude, roll, pitch, or gyro. Those fields are explicitly
filled with zero so path_plan_MPC_qianxun.py and control_MPC_qianxun.py can keep
the same state contract. Heading is the GNSS course over ground, so it is only
valid while the vehicle is moving forward.
"""

import json
import math
import os
import sys
import time
import csv
import datetime

from .frame_convention import (
    enu_heading_to_oxyz,
    enu_velocity,
    nmea_cog_to_enu_heading,
    oxyz_heading_to_enu,
    wrap_deg,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, ".."))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

try:
    import serial
    import zmq
    import rospy
    import geometry_msgs.msg
    import std_msgs.msg
    from .pro_context import proContext
except ImportError as exc:
    print("缺少运行依赖: {}。需要 ROS、pyserial、pyzmq 环境。".format(exc))
    sys.exit(1)


def _env_optional_float(name, default=None):
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    return float(value)


class Cfg:
    # Qianxun serial output. Adjust on the vehicle if the USB-RS232 name changes.
    QX_PORT = os.environ.get("QX_PORT", "/dev/ttyUSB0")
    QX_BAUD = int(os.environ.get("QX_BAUD", "115200"))

    GnssAddr = "tcp://*:8080"

    # Start-point guide. POS_qianxun can read the first point of the current
    # qianxun map and print guidance so the real vehicle can be parked at the
    # planned trajectory start before enabling MPC.
    GUIDE_START_ENABLE = os.environ.get("QX_GUIDE_START_ENABLE", "1").lower() not in ("0", "false", "no")
    GUIDE_MAP_FILE = os.environ.get(
        "QX_GUIDE_MAP_FILE",
        os.path.join(PROJECT_DIR, "scripts_motion_engine", "potato_planner1",
                     "data", "map", "guihua_qianxun.map"))
    GUIDE_POS_TOL = float(os.environ.get("QX_GUIDE_POS_TOL", "0.35"))       # m
    GUIDE_HEAD_TOL_DEG = float(os.environ.get("QX_GUIDE_HEAD_TOL_DEG", "10.0"))
    GUIDE_REQUIRE_HEADING = os.environ.get("QX_GUIDE_REQUIRE_HEADING", "0").lower() in ("1", "true", "yes")

    # O-XYZ convention shared by every Qianxun script:
    # heading is measured from +Y, CCW positive; body +x forward, +y left.
    # For first tests, leave ORIGIN_* as None and park at the desired origin
    # before starting this script. Reuse the printed values for repeatability.
    ORIGIN_LAT = _env_optional_float("QX_ORIGIN_LAT")
    ORIGIN_LON = _env_optional_float("QX_ORIGIN_LON")
    ORIGIN_ALT = _env_optional_float("QX_ORIGIN_ALT", 0.0)
    # Positive value means the O-XYZ axes are rotated CCW from ENU axes.
    YAW_ENU_TO_OXYZ_DEG = float(
        os.environ.get("QX_YAW_ENU_TO_OXYZ_DEG", "71.6")
    )
    ZERO_FIRST_FIX = os.environ.get("QX_ZERO_FIRST_FIX", "0").lower() in ("1", "true", "yes")
    ORIGIN_WARN_METERS = float(os.environ.get("QX_ORIGIN_WARN_METERS", "2.0"))

    # RMC course over ground is unreliable when stopped or creeping.
    COG_MIN_SPEED = 0.25       # m/s
    # Logical heading used by the controllers. For the current tail-first test,
    # keep this at 0: ZMQ reverses chassis speed, while GNSS COG already points
    # along the logical planned motion. Use 180 only after a stationary-capable
    # body-heading sensor proves its reported body axis is opposite to logical +x.
    HEADING_OFFSET_DEG = float(os.environ.get("QX_HEADING_OFFSET_DEG", "0"))
    OMEGA_ALPHA = 0.25         # low-pass for heading-rate estimate
    ACC_ALPHA = 0.25           # low-pass for speed derivative
    OMEGA_LIMIT = 2.0          # rad/s, keep GNSS heading jumps harmless
    # Initial heading seed for pure GNSS. RMC course-over-ground is not a body
    # yaw angle when stopped, so seed Head from the planned start or from an
    # explicit environment value until the vehicle moves fast enough.
    INIT_HEADING_SOURCE = os.environ.get("QX_INIT_HEADING_SOURCE", "auto").strip().lower()
    INIT_HEAD_OXYZ_DEG = os.environ.get("QX_INIT_HEAD_OXYZ_DEG", "").strip()
    PRINT_HZ = 5
    CHECKSUM_STRICT = os.environ.get("QX_CHECKSUM_STRICT", "1").lower() not in ("0", "false", "no")
    DEBUG_RAW = os.environ.get("QX_DEBUG_RAW", "0").lower() in ("1", "true", "yes")
    DEBUG_DIAG = os.environ.get("QX_DEBUG_DIAG", "1").lower() not in ("0", "false", "no")
    DIAG_PERIOD = float(os.environ.get("QX_DIAG_PERIOD", "2.0"))
    DTR = os.environ.get("QX_DTR", "1").lower() not in ("0", "false", "no")
    RTS = os.environ.get("QX_RTS", "0").lower() in ("1", "true", "yes")
    # 默认不再自动发送setGPRMC/setGPGGA，避免脚本启动时覆盖用户在MR02
    # 网页/上位机中设定的20 Hz输出频率。仅在确认命令语义和目标频率后，
    # 才通过QX_AUTO_CONFIG=1及QX_CONFIG_COMMANDS显式启用。
    AUTO_CONFIG = os.environ.get("QX_AUTO_CONFIG", "0").lower() not in ("0", "false", "no")
    CONFIG_COMMANDS = os.environ.get(
        "QX_CONFIG_COMMANDS",
        "getcom;setGPRMC,com2,0.05;setGPGGA,com2,0.05",
    )
    CONFIG_EOL = os.environ.get("QX_CONFIG_EOL", "\r\n")
    CONFIG_RETRY_SEC = float(os.environ.get("QX_CONFIG_RETRY_SEC", "5.0"))

    # Position quality and filtering.
    # Pure GNSS can drift 1~3 m while stopped. MPC would treat that as real
    # lateral error, so publish filtered O-XYZ coordinates by default.
    REQUIRE_VALID_RMC = os.environ.get("QX_REQUIRE_VALID_RMC", "1").lower() not in ("0", "false", "no")
    POS_FILTER_ENABLE = os.environ.get("QX_POS_FILTER_ENABLE", "1").lower() not in ("0", "false", "no")
    # 恢复跟踪时地速常在0.04~0.15 m/s。原0.08 m/s门限会把缓行误判
    # 为静止，并在0.35 m范围内冻结坐标，形成“缓行—跳点—停车”。
    STATIC_SPEED_EPS = float(os.environ.get("QX_STATIC_SPEED_EPS", "0.025"))      # m/s
    STATIC_FREEZE_RADIUS = float(os.environ.get("QX_STATIC_FREEZE_RADIUS", "0.08"))  # m
    STATIC_ALPHA = float(os.environ.get("QX_STATIC_ALPHA", "0.15"))
    MOVING_ALPHA = float(os.environ.get("QX_MOVING_ALPHA", "1.00"))
    MAX_POS_STEP = float(os.environ.get("QX_MAX_POS_STEP", "1.20"))              # m/sample
    # 动态位置门控：先判断本帧是否符合车辆在相邻GNSS帧间的物理可达范围，
    # 再进入静止/运动滤波。低通滤波本身不能区分真实运动和米级跳点。
    POSITION_GATE_ENABLE = os.environ.get("QX_POSITION_GATE_ENABLE", "1").lower() not in ("0", "false", "no")
    # 实车轨迹跟踪默认只允许RTK固定解(q=4)更新控制位置。q=2差分码解、
    # q=5浮点解以及其他质量等级仍可记录为原始测量，但不能推动对外CP坐标。
    POSITION_GATE_ALLOWED_QUALITIES = tuple(map(
        int, os.environ.get("QX_POSITION_GATE_QUALITIES", "4").split(",")
    ))
    POSITION_GATE_MARGIN = float(os.environ.get("QX_POSITION_GATE_MARGIN", "0.25"))
    POSITION_GATE_SPEED_SCALE = float(os.environ.get("QX_POSITION_GATE_SPEED_SCALE", "1.80"))
    POSITION_GATE_ABS_MAX = float(os.environ.get("QX_POSITION_GATE_ABS_MAX", "1.20"))
    POSITION_GATE_MIN_DT = float(os.environ.get("QX_POSITION_GATE_MIN_DT", "0.02"))
    POSITION_GATE_MAX_DT = float(os.environ.get("QX_POSITION_GATE_MAX_DT", "0.50"))
    # 正常接收路径也设置单帧直接更新上限，防止长时间无效后第一帧q=4以
    # 0.4~1.2 m的大步直接跳到新位置。超过该值必须进入下面的重捕获状态机。
    POSITION_DIRECT_ACCEPT_MAX = float(os.environ.get("QX_POSITION_DIRECT_ACCEPT_MAX", "0.10"))
    # 重捕获规则：候选坐标只在门控内部观察，不修改对外CP。连续5帧q=4
    # 落在同一小范围后，才以每个有效GNSS帧最多0.05 m平滑靠近；每次真正
    # 修改CP时PositionValid=1且PositionUpdateSeq同步递增，不再出现
    # “CP移动了但Valid=0、Seq不变”的状态撕裂。
    POSITION_REACQUIRE_COUNT = int(os.environ.get("QX_POSITION_REACQUIRE_COUNT", "5"))
    POSITION_REACQUIRE_STEP = float(os.environ.get("QX_POSITION_REACQUIRE_STEP", "0.05"))
    POSITION_REACQUIRE_RADIUS = float(os.environ.get("QX_POSITION_REACQUIRE_RADIUS", "0.12"))

    # Static-position filter analysis. Leave enabled during field tests: Ctrl+C
    # saves CSV, statistics, and figures comparing Raw_x/Raw_y vs filtered X/Y.
    POS_ANALYSIS_ENABLE = os.environ.get("QX_POS_ANALYSIS_ENABLE", "1").lower() not in ("0", "false", "no")
    POS_ANALYSIS_DIR = os.environ.get("QX_POS_ANALYSIS_DIR", os.path.join(BASE_DIR, "pos_static_analysis"))
    POS_ANALYSIS_STATIC_SPEED_EPS = float(os.environ.get("QX_ANALYSIS_STATIC_SPEED_EPS", str(STATIC_SPEED_EPS)))
    POS_ANALYSIS_MAX_SAMPLES = int(os.environ.get("QX_POS_ANALYSIS_MAX_SAMPLES", "30000"))


WGS84_A = 6378137.0
WGS84_E2 = 0.00669438

_origin_ready = False
_o_lat = _o_lon = _o_h = 0.0
_o_xe = _o_ye = _o_ze = 0.0
_origin_from_first_fix = False
_origin_reported = False

_last_head_enu = None
_last_speed = None
_last_t = None
_omega_lpf = 0.0
_acc_lpf = 0.0
_last_print = 0.0
_last_diag = 0.0
_last_config_t = 0.0
_pos_filter_ready = False
_filt_x = 0.0
_filt_y = 0.0
_last_raw_x = None
_last_raw_y = None
_position_gate_ready = False
_position_gate_x = 0.0
_position_gate_y = 0.0
_position_gate_t = None
_position_gate_speed = 0.0
_position_update_seq = 0
_position_reject_total = 0
_position_reject_streak = 0
_position_candidate_x = None
_position_candidate_y = None
_position_candidate_streak = 0
_last_gga = {"quality": 0, "sats": 0, "hdop": 99.9, "alt": 0.0}
_start_ref = None
_start_arrived_reported = False
_heading_seeded = False
_analysis_samples = []
_analysis_t0 = None

diag = {
    "bytes": 0,
    "lines": 0,
    "rmc_seen": 0,
    "rmc_ok": 0,
    "gga_seen": 0,
    "gga_ok": 0,
    "checksum_bad": 0,
    "parse_bad": 0,
    "last_line": "",
}


current_state = {
    "Lat": 0.0, "Lon": 0.0, "Alt": 0.0,
    "UTM_x": 0.0, "UTM_y": 0.0, "Z": 0.0,
    "OriginLat": None, "OriginLon": None, "OriginAlt": None,
    "OriginMode": "uninitialized",
    "YawEnuToOxyzDeg": Cfg.YAW_ENU_TO_OXYZ_DEG,
    "Raw_x": 0.0, "Raw_y": 0.0, "PosJump": 0.0, "PosFiltered": 0,
    "PosFilterEnable": int(Cfg.POS_FILTER_ENABLE),
    "PosStaticSpeedEps": Cfg.STATIC_SPEED_EPS,
    "PosStaticFreezeRadius": Cfg.STATIC_FREEZE_RADIUS,
    "PosStaticAlpha": Cfg.STATIC_ALPHA,
    "PosMovingAlpha": Cfg.MOVING_ALPHA,
    "PositionValid": 0, "PositionRejected": 0, "PositionDegraded": 1,
    "PositionReacquiring": 0, "PositionUpdateSeq": 0,
    "PositionRejectCount": 0, "PositionRejectStreak": 0,
    "PositionInnovation": 0.0, "PositionGateLimit": Cfg.POSITION_GATE_ABS_MAX,
    "PositionGateEnable": int(Cfg.POSITION_GATE_ENABLE),
    "PositionGateQualities": ",".join(str(v) for v in Cfg.POSITION_GATE_ALLOWED_QUALITIES),
    "PositionGateMargin": Cfg.POSITION_GATE_MARGIN,
    "PositionGateSpeedScale": Cfg.POSITION_GATE_SPEED_SCALE,
    "PositionGateAbsMax": Cfg.POSITION_GATE_ABS_MAX,
    "PositionReacquireCount": Cfg.POSITION_REACQUIRE_COUNT,
    "PositionReacquireStep": Cfg.POSITION_REACQUIRE_STEP,
    "GGA_quality": 0, "GGA_sats": 0, "GGA_hdop": 99.9,
    "Head": 0.0, "HeadRaw": 0.0, "Head_E": 0.0,
    "Roll": 0.0, "Pitch": 0.0,
    "Speed": 0.0, "V_x": 0.0, "V_y": 0.0,
    "Ve": 0.0, "Vn": 0.0, "Vu": 0.0,
    "Omega": 0.0, "A_n": 0.0,
    "Status": 0, "sys_status": 0,
    "slope": 0.0, "slope_avg": 0.0, "grade": 0.0,
}


def wrap180(angle_deg):
    return wrap_deg(angle_deg)


def record_position_analysis():
    """Record raw and filtered O-XYZ positions for static noise analysis."""
    global _analysis_t0
    if not Cfg.POS_ANALYSIS_ENABLE:
        return
    now = time.monotonic()
    if _analysis_t0 is None:
        _analysis_t0 = now
    if len(_analysis_samples) >= Cfg.POS_ANALYSIS_MAX_SAMPLES:
        return
    _analysis_samples.append({
        "t": now - _analysis_t0,
        "raw_x": current_state["Raw_x"],
        "raw_y": current_state["Raw_y"],
        "filt_x": current_state["UTM_x"],
        "filt_y": current_state["UTM_y"],
        "speed": current_state["Speed"],
        "pos_jump": current_state["PosJump"],
        "pos_filtered": current_state["PosFiltered"],
        "gga_quality": current_state["GGA_quality"],
        "gga_sats": current_state["GGA_sats"],
        "gga_hdop": current_state["GGA_hdop"],
    })


def _position_stats(x, y):
    import numpy as np

    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    dx = x - np.mean(x)
    dy = y - np.mean(y)
    r = np.hypot(dx, dy)
    cov = np.cov(np.vstack([x, y])) if len(x) > 1 else np.zeros((2, 2))
    return {
        "mean_x": float(np.mean(x)),
        "mean_y": float(np.mean(y)),
        "var_x": float(np.var(x, ddof=1)) if len(x) > 1 else 0.0,
        "var_y": float(np.var(y, ddof=1)) if len(y) > 1 else 0.0,
        "std_x": float(np.std(x, ddof=1)) if len(x) > 1 else 0.0,
        "std_y": float(np.std(y, ddof=1)) if len(y) > 1 else 0.0,
        "cov_xy": float(cov[0, 1]) if cov.shape == (2, 2) else 0.0,
        "r_rms": float(np.sqrt(np.mean(r * r))),
        "r_p95": float(np.percentile(r, 95)),
        "r_max": float(np.max(r)),
    }


def save_position_analysis():
    """Save CSV, statistics, and distribution figures for raw/filter comparison."""
    if not Cfg.POS_ANALYSIS_ENABLE:
        return
    if len(_analysis_samples) < 5:
        print("\n[QX-ANALYSIS] 样本不足(<5)，未生成滤波对比图。")
        return

    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("\n[QX-ANALYSIS] 缺少 numpy/matplotlib，无法出图: {}".format(exc))
        return

    os.makedirs(Cfg.POS_ANALYSIS_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(Cfg.POS_ANALYSIS_DIR, "pos_filter_static_{}".format(ts))
    csv_path = base + ".csv"
    txt_path = base + "_summary.txt"
    png_path = base + ".png"

    fields = ["t", "raw_x", "raw_y", "filt_x", "filt_y", "speed",
              "pos_jump", "pos_filtered", "gga_quality", "gga_sats", "gga_hdop"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(_analysis_samples)

    arr = {k: np.array([row[k] for row in _analysis_samples], dtype=float) for k in fields}
    static_mask = arr["speed"] <= Cfg.POS_ANALYSIS_STATIC_SPEED_EPS
    used_mask = static_mask if int(np.sum(static_mask)) >= 5 else np.ones_like(static_mask, dtype=bool)
    mode = "static_only" if int(np.sum(static_mask)) >= 5 else "all_samples_fallback"

    raw_x = arr["raw_x"][used_mask]
    raw_y = arr["raw_y"][used_mask]
    filt_x = arr["filt_x"][used_mask]
    filt_y = arr["filt_y"][used_mask]
    t = arr["t"][used_mask]

    raw_stats = _position_stats(raw_x, raw_y)
    filt_stats = _position_stats(filt_x, filt_y)

    def improvement(key):
        before = raw_stats[key]
        after = filt_stats[key]
        if abs(before) < 1e-12:
            return 0.0
        return 100.0 * (before - after) / before

    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("Qianxun POS static filter analysis\n")
        f.write("samples_total: {}\n".format(len(_analysis_samples)))
        f.write("samples_used: {}\n".format(len(raw_x)))
        f.write("mode: {}\n".format(mode))
        f.write("static_speed_eps_mps: {:.4f}\n".format(Cfg.POS_ANALYSIS_STATIC_SPEED_EPS))
        f.write("pos_filter_enable: {}\n".format(int(Cfg.POS_FILTER_ENABLE)))
        f.write("static_freeze_radius_m: {:.4f}\n".format(Cfg.STATIC_FREEZE_RADIUS))
        f.write("static_alpha: {:.4f}\n".format(Cfg.STATIC_ALPHA))
        f.write("moving_alpha: {:.4f}\n".format(Cfg.MOVING_ALPHA))
        f.write("\n[raw]\n")
        for k, v in raw_stats.items():
            f.write("{}: {:.8f}\n".format(k, v))
        f.write("\n[filtered]\n")
        for k, v in filt_stats.items():
            f.write("{}: {:.8f}\n".format(k, v))
        f.write("\n[improvement_percent]\n")
        for k in ["var_x", "var_y", "std_x", "std_y", "r_rms", "r_p95", "r_max"]:
            f.write("{}: {:.3f}\n".format(k, improvement(k)))

    raw_dx = raw_x - np.mean(raw_x)
    raw_dy = raw_y - np.mean(raw_y)
    filt_dx = filt_x - np.mean(filt_x)
    filt_dy = filt_y - np.mean(filt_y)
    raw_r = np.hypot(raw_dx, raw_dy)
    filt_r = np.hypot(filt_dx, filt_dy)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    ax = axes[0, 0]
    ax.scatter(raw_dx, raw_dy, s=10, alpha=0.55, label="raw")
    ax.scatter(filt_dx, filt_dy, s=10, alpha=0.55, label="filtered")
    ax.set_title("XY distribution around mean")
    ax.set_xlabel("dx [m]")
    ax.set_ylabel("dy [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(t, raw_dx, label="raw x-mean", lw=1.2)
    ax.plot(t, raw_dy, label="raw y-mean", lw=1.2)
    ax.plot(t, filt_dx, label="filtered x-mean", lw=1.2)
    ax.plot(t, filt_dy, label="filtered y-mean", lw=1.2)
    ax.set_title("Static drift time series")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("offset [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    bins = max(10, min(60, int(np.sqrt(len(raw_r)))))
    ax.hist(raw_r, bins=bins, alpha=0.55, label="raw radial")
    ax.hist(filt_r, bins=bins, alpha=0.55, label="filtered radial")
    ax.set_title("Radial error distribution")
    ax.set_xlabel("radius from mean [m]")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1, 1]
    labels = ["std_x", "std_y", "r_rms", "r_p95", "r_max"]
    raw_vals = [raw_stats[k] for k in labels]
    filt_vals = [filt_stats[k] for k in labels]
    x_idx = np.arange(len(labels))
    width = 0.38
    ax.bar(x_idx - width / 2, raw_vals, width, label="raw")
    ax.bar(x_idx + width / 2, filt_vals, width, label="filtered")
    ax.set_xticks(x_idx)
    ax.set_xticklabels(labels, rotation=20)
    ax.set_ylabel("meter")
    ax.set_title("Variance/distribution metrics")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()

    fig.suptitle("Qianxun static POS filter analysis | {} | used {}/{} samples".format(
        mode, len(raw_x), len(_analysis_samples)))
    fig.savefig(png_path, dpi=180)
    plt.close(fig)

    print("\n[QX-ANALYSIS] 静止滤波对比已保存:")
    print("  CSV : {}".format(csv_path))
    print("  TXT : {}".format(txt_path))
    print("  PNG : {}".format(png_path))


def load_start_reference():
    """Read the first reference point from the qianxun map file.

    The path planner writes map points as:
        0.0,0.0,head,x,y
    POS only needs the first point to guide the vehicle to the planned start.
    """
    if not Cfg.GUIDE_START_ENABLE:
        return None
    path = Cfg.GUIDE_MAP_FILE
    if not os.path.exists(path):
        print("[QX-START] 未找到参考地图: {}".format(path))
        print("           请先运行 path_plan_MPC_qianxun.py 生成轨迹，或设置 QX_GUIDE_MAP_FILE。")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as exc:
        print("[QX-START] 读取参考地图失败: {}".format(exc))
        return None

    tokens = content.replace("\n", "\t").split("\t")
    for token in tokens:
        parts = [p.strip() for p in token.split(",")]
        if len(parts) < 5:
            continue
        try:
            head = wrap180(float(parts[2]))
            x = float(parts[3])
            y = float(parts[4])
        except ValueError:
            continue
        return {"x": x, "y": y, "head": head, "source": path}
    print("[QX-START] 地图中未解析到参考起点: {}".format(path))
    return None


def start_guide_status():
    """Return compact guidance text and arrival state for terminal output."""
    global _start_arrived_reported
    if _start_ref is None:
        return "", False
    dx = _start_ref["x"] - current_state["UTM_x"]
    dy = _start_ref["y"] - current_state["UTM_y"]
    dist = math.hypot(dx, dy)
    head_err = wrap180(current_state["Head"] - _start_ref["head"])
    pos_ok = dist <= Cfg.GUIDE_POS_TOL
    head_ok = abs(head_err) <= Cfg.GUIDE_HEAD_TOL_DEG
    arrived = pos_ok and (head_ok or not Cfg.GUIDE_REQUIRE_HEADING)
    text = (" startΔ:({dx:+.2f},{dy:+.2f}) d:{dist:.2f}m "
            "Href:{href:+.1f} Herr:{herr:+.1f}°").format(
                dx=dx, dy=dy, dist=dist, href=_start_ref["head"], herr=head_err)
    if arrived and not _start_arrived_reported:
        _start_arrived_reported = True
        print("\n[QX-START] 已到起点处: d={:.2f} m, 航向误差={:+.1f} deg".format(dist, head_err))
        if not Cfg.GUIDE_REQUIRE_HEADING:
            print("           注: 当前默认按距离判定到点；纯卫星静止航向不可靠，请低速移动时对齐 Href。")
    elif not arrived:
        _start_arrived_reported = False
    return text, arrived


def parse_optional_float(text):
    if text is None:
        return None
    text = str(text).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def seed_initial_heading():
    """Seed static heading before GNSS COG becomes reliable.

    Pure GNSS heading comes from course over ground, so a stationary vehicle has
    no physical heading observation. For field tests, park the vehicle along the
    planned start direction and seed Head from the map first point. Override it
    with QX_INIT_HEAD_OXYZ_DEG if the vehicle is deliberately parked at another
    known O-XYZ heading.
    """
    global _last_head_enu, _heading_seeded
    if _heading_seeded:
        return

    source = Cfg.INIT_HEADING_SOURCE
    manual_head = parse_optional_float(Cfg.INIT_HEAD_OXYZ_DEG)
    head_oxyz = None
    source_text = ""

    if source in ("0", "off", "none", "disable", "disabled"):
        return
    if manual_head is not None and source in ("auto", "manual", "env"):
        head_oxyz = manual_head
        source_text = "QX_INIT_HEAD_OXYZ_DEG"
    elif _start_ref is not None and source in ("auto", "map", "ref", "start"):
        head_oxyz = _start_ref["head"]
        source_text = "map start Head"
    elif manual_head is not None:
        head_oxyz = manual_head
        source_text = "QX_INIT_HEAD_OXYZ_DEG"

    if head_oxyz is None:
        return

    head_oxyz = wrap180(head_oxyz)
    raw_head_oxyz = wrap180(head_oxyz - Cfg.HEADING_OFFSET_DEG)
    head_enu = oxyz_heading_to_enu(raw_head_oxyz, Cfg.YAW_ENU_TO_OXYZ_DEG)
    _last_head_enu = head_enu
    current_state["Head"] = head_oxyz
    current_state["HeadRaw"] = raw_head_oxyz
    current_state["Head_E"] = head_enu
    _heading_seeded = True
    print("\n[QX-HEAD] 初始航向种子: HeadO={:+.2f} deg, RawO={:+.2f} deg, HeadE={:+.2f} deg | 来源: {}".format(
        head_oxyz, current_state["HeadRaw"], head_enu, source_text))
    print("          静止/低速时先沿用该航向；速度 >= {:.2f} m/s 后由 RMC 航迹角接管。".format(
        Cfg.COG_MIN_SPEED))


def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    c = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    xe = (n + h) * c * math.cos(lon)
    ye = (n + h) * c * math.sin(lon)
    ze = (n * (1.0 - WGS84_E2) + h) * s
    return xe, ye, ze


def set_origin(lat, lon, h):
    global _origin_ready, _origin_from_first_fix, _o_lat, _o_lon, _o_h, _o_xe, _o_ye, _o_ze
    if (Cfg.ORIGIN_LAT is not None and Cfg.ORIGIN_LON is not None
            and not Cfg.ZERO_FIRST_FIX):
        _o_lat = Cfg.ORIGIN_LAT
        _o_lon = Cfg.ORIGIN_LON
        _o_h = Cfg.ORIGIN_ALT if Cfg.ORIGIN_ALT is not None else h
        _origin_from_first_fix = False
        print("\n[QX O-XYZ] 使用固定原点: LAT={:.10f} LON={:.10f} ALT={:.3f}".format(
            _o_lat, _o_lon, _o_h))
    else:
        _o_lat, _o_lon, _o_h = lat, lon, h
        _origin_from_first_fix = True
        print("\n[QX O-XYZ] 自动锁定原点: LAT={:.10f} LON={:.10f} ALT={:.3f}".format(
            _o_lat, _o_lon, _o_h))
        print("           下次复用同一张地图时，把这些值填入 Cfg.ORIGIN_*。若只想本次起点为0,0，可保持 ORIGIN_* 为 None。")
    _o_xe, _o_ye, _o_ze = geodetic_to_ecef(_o_lat, _o_lon, _o_h)
    current_state["OriginLat"] = _o_lat
    current_state["OriginLon"] = _o_lon
    current_state["OriginAlt"] = _o_h
    current_state["OriginMode"] = (
        "first_fix" if _origin_from_first_fix else "fixed")
    current_state["YawEnuToOxyzDeg"] = Cfg.YAW_ENU_TO_OXYZ_DEG
    _origin_ready = True


def latlon_to_oxyz(lat, lon, h):
    global _origin_reported
    if not _origin_ready:
        set_origin(lat, lon, h)
    xe, ye, ze = geodetic_to_ecef(lat, lon, h)
    dx, dy, dz = xe - _o_xe, ye - _o_ye, ze - _o_ze
    s_lon = math.sin(math.radians(_o_lon))
    c_lon = math.cos(math.radians(_o_lon))
    s_lat = math.sin(math.radians(_o_lat))
    c_lat = math.cos(math.radians(_o_lat))

    east = -s_lon * dx + c_lon * dy
    north = -s_lat * c_lon * dx - s_lat * s_lon * dy + c_lat * dz

    psi = math.radians(Cfg.YAW_ENU_TO_OXYZ_DEG)
    x = east * math.cos(psi) + north * math.sin(psi)
    y = -east * math.sin(psi) + north * math.cos(psi)
    if not _origin_reported:
        dist = math.hypot(east, north)
        print("[QX O-XYZ] 当前首帧: LAT={:.10f} LON={:.10f}".format(lat, lon))
        print("[QX O-XYZ] 首帧相对原点 ENU: E={:+.3f} m N={:+.3f} m |d|={:.3f} m".format(
            east, north, dist))
        print("[QX O-XYZ] 旋转到 O-XYZ: yaw={:+.3f} deg -> X={:+.3f} m Y={:+.3f} m".format(
            Cfg.YAW_ENU_TO_OXYZ_DEG, x, y))
        if not _origin_from_first_fix and dist > Cfg.ORIGIN_WARN_METERS:
            print("[QX O-XYZ] 注意: 固定原点与当前停车点相差 {:.2f} m，起点不会是(0,0)。".format(dist))
            print("           若本次实车想从当前位置作为原点启动，请把 Cfg.ORIGIN_LAT/LON 设为 None，")
            print("           或用 QX_ZERO_FIRST_FIX=1 python3 qianxun/POS_qianxun.py。\n")
        _origin_reported = True
    return x, y, 0.0


def nmea_checksum_ok(line):
    text = line.strip()
    if "*" not in text:
        return not Cfg.CHECKSUM_STRICT
    body, checksum = text[1:].split("*", 1) if text.startswith("$") else text.split("*", 1)
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(checksum[:2], 16)
    except ValueError:
        return False


def dm_to_deg(value, deg_len):
    if value == "":
        return None
    deg = float(value[:deg_len])
    minute = float(value[deg_len:])
    return deg + minute / 60.0


def parse_rmc(line):
    """Parse $GPRMC/$GNRMC and return dict or None."""
    text = line.strip()
    if not text.startswith("$") or "RMC" not in text:
        return None
    diag["rmc_seen"] += 1
    if not nmea_checksum_ok(text):
        diag["checksum_bad"] += 1
        return None

    main = text.split("*", 1)[0]
    parts = main.split(",")
    if len(parts) < 10 or not parts[0].endswith("RMC"):
        diag["parse_bad"] += 1
        return None

    valid = parts[2] == "A"
    lat = dm_to_deg(parts[3], 2)
    lon = dm_to_deg(parts[5], 3)
    if lat is None or lon is None:
        diag["parse_bad"] += 1
        return None
    if parts[4] == "S":
        lat = -lat
    if parts[6] == "W":
        lon = -lon

    speed_mps = float(parts[7]) * 0.514444 if parts[7] else 0.0
    cog_deg = float(parts[8]) if parts[8] else None
    mode = parts[12] if len(parts) > 12 and parts[12] else ""
    diag["rmc_ok"] += 1
    return {
        "lat": lat,
        "lon": lon,
        "speed_mps": speed_mps,
        "cog_deg": cog_deg,
        "valid": valid,
        "mode": mode,
        "utc": parts[1],
        "date": parts[9],
    }


def parse_gga(line):
    """Parse $GPGGA/$GNGGA for fix quality diagnostics."""
    text = line.strip()
    if not text.startswith("$") or "GGA" not in text:
        return None
    diag["gga_seen"] += 1
    if not nmea_checksum_ok(text):
        diag["checksum_bad"] += 1
        return None

    main = text.split("*", 1)[0]
    parts = main.split(",")
    if len(parts) < 10 or not parts[0].endswith("GGA"):
        diag["parse_bad"] += 1
        return None
    try:
        quality = int(parts[6]) if parts[6] else 0
        sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 99.9
        alt = float(parts[9]) if parts[9] else 0.0
    except ValueError:
        diag["parse_bad"] += 1
        return None

    diag["gga_ok"] += 1
    return {"quality": quality, "sats": sats, "hdop": hdop, "alt": alt}


def status_from_rmc(valid, mode):
    if not valid:
        return 0
    if mode == "R":
        return 4      # RTK fixed if the receiver uses NMEA 4.x mode letters.
    if mode == "F":
        return 5      # RTK float.
    if mode == "D":
        return 2      # Differential GNSS.
    return 1          # Autonomous/other valid GNSS.


def status_from_gga(quality):
    if quality == 4:
        return 4      # RTK fixed in standard GGA.
    if quality == 5:
        return 5      # RTK float.
    if quality == 2:
        return 2      # DGPS.
    if quality == 1:
        return 1      # Autonomous GNSS.
    return 0


def gate_oxyz(raw_x, raw_y, speed, now):
    """Reject a non-physical GNSS position update before low-pass filtering.

    Returns the accepted/held measurement and diagnostics. PositionUpdateSeq
    advances only for a normally accepted GNSS frame, so the planner cannot
    advance its path index repeatedly on one held coordinate.
    """
    global _last_raw_x, _last_raw_y
    global _position_gate_ready, _position_gate_x, _position_gate_y
    global _position_gate_t, _position_gate_speed, _position_update_seq
    global _position_reject_total, _position_reject_streak
    global _position_candidate_x, _position_candidate_y, _position_candidate_streak
    global _pos_filter_ready

    if _last_raw_x is None:
        raw_jump = 0.0
    else:
        raw_jump = math.hypot(raw_x - _last_raw_x, raw_y - _last_raw_y)
    _last_raw_x, _last_raw_y = raw_x, raw_y

    quality = int(_last_gga.get("quality", 0))
    quality_allowed = quality in Cfg.POSITION_GATE_ALLOWED_QUALITIES

    if not Cfg.POSITION_GATE_ENABLE:
        _position_gate_ready = True
        _position_gate_x, _position_gate_y = raw_x, raw_y
        _position_gate_t, _position_gate_speed = now, speed
        _position_update_seq += 1
        return raw_x, raw_y, {
            "raw_jump": raw_jump, "valid": 1, "rejected": 0,
            "degraded": int(quality != 4), "reacquiring": 0,
            "innovation": 0.0, "limit": Cfg.POSITION_GATE_ABS_MAX,
            "seq": _position_update_seq,
        }

    if not _position_gate_ready:
        # 启动阶段也不能让q=2/浮点解建立控制原点。先在内部连续观察
        # 若干帧q=4，确认稳定后再一次性建立首个有效CP。
        if quality_allowed:
            if (
                _position_candidate_x is not None
                and math.hypot(
                    raw_x - _position_candidate_x,
                    raw_y - _position_candidate_y,
                ) <= Cfg.POSITION_REACQUIRE_RADIUS
            ):
                _position_candidate_streak += 1
            else:
                _position_candidate_streak = 1
            _position_candidate_x, _position_candidate_y = raw_x, raw_y
        else:
            _position_candidate_x = _position_candidate_y = None
            _position_candidate_streak = 0

        if _position_candidate_streak >= max(Cfg.POSITION_REACQUIRE_COUNT, 1):
            _position_gate_ready = True
            _position_gate_x = _position_candidate_x
            _position_gate_y = _position_candidate_y
            _position_gate_t, _position_gate_speed = now, speed
            _position_update_seq += 1
            _position_reject_streak = 0
            _position_candidate_x = _position_candidate_y = None
            _position_candidate_streak = 0
            # 先前无效占位坐标不得污染低通滤波器初值。
            _pos_filter_ready = False
            return _position_gate_x, _position_gate_y, {
                "raw_jump": raw_jump, "valid": 1, "rejected": 0,
                "degraded": 0, "reacquiring": 0,
                "innovation": 0.0, "limit": Cfg.POSITION_GATE_ABS_MAX,
                "seq": _position_update_seq,
            }

        _position_reject_total += 1
        _position_reject_streak += 1
        return _position_gate_x, _position_gate_y, {
            "raw_jump": raw_jump, "valid": 0, "rejected": 1,
            "degraded": 1, "reacquiring": 0,
            "innovation": 0.0, "limit": Cfg.POSITION_GATE_ABS_MAX,
            "seq": _position_update_seq,
        }

    dt = now - (_position_gate_t if _position_gate_t is not None else now)
    dt = max(Cfg.POSITION_GATE_MIN_DT, min(dt, Cfg.POSITION_GATE_MAX_DT))
    speed_bound = max(abs(speed), abs(_position_gate_speed), Cfg.STATIC_SPEED_EPS)
    gate_limit = min(
        Cfg.POSITION_GATE_ABS_MAX,
        Cfg.POSITION_GATE_MARGIN + Cfg.POSITION_GATE_SPEED_SCALE * speed_bound * dt,
    )
    innovation = math.hypot(
        raw_x - _position_gate_x, raw_y - _position_gate_y
    )
    accepted = bool(
        quality_allowed
        and innovation <= gate_limit
        and innovation <= Cfg.POSITION_DIRECT_ACCEPT_MAX
        and _position_reject_streak == 0
    )

    if accepted:
        _position_gate_x, _position_gate_y = raw_x, raw_y
        _position_gate_t, _position_gate_speed = now, speed
        _position_update_seq += 1
        _position_reject_streak = 0
        _position_candidate_x = _position_candidate_y = None
        _position_candidate_streak = 0
        return _position_gate_x, _position_gate_y, {
            "raw_jump": raw_jump, "valid": 1, "rejected": 0,
            "degraded": int(quality != 4), "reacquiring": 0,
            "innovation": innovation, "limit": gate_limit,
            "seq": _position_update_seq,
        }

    _position_reject_total += 1
    _position_reject_streak += 1
    reacquiring = 0
    if quality_allowed:
        if (
            _position_candidate_x is not None
            and math.hypot(
                raw_x - _position_candidate_x, raw_y - _position_candidate_y
            ) <= Cfg.POSITION_REACQUIRE_RADIUS
        ):
            _position_candidate_streak += 1
            _position_candidate_x, _position_candidate_y = raw_x, raw_y
        else:
            _position_candidate_x, _position_candidate_y = raw_x, raw_y
            _position_candidate_streak = 1

        if _position_candidate_streak >= max(Cfg.POSITION_REACQUIRE_COUNT, 1):
            dx = _position_candidate_x - _position_gate_x
            dy = _position_candidate_y - _position_gate_y
            distance = math.hypot(dx, dy)
            if distance > 1e-9:
                step = min(Cfg.POSITION_REACQUIRE_STEP, distance)
                _position_gate_x += step * dx / distance
                _position_gate_y += step * dy / distance
                reacquiring = 1
                # 对外CP、有效标志和序号在同一分支内原子更新。
                _position_gate_t, _position_gate_speed = now, speed
                _position_update_seq += 1
                _position_reject_streak = 0
                return _position_gate_x, _position_gate_y, {
                    "raw_jump": raw_jump, "valid": 1, "rejected": 0,
                    "degraded": int(quality != 4), "reacquiring": 1,
                    "innovation": innovation, "limit": gate_limit,
                    "seq": _position_update_seq,
                }
            # 候选已经与保持点重合，也按一次有效恢复处理。
            _position_gate_t, _position_gate_speed = now, speed
            _position_update_seq += 1
            _position_reject_streak = 0
            _position_candidate_x = _position_candidate_y = None
            _position_candidate_streak = 0
            return _position_gate_x, _position_gate_y, {
                "raw_jump": raw_jump, "valid": 1, "rejected": 0,
                "degraded": int(quality != 4), "reacquiring": 0,
                "innovation": innovation, "limit": gate_limit,
                "seq": _position_update_seq,
            }
    else:
        _position_candidate_x = _position_candidate_y = None
        _position_candidate_streak = 0

    # 无效定位期间严格保持最后一次有效CP；候选量绝不写入发布状态。
    return _position_gate_x, _position_gate_y, {
        "raw_jump": raw_jump, "valid": 0, "rejected": 1,
        "degraded": 1, "reacquiring": reacquiring,
        "innovation": innovation, "limit": gate_limit,
        "seq": _position_update_seq,
    }


def filter_oxyz(raw_x, raw_y, speed):
    """Return filtered O-XYZ position and diagnostics.

    The filter is intentionally conservative while stopped: GNSS static drift is
    not vehicle motion, and feeding it to MPC creates false lateral error.
    """
    global _pos_filter_ready, _filt_x, _filt_y

    if (not Cfg.POS_FILTER_ENABLE) or (not _pos_filter_ready):
        _filt_x, _filt_y = raw_x, raw_y
        _pos_filter_ready = True
        return _filt_x, _filt_y, 0

    err = math.hypot(raw_x - _filt_x, raw_y - _filt_y)
    stopped = abs(speed) < Cfg.STATIC_SPEED_EPS

    if stopped and err <= Cfg.STATIC_FREEZE_RADIUS:
        alpha = 0.0
    elif stopped:
        alpha = Cfg.STATIC_ALPHA
    else:
        alpha = Cfg.MOVING_ALPHA

    _filt_x += alpha * (raw_x - _filt_x)
    _filt_y += alpha * (raw_y - _filt_y)
    return _filt_x, _filt_y, 1


def update_state(rmc):
    global _last_head_enu, _last_speed, _last_t, _omega_lpf, _acc_lpf

    now = time.time()
    lat, lon = rmc["lat"], rmc["lon"]
    speed = rmc["speed_mps"]
    alt = _last_gga["alt"] if _last_gga["quality"] > 0 else 0.0
    raw_x, raw_y, z = latlon_to_oxyz(lat, lon, alt)
    gated_x, gated_y, position_diag = gate_oxyz(
        raw_x, raw_y, speed, now
    )
    x, y, pos_filtered = filter_oxyz(gated_x, gated_y, speed)

    if rmc["cog_deg"] is not None and speed >= Cfg.COG_MIN_SPEED:
        # GPRMC COG: +North origin, clockwise positive. Internal heading:
        # +Y/+North origin, counterclockwise positive.
        head_enu = nmea_cog_to_enu_heading(rmc["cog_deg"])
    else:
        if _last_head_enu is None:
            seed_initial_heading()
        head_enu = _last_head_enu if _last_head_enu is not None else current_state["Head_E"]

    head_raw_oxyz = enu_heading_to_oxyz(head_enu, Cfg.YAW_ENU_TO_OXYZ_DEG)
    head_oxyz = wrap180(head_raw_oxyz + Cfg.HEADING_OFFSET_DEG)

    if _last_t is not None:
        dt = max(now - _last_t, 1e-3)
        if _last_head_enu is not None and speed >= Cfg.COG_MIN_SPEED:
            omega_raw = math.radians(wrap180(head_enu - _last_head_enu)) / dt
            omega_raw = max(-Cfg.OMEGA_LIMIT, min(Cfg.OMEGA_LIMIT, omega_raw))
        else:
            omega_raw = 0.0
        acc_raw = (speed - (_last_speed if _last_speed is not None else speed)) / dt
        _omega_lpf = (1.0 - Cfg.OMEGA_ALPHA) * _omega_lpf + Cfg.OMEGA_ALPHA * omega_raw
        _acc_lpf = (1.0 - Cfg.ACC_ALPHA) * _acc_lpf + Cfg.ACC_ALPHA * acc_raw

    _last_head_enu = head_enu
    _last_speed = speed
    _last_t = now

    theta = math.radians(head_enu)
    ve, vn = enu_velocity(speed, theta)
    status = max(status_from_rmc(rmc["valid"], rmc["mode"]),
                 status_from_gga(_last_gga["quality"]))

    current_state.update({
        "Lat": lat, "Lon": lon, "Alt": alt,
        "UTM_x": x, "UTM_y": y, "Z": z,
        "Raw_x": raw_x, "Raw_y": raw_y,
        "PosJump": position_diag["raw_jump"],
        "PosFiltered": pos_filtered,
        "PositionValid": position_diag["valid"],
        "PositionRejected": position_diag["rejected"],
        "PositionDegraded": position_diag["degraded"],
        "PositionReacquiring": position_diag["reacquiring"],
        "PositionUpdateSeq": position_diag["seq"],
        "PositionRejectCount": _position_reject_total,
        "PositionRejectStreak": _position_reject_streak,
        "PositionInnovation": position_diag["innovation"],
        "PositionGateLimit": position_diag["limit"],
        "GGA_quality": _last_gga["quality"],
        "GGA_sats": _last_gga["sats"],
        "GGA_hdop": _last_gga["hdop"],
        "Head": head_oxyz, "HeadRaw": head_raw_oxyz, "Head_E": head_enu,
        "Roll": 0.0, "Pitch": 0.0,
        "Speed": speed, "V_x": speed, "V_y": 0.0,
        "Ve": ve, "Vn": vn, "Vu": 0.0,
        "Omega": _omega_lpf, "A_n": _acc_lpf,
        "Status": status, "sys_status": 1 if status > 0 else 0,
        "slope": 0.0, "slope_avg": 0.0, "grade": 0.0,
    })
    record_position_analysis()


def publish_data(pub, pub_ros_location, pub_ros_pose, pub_ros_sensor):
    pub.sendPro(b"CurGNSS", current_state)

    location_msg = std_msgs.msg.String()
    location_msg.data = json.dumps(current_state)
    pub_ros_location.publish(location_msg)

    pose_msg = geometry_msgs.msg.Pose2D()
    pose_msg.x = current_state["UTM_x"]
    pose_msg.y = current_state["UTM_y"]
    pose_msg.theta = math.radians(current_state["Head"])
    pub_ros_pose.publish(pose_msg)

    sensor_msg = std_msgs.msg.String()
    sensor_msg.data = json.dumps({
        "Roll": current_state["Roll"],
        "Pitch": current_state["Pitch"],
        "Alt": current_state["Alt"],
    })
    pub_ros_sensor.publish(sensor_msg)

    global _last_print
    now = time.time()
    if now - _last_print >= 1.0 / Cfg.PRINT_HZ:
        _last_print = now
        raw_dx = current_state["Raw_x"] - current_state["UTM_x"]
        raw_dy = current_state["Raw_y"] - current_state["UTM_y"]
        guide_text, _ = start_guide_status()
        sys.stdout.write(
            "\r[QX-RMC] X:{x:7.2f} Y:{y:7.2f} HeadO:{ho:+6.1f} RawO:{hr:+6.1f} HeadE:{he:+6.1f} "
            "V:{v:4.2f} w:{w:+5.2f} raw-f:({rdx:+.2f},{rdy:+.2f}) "
            "q:{q} sat:{sat:02d} hdop:{hdop:3.1f} fix:{fix} "
            "P:{pv}/{seq} J:{jump:.2f}/{gate:.2f} R:{rej} "
            "Lat:{lat:.10f} Lon:{lon:.10f}{guide}   ".format(
                x=current_state["UTM_x"], y=current_state["UTM_y"],
                ho=current_state["Head"], hr=current_state["HeadRaw"], he=current_state["Head_E"],
                v=current_state["Speed"], w=current_state["Omega"],
                rdx=raw_dx, rdy=raw_dy,
                q=current_state["GGA_quality"], sat=current_state["GGA_sats"],
                hdop=current_state["GGA_hdop"],
                pv=current_state["PositionValid"],
                seq=current_state["PositionUpdateSeq"],
                jump=current_state["PositionInnovation"],
                gate=current_state["PositionGateLimit"],
                rej=current_state["PositionRejectCount"],
                lat=current_state["Lat"], lon=current_state["Lon"],
                fix=current_state["Status"], guide=guide_text))
        sys.stdout.flush()


def print_diag():
    global _last_diag
    if not Cfg.DEBUG_DIAG:
        return
    now = time.time()
    if now - _last_diag < Cfg.DIAG_PERIOD:
        return
    _last_diag = now
    sys.stdout.write(
        "\n[QX-DIAG] bytes={bytes} lines={lines} rmc_seen={rmc_seen} "
        "rmc_ok={rmc_ok} gga_seen={gga_seen} gga_ok={gga_ok} "
        "checksum_bad={checksum_bad} parse_bad={parse_bad}\n"
        "          last_line={last}\n".format(
            last=diag["last_line"][:160],
            **diag))
    sys.stdout.flush()


def send_qianxun_config(ser, reason="startup"):
    """Enable Qianxun RMC output on the receiver's internal com2 port."""
    global _last_config_t
    if not Cfg.AUTO_CONFIG:
        return
    commands = [cmd.strip() for cmd in Cfg.CONFIG_COMMANDS.split(";") if cmd.strip()]
    if not commands:
        return
    _last_config_t = time.time()
    for cmd in commands:
        payload = (cmd + Cfg.CONFIG_EOL).encode("ascii", "ignore")
        try:
            ser.write(payload)
            ser.flush()
            print("[QX-CFG] {} -> {}".format(reason, cmd))
            time.sleep(0.15)
        except Exception as exc:
            print("[QX-CFG] 发送配置失败 {}: {}".format(cmd, exc))


def main():
    global _last_gga, _start_ref
    rospy.init_node("POS_qianxun", anonymous=True)
    pub_ros_pose = rospy.Publisher("/bus/pose", geometry_msgs.msg.Pose2D, queue_size=10)
    pub_ros_location = rospy.Publisher("/fusion_location", std_msgs.msg.String, queue_size=10)
    pub_ros_sensor = rospy.Publisher("/bus/sensor", std_msgs.msg.String, queue_size=10)

    ctx = proContext()
    pub = ctx.socket(zmq.PUB)
    pub.bind(Cfg.GnssAddr)

    print("===== 千寻纯卫星 RMC POS 桥接启动(O-XYZ, 姿态/海拔置零) =====")
    print("串口: {} @ {} | RMC低速航迹角门限: {:.2f} m/s | 航向逻辑偏置: {:+.1f} deg".format(
        Cfg.QX_PORT, Cfg.QX_BAUD, Cfg.COG_MIN_SPEED, Cfg.HEADING_OFFSET_DEG))
    print("初始航向: source={} manual={!r} | 静止时RMC航迹角不可作为车头角".format(
        Cfg.INIT_HEADING_SOURCE, Cfg.INIT_HEAD_OXYZ_DEG))
    if abs(wrap180(Cfg.HEADING_OFFSET_DEG)) > 1e-6:
        print("[QX-HEAD] 注意: 已启用非零航向偏置；纯 GPRMC 尾部向前循迹通常应保持 0 deg。")
    print("调试: QX_DEBUG_RAW={} QX_CHECKSUM_STRICT={} QX_DEBUG_DIAG={} DTR={} RTS={} AUTO_CONFIG={}".format(
        int(Cfg.DEBUG_RAW), int(Cfg.CHECKSUM_STRICT), int(Cfg.DEBUG_DIAG), int(Cfg.DTR), int(Cfg.RTS), int(Cfg.AUTO_CONFIG)))
    if Cfg.AUTO_CONFIG:
        print("千寻输出配置: {} | EOL={!r} | retry={:.1f}s".format(
            Cfg.CONFIG_COMMANDS, Cfg.CONFIG_EOL, Cfg.CONFIG_RETRY_SEC))
    print("航向约定: +Y=0 deg, 逆时针为正；+X=-90 deg，-X=+90 deg。")
    print("标定: YAW_ENU_TO_OXYZ_DEG 表示 O-XYZ 坐标轴相对 ENU 的逆时针旋转角。")
    print("位置门控: enable={} q={} margin={:.2f}m scale={:.2f} abs_max={:.2f}m "
          "reacquire={}x/{:.2f}m".format(
              int(Cfg.POSITION_GATE_ENABLE),
              ",".join(str(v) for v in Cfg.POSITION_GATE_ALLOWED_QUALITIES),
              Cfg.POSITION_GATE_MARGIN, Cfg.POSITION_GATE_SPEED_SCALE,
              Cfg.POSITION_GATE_ABS_MAX, Cfg.POSITION_REACQUIRE_COUNT,
              Cfg.POSITION_REACQUIRE_STEP))
    _start_ref = load_start_reference()
    if _start_ref is not None:
        print("[QX-START] 参考起点: X={:+.3f} m Y={:+.3f} m Head={:+.2f} deg".format(
            _start_ref["x"], _start_ref["y"], _start_ref["head"]))
        print("           判定阈值: 距离≤{:.2f} m{} | 来源: {}".format(
            Cfg.GUIDE_POS_TOL,
            " 且航向≤{:.1f} deg".format(Cfg.GUIDE_HEAD_TOL_DEG) if Cfg.GUIDE_REQUIRE_HEADING else "",
            _start_ref["source"]))

    # 20 Hz时每帧相隔约50 ms。短超时配合in_waiting按现有字节读取，
    # 避免read(512)等待填满后把多帧RMC/GGA成批交给融合线程。
    ser = serial.Serial(Cfg.QX_PORT, Cfg.QX_BAUD, timeout=0.02)
    ser.dtr = Cfg.DTR
    ser.rts = Cfg.RTS
    ser.reset_input_buffer()
    time.sleep(0.2)
    send_qianxun_config(ser, "startup")
    print("[QX-RMC] 串口已打开，等待 NMEA RMC 数据...")
    buf = b""
    try:
        while not rospy.is_shutdown():
            try:
                waiting = getattr(ser, "in_waiting", 0)
                chunk = ser.read(max(1, min(waiting, 512)))
                if chunk:
                    diag["bytes"] += len(chunk)
                    buf += chunk.replace(b"\r", b"\n")
                elif not buf:
                    if (Cfg.AUTO_CONFIG and diag["rmc_seen"] == 0 and
                            time.time() - _last_config_t >= Cfg.CONFIG_RETRY_SEC):
                        send_qianxun_config(ser, "retry-no-rmc")
                    print_diag()
                    continue

                if len(buf) > 8192:
                    diag["last_line"] = "<buffer too long; no line break or wrong baud/protocol>"
                    buf = b""
                    print_diag()

                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("ascii", "ignore").strip()
                    if not line:
                        continue
                    diag["lines"] += 1
                    diag["last_line"] = line
                    if Cfg.DEBUG_RAW:
                        print("[QX-RAW] {}".format(line))
                    gga = parse_gga(line)
                    if gga is not None:
                        _last_gga.update(gga)
                        continue
                    rmc = parse_rmc(line)
                    if rmc is None:
                        continue
                    if Cfg.REQUIRE_VALID_RMC and not rmc["valid"]:
                        continue
                    update_state(rmc)
                    publish_data(pub, pub_ros_location, pub_ros_pose, pub_ros_sensor)
                print_diag()
            except KeyboardInterrupt:
                break
            except Exception as exc:
                print("\n[QX-RMC] 串口/解析异常: {}".format(exc))
                time.sleep(0.5)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        save_position_analysis()


if __name__ == "__main__":
    main()
