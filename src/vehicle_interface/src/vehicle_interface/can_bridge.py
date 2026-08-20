#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ZMQ to SocketCAN bridge for the Qianxun MPC test chain.

Data flow:
    control_MPC_qianxun.py
        -> ZMQ PoliAcc  (tcp://127.0.0.1:8095, linear speed v_cmd, m/s)
        -> ZMQ PoliSteer(tcp://127.0.0.1:8092, yaw rate w_cmd, rad/s)
        -> ZMQ PoliBrake(tcp://127.0.0.1:8096)

    this script
        -> CAN ID 0x200: int16 speed(mm/s with sign), int16 yaw_rate(mrad/s)
        <- CAN ID 0x203: int16 raw left/right motor feedback
        -> ZMQ MotorFB (tcp://*:8099, n_l/n_r as signed motor rpm)
        -> ROS /motor_feedback (raw int16 values and signed rpm as JSON)
        -> ROS /joint_states (signed sprocket angular velocity)

This is the Qianxun-local version of scripts_motion_engine/zmq_can_MPC.py.
ROS imports remain inside main() so protocol helpers stay unit-testable without
a running ROS master.
"""

import argparse
import json
import math
import signal
import socket
import struct
import sys
import threading
import time

zmq = None


class Cfg:
    can_iface = "can0"
    policy_acc = "tcp://127.0.0.1:8095"
    policy_steer = "tcp://127.0.0.1:8092"
    policy_brake = "tcp://127.0.0.1:8096"
    motor_fb_bind = "tcp://*:8099"

    can_id_cmd = 0x200
    can_id_feedback = 0x203
    send_period = 0.05  # 20 Hz，与MPC控制周期一致
    stale_timeout = 0.5

    # The controllers always publish the standard logical vehicle command:
    # v>0 forward, omega>0 counterclockwise. The white vehicle is physically
    # operated tail-first, so only linear speed is inverted at this boundary.
    # The original VCU yaw field is already counterclockwise-positive.
    speed_sign = -1.0
    steer_sign = 1.0
    speed_scale = 1000.0      # m/s -> mm/s int16
    steer_scale = 1000.0      # rad/s -> mrad/s int16
    max_speed_cmd = 2.0       # m/s safety clamp for command packing
    max_steer_cmd = 2.0       # rad/s safety clamp for command packing

    # VCU feedback is published as signed motor rpm.  The controller owns the
    # only rpm -> sprocket rad/s -> track m/s conversion, avoiding duplicate
    # 2*pi/60 conversion across the ZMQ and control layers.
    feedback_sign = -1.0
    gear_ratio = 1.0

    # Tracked chassis can pivot in place. Keep this false for the real vehicle;
    # the optional CLI switch exists only for a chassis that forbids pivoting.
    zero_steer_when_stopped = False
    stop_speed_eps = 0.03
    print_hz = 5.0


is_exit = False
cmd_lock = threading.Lock()
REQUIRED_POLICY_TOPICS = ("PoliAcc", "PoliSteer", "PoliBrake")
cmd_state = {
    "v_cmd": 0.0,
    "w_cmd": 0.0,
    # Start in the fail-safe state. Motion is released only after every
    # required policy channel has delivered at least one fresh frame.
    "brake": 1.0,
    "seen_topics": set(),
    "topic_times": {topic: 0.0 for topic in REQUIRED_POLICY_TOPICS},
}


def clamp(value, limit):
    return max(-limit, min(limit, value))


def parse_topic_value(message):
    """Parse either 'Topic json' strings or multipart [topic, json]."""
    if isinstance(message, (list, tuple)):
        if len(message) < 2:
            raise ValueError("multipart message has fewer than 2 frames")
        payload = message[1]
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8", "ignore")
        return float(json.loads(payload))

    if isinstance(message, bytes):
        message = message.decode("utf-8", "ignore")
    _, payload = str(message).split(" ", 1)
    return float(json.loads(payload))


def recv_policy(ctx, addr, topic, state_key):
    sub = ctx.socket(zmq.SUB)
    sub.connect(addr)
    sub.setsockopt_string(zmq.SUBSCRIBE, topic)
    sub.setsockopt(zmq.RCVTIMEO, 200)
    print(f"[ZMQ] SUB {topic} <- {addr}")
    while not is_exit:
        try:
            try:
                msg = sub.recv_string()
            except UnicodeDecodeError:
                msg = sub.recv_multipart()
            value = parse_topic_value(msg)
            with cmd_lock:
                first_topic_frame = topic not in cmd_state["seen_topics"]
                cmd_state[state_key] = value
                cmd_state["seen_topics"].add(topic)
                cmd_state["topic_times"][topic] = time.monotonic()
            if first_topic_frame:
                print("\n[ZMQ] 已收到首帧 {}={:+.3f}".format(topic, value))
        except zmq.Again:
            continue
        except Exception as exc:
            print(f"[ZMQ] recv {topic} error: {exc}")
            time.sleep(0.05)


def pack_can_frame(can_id, payload8):
    return struct.pack("<IB3x8s", can_id, 8, payload8)


def unpack_can_frame(frame):
    can_id, dlc, data = struct.unpack("<IB3x8s", frame)
    return can_id & 0x1FFFFFFF, dlc, data


def open_can_socket(can_iface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((can_iface,))
    sock.setblocking(False)
    return sock


def current_command(cfg):
    """Return a command only when all three policy channels are fresh.

    A shared timestamp is unsafe here: one live channel could otherwise keep
    an old speed, yaw-rate or brake value alive indefinitely.  The returned
    diagnostics identify each missing/stale channel for field troubleshooting.
    """
    now = time.monotonic()
    with cmd_lock:
        v = float(cmd_state["v_cmd"])
        w = float(cmd_state["w_cmd"])
        brk = float(cmd_state["brake"])
        seen_topics = set(cmd_state["seen_topics"])
        topic_times = dict(cmd_state["topic_times"])

    missing_topics = [
        topic for topic in REQUIRED_POLICY_TOPICS if topic not in seen_topics
    ]
    topic_ages = {
        topic: (
            max(0.0, now - float(topic_times.get(topic, 0.0)))
            if topic in seen_topics else float("inf")
        )
        for topic in REQUIRED_POLICY_TOPICS
    }
    stale_topics = [
        topic for topic in REQUIRED_POLICY_TOPICS
        if topic in seen_topics and topic_ages[topic] > cfg.stale_timeout
    ]
    channels_healthy = not missing_topics and not stale_topics
    max_age = max(topic_ages.values())

    if not channels_healthy or brk > 0.5:
        v, w = 0.0, 0.0
    if not channels_healthy:
        brk = 1.0
    if cfg.zero_steer_when_stopped and abs(v) < cfg.stop_speed_eps:
        w = 0.0

    v = clamp(v, cfg.max_speed_cmd)
    w = clamp(w, cfg.max_steer_cmd)
    diagnostics = {
        "healthy": channels_healthy,
        "missing_topics": missing_topics,
        "stale_topics": stale_topics,
        "topic_ages": topic_ages,
    }
    return v, w, brk, max_age, diagnostics


def pack_motion_payload(v_cmd, w_cmd, cfg):
    """Pack the documented VCU 0x200 motion payload.

    Bytes 0..3 contain only signed speed and yaw-rate int16 values; bytes 4..7
    are reserved zeros.  There is no verified physical-brake field in the
    inherited protocol, so PoliBrake deliberately remains a fail-safe request
    that forces both motion fields to zero.
    """
    desire_speed = int(cfg.speed_sign * v_cmd * cfg.speed_scale)
    desire_steer = int(cfg.steer_sign * w_cmd * cfg.steer_scale)
    desire_speed = int(clamp(desire_speed, 32767))
    desire_steer = int(clamp(desire_steer, 32767))
    return struct.pack("<hh4x", desire_speed, desire_steer), \
        desire_speed, desire_steer


def raw_motor_to_rpm(raw_value, cfg):
    """Return signed VCU motor speed in rpm without unit conversion."""
    return cfg.feedback_sign * float(raw_value) / cfg.gear_ratio


def can_loop(ctx, cfg, dry_run=False, ros_feedback=None):
    pub_motor = ctx.socket(zmq.PUB)
    pub_motor.bind(cfg.motor_fb_bind)
    print(f"[ZMQ] PUB MotorFB -> {cfg.motor_fb_bind}")

    sock = None
    if not dry_run:
        sock = open_can_socket(cfg.can_iface)
        print(f"[CAN] opened {cfg.can_iface}, cmd_id=0x{cfg.can_id_cmd:X}, fb_id=0x{cfg.can_id_feedback:X}")
    else:
        print("[CAN] dry-run mode, not opening SocketCAN")
    print(
        "[CAN-SAFETY] PoliBrake is not a verified physical-brake CAN field; "
        "it only forces 0x200 speed/yaw to zero. Keep the physical E-stop "
        "ready and verify the VCU protocol before relying on active braking."
    )

    last_send = 0.0
    last_print = 0.0
    while not is_exit:
        now = time.time()

        if now - last_send >= cfg.send_period:
            v_cmd, w_cmd, brk, age, command_diag = current_command(cfg)
            data, desire_speed, desire_steer = pack_motion_payload(
                v_cmd, w_cmd, cfg
            )
            frame = pack_can_frame(cfg.can_id_cmd, data)

            if not dry_run:
                try:
                    sock.send(frame)
                except Exception as exc:
                    print(f"[CAN] send error: {exc}")

            if now - last_print >= 1.0 / max(cfg.print_hz, 0.1):
                last_print = now
                age_text = "never" if not math.isfinite(age) or age > 1e8 else "{:.2f}s".format(age)
                if command_diag["missing_topics"]:
                    channel_status = "missing:" + ",".join(
                        command_diag["missing_topics"]
                    )
                elif command_diag["stale_topics"]:
                    channel_status = "stale:" + ",".join(
                        command_diag["stale_topics"]
                    )
                else:
                    channel_status = "all-fresh"
                print(
                    "\r[QX-CAN] v={:+.3f}m/s w={:+.3f}rad/s -> "
                    "CAN(speed={}, steer={}) age={} channels={} brk={:.1f}".format(
                        v_cmd, w_cmd, desire_speed, desire_steer, age_text,
                        channel_status, brk),
                    end="",
                    flush=True,
                )
            last_send = now

        if not dry_run:
            try:
                frame_recv = sock.recv(16)
                can_id, dlc, data = unpack_can_frame(frame_recv)
                if can_id == cfg.can_id_feedback and dlc >= 4:
                    raw_l, raw_r = struct.unpack("<hh", data[:4])
                    n_l = raw_motor_to_rpm(raw_l, cfg)
                    n_r = raw_motor_to_rpm(raw_r, cfg)
                    pub_motor.send_string(f"MotorFB {json.dumps({'n_l': n_l, 'n_r': n_r})}")
                    if ros_feedback is not None:
                        rospy, string_type, joint_state_type, feedback_pub, joint_pub = ros_feedback
                        stamp = rospy.Time.now()
                        feedback_pub.publish(string_type(data=json.dumps({
                            "stamp_s": stamp.to_sec(),
                            "can_id": int(can_id),
                            "raw_left": int(raw_l),
                            "raw_right": int(raw_r),
                            "left_rpm": float(n_l),
                            "right_rpm": float(n_r),
                        }, sort_keys=True)))
                        joints = joint_state_type()
                        joints.header.stamp = stamp
                        joints.name = ["left_track_joint", "right_track_joint"]
                        joints.velocity = [
                            float(n_l) * 2.0 * math.pi / 60.0,
                            float(n_r) * 2.0 * math.pi / 60.0,
                        ]
                        joint_pub.publish(joints)
            except BlockingIOError:
                pass
            except Exception as exc:
                print(f"\n[CAN] recv error: {exc}")

        time.sleep(0.001)


def on_signal(signum, _frame):
    global is_exit
    is_exit = True
    print(f"\n[System] signal {signum}, exiting...")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Qianxun ZMQ-CAN bridge for the VCU chassis.")
    p.add_argument("--can", default=Cfg.can_iface, help="SocketCAN interface, default can0.")
    p.add_argument("--dry-run", action="store_true", help="Do not open/send CAN; print packed command only.")
    p.add_argument("--speed-sign", type=float, default=Cfg.speed_sign, help="Speed sign before CAN packing.")
    p.add_argument("--steer-sign", type=float, default=Cfg.steer_sign, help="Yaw-rate sign before CAN packing.")
    p.add_argument("--feedback-sign", type=float, default=Cfg.feedback_sign, help="Motor feedback sign.")
    p.add_argument("--gear-ratio", type=float, default=Cfg.gear_ratio, help="Feedback rpm divisor; keep 1.0 when control_MPC_qianxun.py owns drivetrain ratio.")
    p.add_argument("--zero-steer-when-stopped", action="store_true", help="Force yaw-rate command to zero when speed is almost zero.")
    # roslaunch appends remapping arguments such as ``__name:=...``.  They are
    # unrelated to this SocketCAN bridge, so accept and ignore unknown values.
    return p.parse_known_args(argv)[0]


def main():
    global zmq
    args = parse_args()
    try:
        import zmq as _zmq
        zmq = _zmq
    except ImportError as exc:
        print("缺少 pyzmq 运行依赖: {}。请在 Ubuntu 工程环境中安装/加载 pyzmq。".format(exc), file=sys.stderr)
        return 1

    try:
        import rospy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rospy.init_node("zmq_can_bridge", disable_signals=True)
        feedback_pub = rospy.Publisher(
            "/motor_feedback", String, queue_size=100)
        joint_pub = rospy.Publisher(
            "/joint_states", JointState, queue_size=100)
        ros_feedback = (
            rospy, String, JointState, feedback_pub, joint_pub)
    except ImportError as exc:
        print("缺少ROS Python运行依赖，无法发布电机反馈: {}".format(exc),
              file=sys.stderr)
        return 1

    cfg = Cfg()
    cfg.can_iface = args.can
    cfg.speed_sign = args.speed_sign
    cfg.steer_sign = args.steer_sign
    cfg.feedback_sign = args.feedback_sign
    cfg.gear_ratio = max(1e-6, abs(args.gear_ratio))
    cfg.zero_steer_when_stopped = args.zero_steer_when_stopped

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    ctx = zmq.Context()
    threads = [
        threading.Thread(target=recv_policy, args=(ctx, cfg.policy_acc, "PoliAcc", "v_cmd"), daemon=True),
        threading.Thread(target=recv_policy, args=(ctx, cfg.policy_steer, "PoliSteer", "w_cmd"), daemon=True),
        threading.Thread(target=recv_policy, args=(ctx, cfg.policy_brake, "PoliBrake", "brake"), daemon=True),
        threading.Thread(target=can_loop,
                         args=(ctx, cfg, args.dry_run, ros_feedback),
                         daemon=True),
    ]

    for t in threads:
        t.start()

    print("[System] Qianxun ZMQ-CAN bridge running.")
    while not is_exit and not rospy.is_shutdown():
        time.sleep(0.1)

    ctx.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
