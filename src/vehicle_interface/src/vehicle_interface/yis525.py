# -*- coding: utf-8 -*-
#!/usr/bin/env python3
"""YIS525串口采集与ROS IMU发布节点。

本脚本参考资料目录中的Yesense Python V2.1厂家例程编写，并改成自包含
实现，使车载Ubuntu运行时不需要从中文资料路径导入厂家代码。

本节点只负责解析并发布IMU传感器坐标系下的原始测量值。坐标轴符号、
安装偏角、磁北/地图对准、滤波和GNSS杆臂补偿均由
POS_qianxun_imu.py完成。

输出：
    唯一发布ROS ``/imu/data``。千寻融合与LIO-SAM订阅同一条消息，因而
    使用完全相同的测量值和header时间戳。

环境变量：
    YIS_PORT=/dev/ttyUSB1
    YIS_BAUD=460800
    YIS_ROS_TOPIC=/imu/data
    YIS_FRAME_ID=imu_link
    YIS_PRINT_HZ=5
    YIS_DEBUG_RAW=0

默认IMU端口为ttyUSB1，因为千寻通常占用ttyUSB0。实车应优先使用稳定的
/dev/serial/by-id/...或/dev/serial/by-path/...设备路径。
"""

import math
import os
import struct
import sys
import time


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, ".."))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

try:
    import serial
    import rosgraph
    import rospy
    from sensor_msgs.msg import Imu
except ImportError as exc:
    print("缺少运行依赖: {}。需要ROS与pyserial。".format(exc))
    sys.exit(1)


class Cfg:
    # 【运行前必须确认】IMU串口路径和波特率；不要与千寻使用同一个设备节点。
    PORT = os.environ.get("YIS_PORT", "/dev/ttyUSB1")
    BAUD = int(os.environ.get("YIS_BAUD", "460800"))
    ROS_TOPIC = os.environ.get("YIS_ROS_TOPIC", "/imu/data")
    FRAME_ID = os.environ.get("YIS_FRAME_ID", "imu_link")
    PRINT_HZ = float(os.environ.get("YIS_PRINT_HZ", "5"))
    DEBUG_RAW = os.environ.get("YIS_DEBUG_RAW", "0").lower() in ("1", "true", "yes")
    DTR = os.environ.get("YIS_DTR", "0").lower() in ("1", "true", "yes")
    RTS = os.environ.get("YIS_RTS", "0").lower() in ("1", "true", "yes")
    READ_SIZE = int(os.environ.get("YIS_READ_SIZE", "4096"))
    MAX_BUFFER = int(os.environ.get("YIS_MAX_BUFFER", "65536"))


HEADER = b"\x59\x53"
MIN_FRAME_LEN = 7  # 帧头2字节 + tid 2字节 + 数据长度1字节 + CK1/CK2 2字节


def _new_sensor_state():
    return {
        "tid": 0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "q0": 1.0,
        "q1": 0.0,
        "q2": 0.0,
        "q3": 0.0,
        "sensor_temp": 0.0,
        "acc_x": 0.0,
        "acc_y": 0.0,
        "acc_z": 0.0,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "gyro_z": 0.0,
        "norm_mag_x": 0.0,
        "norm_mag_y": 0.0,
        "norm_mag_z": 0.0,
        "raw_mag_x": 0.0,
        "raw_mag_y": 0.0,
        "raw_mag_z": 0.0,
        "smp_timestamp": 0,
        "ready_timestamp": 0,
        "device_status": 0,
    }


def calc_checksum(data):
    """计算Yesense CK1/CK2校验和，并按小端uint16形式返回。"""
    check_a = 0
    check_b = 0
    for value in data:
        check_a = (check_a + value) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return (check_b << 8) | check_a


class Yis525Decoder:
    """Yesense标准串口协议的流式解码器。"""

    def __init__(self):
        self.buffer = bytearray()
        self.state = _new_sensor_state()
        self.frames_ok = 0
        self.checksum_bad = 0
        self.bytes_discarded = 0
        self.last_tid = None
        self.tid_dropped = 0
        self.tid_duplicate = 0

    def feed(self, chunk):
        if chunk:
            self.buffer.extend(chunk)
        frames = []

        while True:
            header_pos = self.buffer.find(HEADER)
            if header_pos < 0:
                if len(self.buffer) > 1:
                    self.bytes_discarded += len(self.buffer) - 1
                    del self.buffer[:-1]
                break
            if header_pos > 0:
                self.bytes_discarded += header_pos
                del self.buffer[:header_pos]

            if len(self.buffer) < MIN_FRAME_LEN:
                break

            payload_len = self.buffer[4]
            frame_len = MIN_FRAME_LEN + payload_len
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            received = struct.unpack_from("<H", frame, 5 + payload_len)[0]
            calculated = calc_checksum(frame[2:5 + payload_len])
            if received != calculated:
                self.checksum_bad += 1
                self.bytes_discarded += 1
                del self.buffer[0]
                continue

            del self.buffer[:frame_len]
            parsed = self._parse_frame(frame, payload_len)
            self.frames_ok += 1
            frames.append(parsed)

        if len(self.buffer) > Cfg.MAX_BUFFER:
            self.bytes_discarded += len(self.buffer)
            self.buffer.clear()
        return frames

    def _parse_frame(self, frame, payload_len):
        tid = struct.unpack_from("<H", frame, 2)[0]
        self._update_tid_health(tid)
        self.state["tid"] = tid

        present = set()
        pos = 5
        payload_end = 5 + payload_len
        while pos + 2 <= payload_end:
            data_id = frame[pos]
            data_len = frame[pos + 1]
            data_start = pos + 2
            data_end = data_start + data_len
            if data_end > payload_end:
                break
            self._parse_tlv(data_id, frame[data_start:data_end])
            present.add(data_id)
            pos = data_end

        out = dict(self.state)
        out.update({
            "HasAcc": int(0x10 in present),
            "HasGyro": int(0x20 in present),
            "HasEuler": int(0x40 in present),
            "HasQuaternion": int(0x41 in present),
            "HasMag": int(0x30 in present or 0x31 in present),
            "HasSampleTimestamp": int(0x51 in present),
            "FrameValid": 1,
        })
        return out

    def _update_tid_health(self, tid):
        if self.last_tid is not None:
            delta = (tid - self.last_tid) & 0xFFFF
            if delta == 0:
                self.tid_duplicate += 1
            elif delta > 1 and delta < 0x8000:
                self.tid_dropped += delta - 1
        self.last_tid = tid

    def _parse_tlv(self, data_id, payload):
        try:
            if data_id == 0x01 and len(payload) == 2:
                self.state["sensor_temp"] = struct.unpack("<h", payload)[0] * 0.01
            elif data_id == 0x10 and len(payload) == 12:
                values = struct.unpack("<iii", payload)
                self.state["acc_x"], self.state["acc_y"], self.state["acc_z"] = [v * 1e-6 for v in values]
            elif data_id == 0x20 and len(payload) == 12:
                values = struct.unpack("<iii", payload)
                self.state["gyro_x"], self.state["gyro_y"], self.state["gyro_z"] = [v * 1e-6 for v in values]
            elif data_id == 0x30 and len(payload) == 12:
                values = struct.unpack("<iii", payload)
                self.state["norm_mag_x"], self.state["norm_mag_y"], self.state["norm_mag_z"] = [v * 1e-6 for v in values]
            elif data_id == 0x31 and len(payload) == 12:
                values = struct.unpack("<iii", payload)
                self.state["raw_mag_x"], self.state["raw_mag_y"], self.state["raw_mag_z"] = [v * 1e-3 for v in values]
            elif data_id == 0x40 and len(payload) == 12:
                pitch, roll, yaw = struct.unpack("<iii", payload)
                self.state["pitch"] = pitch * 1e-6
                self.state["roll"] = roll * 1e-6
                self.state["yaw"] = yaw * 1e-6
            elif data_id == 0x41 and len(payload) == 16:
                values = struct.unpack("<iiii", payload)
                self.state["q0"], self.state["q1"], self.state["q2"], self.state["q3"] = [v * 1e-6 for v in values]
            elif data_id == 0x51 and len(payload) == 4:
                self.state["smp_timestamp"] = struct.unpack("<I", payload)[0]
            elif data_id == 0x52 and len(payload) == 4:
                self.state["ready_timestamp"] = struct.unpack("<I", payload)[0]
            elif data_id == 0x80 and len(payload) == 1:
                self.state["device_status"] = payload[0]
        except struct.error:
            # 外层帧校验正确但TLV长度异常时忽略该数据项；通信健康计数仍保留，
            # 便于现场判断串口数据流是否连续。
            pass


def _finite_required(data):
    names = ("yaw", "roll", "pitch", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z")
    return all(math.isfinite(float(data.get(name, float("nan")))) for name in names)


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _publish_imu(pub, data, stamp):
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = Cfg.FRAME_ID
    roll = math.radians(float(data["roll"]))
    pitch = math.radians(float(data["pitch"]))
    yaw = math.radians(float(data["yaw"]))
    qx, qy, qz, qw = _quaternion_from_rpy(roll, pitch, yaw)
    msg.orientation.x, msg.orientation.y = qx, qy
    msg.orientation.z, msg.orientation.w = qz, qw
    msg.angular_velocity.x = math.radians(float(data["gyro_x"]))
    msg.angular_velocity.y = math.radians(float(data["gyro_y"]))
    msg.angular_velocity.z = math.radians(float(data["gyro_z"]))
    msg.linear_acceleration.x = float(data["acc_x"])
    msg.linear_acceleration.y = float(data["acc_y"])
    msg.linear_acceleration.z = float(data["acc_z"])
    pub.publish(msg)


def _check_sole_publisher(_event):
    try:
        publishers, _subscribers, _services = rosgraph.Master(
            rospy.get_name()).getSystemState()
        topic = rospy.resolve_name(Cfg.ROS_TOPIC)
        owners = sorted(set(
            node for published_topic, nodes in publishers
            if rospy.resolve_name(published_topic) == topic for node in nodes
        ))
    except Exception as exc:
        rospy.logerr("无法检查IMU发布者: %s", exc)
        return
    if owners != [rospy.get_name()]:
        rospy.logfatal("%s必须仅由%s发布，当前发布者=%s",
                       topic, rospy.get_name(), owners)
        rospy.signal_shutdown("conflicting IMU publishers")


def main():
    rospy.init_node("yis525")
    print("===== YIS525 IMU采集节点 =====")
    print("串口: {} @ {} 8N1 | 唯一发布: {} frame={}".format(
        Cfg.PORT, Cfg.BAUD, Cfg.ROS_TOPIC, Cfg.FRAME_ID))
    print("说明: 本节点发布传感器原始轴定义；安装偏角和项目坐标转换在POS_qianxun_imu.py完成。")

    pub = rospy.Publisher(Cfg.ROS_TOPIC, Imu, queue_size=200)
    ownership_timer = rospy.Timer(
        rospy.Duration(2.0), _check_sole_publisher, oneshot=True)

    ser = serial.Serial(
        Cfg.PORT,
        Cfg.BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        # 50 Hz报文每20 ms到达；短超时防止定长读取造成多帧批量发布。
        timeout=0.01,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    ser.dtr = Cfg.DTR
    ser.rts = Cfg.RTS
    ser.reset_input_buffer()

    decoder = Yis525Decoder()
    last_print = 0.0
    bytes_rx = 0
    rate_window_start = time.monotonic()
    rate_window_frames = 0
    observed_hz = 0.0

    try:
        while not rospy.is_shutdown():
            waiting = getattr(ser, "in_waiting", 0)
            chunk = ser.read(max(1, min(waiting, Cfg.READ_SIZE)))
            bytes_rx += len(chunk)
            if Cfg.DEBUG_RAW and chunk:
                print("[YIS-RAW]", chunk.hex(" "))

            for data in decoder.feed(chunk):
                now_wall = time.time()
                now_mono = time.monotonic()
                rate_window_frames += 1
                rate_elapsed = now_mono - rate_window_start
                if rate_elapsed >= 1.0:
                    observed_hz = rate_window_frames / rate_elapsed
                    rate_window_start = now_mono
                    rate_window_frames = 0
                data.update({
                    "Timestamp": now_wall,
                    "Monotonic": now_mono,
                    "Port": Cfg.PORT,
                    "Baud": Cfg.BAUD,
                    "DataValid": int(_finite_required(data)),
                    "FramesOK": decoder.frames_ok,
                    "ChecksumBad": decoder.checksum_bad,
                    "TidDropped": decoder.tid_dropped,
                    "TidDuplicate": decoder.tid_duplicate,
                    "BytesRx": bytes_rx,
                    "ObservedHz": observed_hz,
                })
                if data["DataValid"]:
                    # One stamp is created here and carried unchanged to both
                    # Qianxun fusion and LIO-SAM.
                    _publish_imu(pub, data, rospy.Time.now())

                if now_mono - last_print >= 1.0 / max(Cfg.PRINT_HZ, 0.1):
                    last_print = now_mono
                    print(
                        "\r[YIS525] tid:{tid:5d} yaw:{yaw:+8.3f} pitch:{pitch:+7.3f} roll:{roll:+7.3f} "
                        "gyro:({gx:+8.4f},{gy:+8.4f},{gz:+8.4f})deg/s "
                        "acc:({ax:+6.3f},{ay:+6.3f},{az:+6.3f})m/s2 "
                        "Hz:{hz:5.1f} ok:{ok} crc:{crc} drop:{drop}   ".format(
                            tid=int(data["tid"]), yaw=data["yaw"], pitch=data["pitch"], roll=data["roll"],
                            gx=data["gyro_x"], gy=data["gyro_y"], gz=data["gyro_z"],
                            ax=data["acc_x"], ay=data["acc_y"], az=data["acc_z"],
                            hz=observed_hz,
                            ok=decoder.frames_ok, crc=decoder.checksum_bad, drop=decoder.tid_dropped),
                        end="",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print("\n[YIS525] 用户停止。")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        del ownership_timer


if __name__ == "__main__":
    main()
