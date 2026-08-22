# -*- coding: utf-8 -*-
#!/usr/bin/env python3
"""千寻GNSS + YIS525组合定位节点，用于平地和斜坡轨迹跟踪。

设计目标
========
在第一阶段不开发完整INS/ESKF的前提下，恢复POS_MPC_xiepo.py向规划器和
控制器提供的主要状态接口：

* 千寻提供GNSS天线经纬度、地速、地面航迹角Course和定位状态；
* YIS525提供车体航向、横滚角、俯仰角、三轴角速度和诊断信息；
* GNSS与IMU独立读取，使用主机单调时钟把IMU插值到GNSS帧时刻；
* 再用GNSS地速和高频IMU航向将控制点短时预测到当前发布时刻，以50 Hz
  发布“当前状态估计”，而不是重复发布旧GNSS坐标；
* 根据可配置的三维车体系杆臂，将GNSS天线位置和速度刚性转换到车辆控制点；
* 当前版本不积分IMU加速度，避免MEMS零偏和履带振动产生虚假速度与位置。

坐标约定
========
本约定与frame_convention.py及所有千寻控制器一致：

* O-XYZ航向角从全局+Y轴开始计量；
* 航向角和横摆角速度均为逆时针正、顺时针负；
* 逻辑车体系+x向前、+y向左、+z向上；
* 杆臂定义为r_body = p_control - p_antenna，并在逻辑车体系表达；
* 车辆用车尾在前跟踪时，逻辑+x必须指向车尾。

YIS525原始yaw方向和实际安装角不能靠猜测确定。所有标定参数集中在
FusionCfg中，可由环境变量覆盖。自动驾驶前必须用手推方式核对符号。

数据流
======
    YIS525 serial -> /imu/data ---------+
                                         +--> 组合定位节点 --> CurGNSS/tcp:8080
    Qianxun RMC/GGA serial --------------+

LIO-SAM订阅完全相同的/imu/data消息；本节点不再使用第二条ZMQ IMU链路。

现有轨迹规划器和MPC/LQR/STSMC控制器继续订阅CurGNSS，无需修改接口。
"""

import copy
import json
import math
import os
import sys
import threading
import time
from collections import deque


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(BASE_DIR, ".."))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from .frame_convention import (  # noqa: E402
    enu_heading_to_oxyz,
    heading_from_delta,
    nmea_cog_to_enu_heading,
    tangent,
    left_normal,
    wrap_deg,
)
from .contracts import (  # noqa: E402
    resolve_gnss_measurement_timestamp,
    rmc_measurement_epoch,
    time_alignment_is_valid,
)

try:
    import serial
    import zmq
    import rospy
    import geometry_msgs.msg
    import sensor_msgs.msg
    import std_msgs.msg
    from .pro_context import proContext
    from . import qianxun_position as qx
except ImportError as exc:
    print("缺少运行依赖: {}。需要ROS、pyserial、pyzmq以及同目录POS_qianxun.py。".format(exc))
    sys.exit(1)


def _env_float(name, default):
    return float(os.environ.get(name, str(default)))


def _env_heading_mode():
    """读取人工确认的YIS525航向模式；不能根据磁场字段自动推断。"""
    aliases = {
        "AHRS": "AHRS",
        "MAG": "AHRS",
        "MAGNETIC": "AHRS",
        "VRU": "VRU",
        "REL": "VRU",
        "RELATIVE": "VRU",
        "REFERENCE_FREE": "VRU",
        "UNKNOWN": "UNKNOWN",
        "": "UNKNOWN",
    }
    raw = os.environ.get("IMU_HEADING_MODE", "AHRS").strip().upper()
    if raw not in aliases:
        raise ValueError(
            "IMU_HEADING_MODE={!r}无效，只能填写、VRU或UNKNOWN".format(raw)
        )
    return aliases[raw]


def _env_bool(name, default):
    value = os.environ.get(name, "1" if default else "0").strip().lower()
    return value in ("1", "true", "yes", "on")


def _rpy_from_quaternion(x, y, z, w):
    """Return ROS roll/pitch/yaw without creating a second IMU transform path."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


# ============================================================================
# 【控制前还必须检查POS_qianxun.py中的基础地图配置】
# 1. QX_PORT/QX_BAUD：千寻独立串口及波特率，不能与IMU使用同一设备节点；
# 2. ORIGIN_LAT/LON/ALT：必须与规划地图使用的经纬高原点一致；
# 3. YAW_ENU_TO_OXYZ_DEG：ENU到项目O-XYZ地图的轴旋转角；
# 4. GUIDE_MAP_FILE：必须指向本次实际运行的guihua_qianxun.map；
# 5. 重复运行同一地图时不能随意改用新的第一帧GNSS位置作为原点。
# ============================================================================
class FusionCfg:
    # ======================================================================
    # 【运行前必须确认】通信地址和数据超时
    # 千寻RMC/GGA目标20 Hz；GNSS超时阈值仍需容忍短时串口和解算抖动。
    # 千寻融合和LIO-SAM必须订阅同一个ROS IMU话题。
    # 千寻串口QX_PORT/QX_BAUD在POS_qianxun.py中设置。
    # ======================================================================
    IMU_TOPIC = os.environ.get("YIS_ROS_TOPIC", "/imu/data")
    PUBLISH_HZ = _env_float("QX_IMU_PUBLISH_HZ", 50.0) #IMU和千寻GNSS融合发布频率
    IMU_TIMEOUT = _env_float("QX_IMU_TIMEOUT", 0.30)
    # 本项目计划把RMC/GGA配置为20 Hz；0.60 s仍允许短时串口抖动，
    # 但不会像旧2.5 s阈值那样长期使用陈旧位置。
    GNSS_TIMEOUT = _env_float("QX_GNSS_TIMEOUT", 0.60)
    PRINT_HZ = _env_float("QX_IMU_PRINT_HZ", 5.0)

    # ======================================================================
    # 【软件时间对齐与当前状态预测】
    # 没有PPS硬同步时，GNSS和YIS525使用同一台Ubuntu主机的monotonic接收时间
    # 建立统一时间轴。IMU以50 Hz缓存，插值到RMC到达时刻；随后把GNSS控制点
    # 沿最新IMU航向短时传播到50 Hz发布时刻。
    #
    # GNSS_FIXED_LATENCY_S用于以后通过日志辨识并补偿MR02内部解算/串口固定延迟；
    # 未辨识前保持0，绝不能凭感觉填大值。PREDICT_MAX_AGE限制失联外推距离。
    # ======================================================================
    IMU_EXPECTED_HZ = _env_float("QX_IMU_EXPECTED_HZ", 50.0)
    IMU_HISTORY_SECONDS = _env_float("QX_IMU_HISTORY_SECONDS", 1.0)
    # 50 Hz IMU的正常最近邻误差应远小于一个GNSS周期。超过60 ms说明
    # IMU缓存断流、调度严重阻塞或两传感器时间轴不再可信，融合必须失效。
    IMU_ALIGNMENT_MAX_ERROR_S = max(
        0.001, _env_float("QX_IMU_ALIGNMENT_MAX_ERROR_S", 0.060)
    )
    STATE_PREDICT_ENABLE = _env_bool("QX_STATE_PREDICT_ENABLE", True)
    GNSS_FIXED_LATENCY_S = _env_float("QX_GNSS_FIXED_LATENCY_S", 0.0)
    # RMC UTC与主机接收时钟扣除固定延迟后的残差门限。失配时对外时间戳
    # 回退到主机接收时间，同时FusionValid=0，避免绕过状态新鲜度watchdog。
    GNSS_CLOCK_MAX_RESIDUAL_S = max(
        0.01, _env_float("QX_GNSS_CLOCK_MAX_RESIDUAL_S", 0.30)
    )
    REQUIRE_GNSS_CLOCK_CONSISTENCY = _env_bool(
        "QX_REQUIRE_GNSS_CLOCK_CONSISTENCY", True
    )
    PREDICT_MAX_AGE = _env_float("QX_STATE_PREDICT_MAX_AGE", 0.30)
    PREDICT_MAX_SPEED = _env_float("QX_STATE_PREDICT_MAX_SPEED", 0.80)
    PREDICT_MAX_OMEGA = _env_float("QX_STATE_PREDICT_MAX_OMEGA", 0.70)

    # ======================================================================
    # 【GNSS纵向速度在线滤波】只在每个新RMC采样到达时更新一次，不能在
    # 50 Hz融合发布循环内重复滤同一帧。原始速度V_x_raw始终保留；V_x与
    # V_x_filt用于状态预测、控制器速度反馈和停车判定。
    #
    # 20 Hz下3点中值可消除单帧尖峰，alpha=0.25的一阶低通主要抑制
    # RMC地速/航迹角投影抖动。alpha越小越平滑但延迟越大，不建议低于0.15。
    # ======================================================================
    SPEED_FILTER_ENABLE = _env_bool("QX_SPEED_FILTER_ENABLE", True)
    SPEED_FILTER_MEDIAN_WINDOW = max(
        1, int(os.environ.get("QX_SPEED_FILTER_MEDIAN_WINDOW", "3"))
    )
    if SPEED_FILTER_MEDIAN_WINDOW % 2 == 0:
        SPEED_FILTER_MEDIAN_WINDOW += 1
    SPEED_FILTER_ALPHA = _env_float("QX_SPEED_FILTER_ALPHA", 0.25)
    SPEED_FILTER_ALPHA = max(0.05, min(1.0, SPEED_FILTER_ALPHA))
    SPEED_FILTER_RESET_GAP = _env_float("QX_SPEED_FILTER_RESET_GAP", 0.50)
    # 自适应模式默认关闭。需要时可在确认真实加减速判据后启用；当前首轮
    # 实车验证使用固定alpha，避免GNSS尖峰被误判为真实加速后反而快速放大。
    SPEED_FILTER_ADAPTIVE_ENABLE = _env_bool(
        "QX_SPEED_FILTER_ADAPTIVE_ENABLE", False
    )
    SPEED_FILTER_DYNAMIC_ALPHA = _env_float(
        "QX_SPEED_FILTER_DYNAMIC_ALPHA", 0.60
    )
    SPEED_FILTER_DYNAMIC_DELTA = _env_float(
        "QX_SPEED_FILTER_DYNAMIC_DELTA", 0.06
    )
    # 位置帧临时无效时，不使用当前q=2/q=5/被拒绝的GNSS坐标，而从最后
    # 有效CP出发，使用实测纵向速度和IMU航向做极短时航迹推算。超过该时间
    # 立即冻结，避免长时间纯积分漂移。
    INVALID_DEAD_RECKON_MAX_AGE = _env_float(
        "QX_INVALID_DEAD_RECKON_MAX_AGE", 0.20
    )
    INVALID_DEAD_RECKON_MAX_DT = _env_float(
        "QX_INVALID_DEAD_RECKON_MAX_DT", 0.05
    )

    # ======================================================================
    # 【控制状态启动平移】原始参考轨迹始终不动。车辆在参考起点附近
    # 静止后，自动计算一次“原参考起点-实际CP均值”，并把该固定平移
    # 仅施加到规划/MPC使用的控制状态坐标。物理CP、天线坐标、航向角、
    # 坐标轴旋转、IMU偏置、杆臂和轨迹形状均不被修改。
    # ======================================================================
    START_ALIGN_ENABLE = _env_bool("QX_CONTROL_START_ALIGN_ENABLE", True)
    START_ALIGN_SAMPLES = max(3, int(os.environ.get("QX_CONTROL_START_ALIGN_SAMPLES", "20")))
    START_ALIGN_MAX_SPEED = _env_float("QX_CONTROL_START_ALIGN_MAX_SPEED", 0.05)
    START_ALIGN_MAX_DISTANCE = _env_float("QX_CONTROL_START_ALIGN_MAX_DISTANCE", 0.50)
    START_ALIGN_MAX_STD = _env_float("QX_CONTROL_START_ALIGN_MAX_STD", 0.05)
    START_ALIGN_RELEASE_SPEED = _env_float("QX_CONTROL_START_ALIGN_RELEASE_SPEED", 0.06)
    START_ALIGN_REQUIRED_QUALITY = int(os.environ.get(
        "QX_CONTROL_START_ALIGN_QUALITY", "4"
    ))
    # 当前版本固定采用自动锁定，不启动stdin/Enter线程。保留字段仅用于
    # 兼容旧日志和旧环境变量，任何旧的=1设置都不会再次阻塞车辆起步。
    START_ALIGN_REQUIRE_ENTER = False

    # 【每次实验必须确认】YIS525当前运动传感模式
    # AHRS：磁参考航向角；VRU：无参考相对航向角；UNKNOWN：尚未人工确认。
    # YIS525常规输出帧不会可靠报告AHRS/VRU状态，HasMag只代表磁场数据字段存在，
    # 不能证明设备内部航向解算正在使用磁场。因此该值必须与YIS Manager中的设置一致。
    # Ubuntu示例：IMU_HEADING_MODE=AHRS python3 qianxun/POS_qianxun_imu.py
    IMU_HEADING_MODE = _env_heading_mode()

    # ======================================================================
    # 【控制前必须填写/标定】YIS525航向角
    # 1) 将YIS原始yaw转换为真北/+N/逆时针为正的ENU车体航向：
    #      Head_ENU = sign * Yaw_raw + TRUE_NORTH_OFFSET + MOUNT_YAW
    # 2) 再使用qx.Cfg.YAW_ENU_TO_OXYZ_DEG旋转到项目O-XYZ地图坐标系。
    #
    # IMU_YAW_SIGN：安装后通过逆/顺时针手推试验标定一次。
    # IMU_YAW_TRUE_NORTH_OFFSET_DEG：
    #   - 无参考航向模式：每次实验需在已知朝向重新计算启动对准偏置；
    #   - 磁参考航向模式：包含磁偏角及固定航向零偏。
    # IMU_MOUNT_YAW_DEG：IMU参考轴到车辆逻辑+x的安装偏角，安装后标定一次。
    # 车尾在前且IMU箭头指向物理车头时可能接近180 deg，但必须实测，不能猜测。
    # HEAD_ALPHA为航向低通系数，首轮实验保持默认。
    # ======================================================================
    IMU_YAW_SIGN = _env_float("IMU_YAW_SIGN", 1.0)#定了
    # 072547实车直线数据中Head比控制点位移方向平均偏大约20 deg，因此先减20 deg。
    # 这是本次设备上电的实测初值；无参考航向模式下IMU重新上电后仍必须直线复核，
    # 可用同名环境变量覆盖，避免为了临时标定反复改代码。
    IMU_YAW_TRUE_NORTH_OFFSET_DEG = _env_float("IMU_YAW_TRUE_NORTH_OFFSET_DEG", 181.6)
    IMU_MOUNT_YAW_DEG = _env_float("IMU_MOUNT_YAW_DEG", 0.0)#定了
    HEAD_ALPHA = _env_float("IMU_HEAD_ALPHA", 0.30)#默认

    # ======================================================================
    # 【安装后必须标定一次】横滚角Roll和俯仰角Pitch
    # IMU固定在最终安装位置后，先通过人工左右倾斜/抬高车头确定SIGN，再标定ZERO_DEG。
    #
    # 方法A（优先）：已知水平面标定
    #   1) 将车辆完整停放在纵、横两个方向均已用水平仪确认的刚性水平面；
    #   2) 静止30~60 s，分别计算raw_roll和raw_pitch的时间平均值；
    #   3) 按本程序的校正公式填写：
    #        ROLL_ZERO_DEG  = ROLL_SIGN  * mean(raw_roll)
    #        PITCH_ZERO_DEG = PITCH_SIGN * mean(raw_pitch)
    #   4) 标定后仍在该水平面静止时，Roll/Pitch均应接近0 deg。
    #
    # 方法B：同一位置旋转180 deg标定（地面存在轻微未知坡度时使用）
    #   1) 车辆在同一块刚性平面、同一接触区域，以朝向A静止并记录均值；
    #   2) 原地调整为相反朝向A+180 deg，仍在同一接触区域静止并记录均值；
    #   3) 对某一姿态角先进行符号修正，记
    #        q1 = SIGN * mean(raw_angle_at_A)
    #        q2 = SIGN * mean(raw_angle_at_A_plus_180)
    #      则小坡度条件下安装零偏和地面坡度分量可估计为：
    #        ZERO_DEG        = (q1 + q2) / 2
    #        ground_component = (q1 - q2) / 2
    #      Roll和Pitch分别计算，建议重复2~3组后取平均。
    #
    # ZERO_DEG表示“IMU相对车辆逻辑车体的固定安装零偏”，不是实验场地坡度。
    # IMU安装位置、支架和坐标轴配置未变化时，换到任意场地不应重新清零；否则会把
    # 待测地形的真实坡度一起扣除。仅在IMU/支架重新安装、坐标轴/固件配置改变、车辆
    # 静态姿态因结构或载荷发生明显变化，或水平复核发现长期零偏漂移时重新标定。
    # ATTITUDE_ALPHA为姿态低通系数，首轮实验保持默认。
    # ======================================================================
    IMU_ROLL_SIGN = _env_float("IMU_ROLL_SIGN", 1.0)#定了
    IMU_PITCH_SIGN = _env_float("IMU_PITCH_SIGN", -1.0)#定了
    IMU_ROLL_ZERO_DEG = _env_float("IMU_ROLL_ZERO_DEG",3.6)#根据不同实验场地，现在定，没定
    IMU_PITCH_ZERO_DEG = _env_float("IMU_PITCH_ZERO_DEG", 3.8)#根据不同实验场地，现在定，没定
    ATTITUDE_ALPHA = _env_float("IMU_ATTITUDE_ALPHA", 0.25)#默认

    # ======================================================================
    # 【安装后必须标定一次】三轴陀螺仪符号和静止零偏
    # SIGN用于把传感器轴转换到逻辑车体系；BIAS_DPS填写车辆静止时均值。
    # 控制器Omega使用校正后的车体z轴角速度，单位由deg/s转换为rad/s。
    # 必须满足：逆时针Head增大且Omega>0；顺时针Head减小且Omega<0。
    # OMEGA_ALPHA为角速度低通系数，首轮实验保持默认。
    # ======================================================================
    GYRO_X_SIGN = _env_float("IMU_GYRO_X_SIGN", 1.0)#定了
    GYRO_Y_SIGN = _env_float("IMU_GYRO_Y_SIGN", 1.0)#定了
    GYRO_Z_SIGN = _env_float("IMU_GYRO_Z_SIGN", 1.0)#定了
    GYRO_X_BIAS_DPS = _env_float("IMU_GYRO_X_BIAS_DPS", 0.0)#根据不同实验场地，现在定，没定
    GYRO_Y_BIAS_DPS = _env_float("IMU_GYRO_Y_BIAS_DPS", 0.0)#根据不同实验场地，现在定，没定
    GYRO_Z_BIAS_DPS = _env_float("IMU_GYRO_Z_BIAS_DPS", 0.0)#根据不同实验场地，现在定，没定
    OMEGA_ALPHA = _env_float("IMU_OMEGA_ALPHA", 0.35)#默认

    # 【建议核对、当前不进入控制】加速度计轴向符号。
    # 当前仅转换并记录加速度，不积分计算速度或位置。首轮可保持默认，后续做
    # ESKF或运动状态判断前再严格标定。
    ACC_X_SIGN = _env_float("IMU_ACC_X_SIGN", 1.0)#定了
    ACC_Y_SIGN = _env_float("IMU_ACC_Y_SIGN", 1.0)#定了
    ACC_Z_SIGN = _env_float("IMU_ACC_Z_SIGN", 1.0)#定了

    # ======================================================================
    # 【控制前必须实测填写】GNSS天线到车辆控制点的三维杆臂
    # 杆臂在逻辑车体系定义：r_body = p_control - p_antenna，单位m。
    #   +x：逻辑前进方向（本车车尾方向）；+y：前进方向左侧；+z：向上。
    # 默认0只用于接线测试，正式控制前必须用天线相位中心和控制点实测填写。
    # 启用非零杆臂后，旧的天线点地图必须重新核对或重新采集。
    # ======================================================================
    # 诊断时可设QX_ENABLE_LEVER_COMPENSATION=0，使CP临时退化为旧POS的天线点。
    # 如果关闭后坐标恢复正常，问题在杆臂/姿态标定或旧地图点位定义，不在经纬度转换。
    ENABLE_LEVER_COMPENSATION = _env_bool("QX_ENABLE_LEVER_COMPENSATION", True)
    LEVER_X = _env_float("QX_LEVER_X", -0.114)#定了
    LEVER_Y = _env_float("QX_LEVER_Y", 0.004)#定了
    LEVER_Z = _env_float("QX_LEVER_Z", -0.970)#定了

    # 【首轮保持默认False】GGA海拔可用于记录，但小范围斜坡上噪声较大；
    # 初次平地/小坡实验不启用相对Z坐标。
    USE_GGA_RELATIVE_Z = _env_bool("QX_USE_GGA_Z", False)

    # ======================================================================
    # 【GNSS航迹角 + YIS525航向互补融合】
    # 单天线GNSS只能提供车辆运动方向Course，并不能在静止时直接测得车身航向，
    # 履带侧滑时Course也不等于Head。因此GNSS绝不直接替换IMU航向，只在：
    #   q=4、速度足够、转动很小、连续Course稳定且二者差值合理
    # 时，慢速估计IMU航向偏置；低速、急转和Course不稳定时自动退回纯IMU。
    # 这种门控可利用直线段抑制IMU长期漂移，同时避免S弯中把蟹形角误当零偏。
    # ======================================================================
    ENABLE_GNSS_COURSE_CORRECTION = _env_bool("QX_ENABLE_COURSE_CORRECTION", True)
    COURSE_MIN_SPEED = _env_float("QX_COURSE_CORR_MIN_SPEED", 0.28)
    COURSE_MAX_OMEGA = _env_float("QX_COURSE_CORR_MAX_OMEGA", 0.10)
    COURSE_MAX_DIFF_DEG = _env_float("QX_COURSE_CORR_MAX_DIFF_DEG", 12.0)
    COURSE_GAIN = _env_float("QX_COURSE_CORR_GAIN", 0.05)
    COURSE_WINDOW = max(3, int(os.environ.get("QX_COURSE_CORR_WINDOW", "7")))
    COURSE_MAX_STD_DEG = _env_float("QX_COURSE_CORR_MAX_STD_DEG", 3.0)
    COURSE_REQUIRED_QUALITY = int(os.environ.get("QX_COURSE_CORR_QUALITY", "4"))
    COURSE_BIAS_LIMIT_DEG = _env_float("QX_COURSE_CORR_BIAS_LIMIT_DEG", 10.0)
    COURSE_BIAS_RATE_DEGPS = _env_float("QX_COURSE_CORR_BIAS_RATE_DEGPS", 1.0)

    # 当前发布时刻通常比最新IMU帧晚0~20 ms；另外本脚本对Head又做了一阶低通，
    # 其等效延迟约为(1-alpha)/(alpha*fs)。用陀螺z轴只做几十毫秒短时前推，
    # 将Head、GNSS位置和MPC状态统一到发布时刻，不进行长期航向积分。
    HEADING_PREDICT_ENABLE = _env_bool("QX_HEADING_PREDICT_ENABLE", True)
    HEADING_PREDICT_MAX_DT = _env_float("QX_HEADING_PREDICT_MAX_DT", 0.08)
    HEAD_FILTER_DELAY_COMP_ENABLE = _env_bool(
        "QX_HEAD_FILTER_DELAY_COMP_ENABLE", True
    )
    # 负值表示按HEAD_ALPHA和IMU_EXPECTED_HZ自动估算；有实测延迟后可显式覆盖。
    HEAD_FILTER_DELAY_S = _env_float("QX_HEAD_FILTER_DELAY_S", -1.0)

    SLOPE_AVG_SECONDS = _env_float("QX_SLOPE_AVG_SECONDS", 1.0)


def _angle_lpf(previous, measured, alpha):
    if previous is None:
        return wrap_deg(measured)
    return wrap_deg(previous + alpha * wrap_deg(measured - previous))


def _scalar_lpf(previous, measured, alpha):
    if previous is None:
        return float(measured)
    return (1.0 - alpha) * previous + alpha * float(measured)


def _circular_mean_std_deg(values):
    """返回角度序列的圆均值与圆标准差，正确处理+/-180度跨界。"""
    if not values:
        return float("nan"), float("inf")
    radians_values = [math.radians(float(value)) for value in values]
    mean_sin = sum(math.sin(value) for value in radians_values) / len(radians_values)
    mean_cos = sum(math.cos(value) for value in radians_values) / len(radians_values)
    mean_deg = wrap_deg(math.degrees(math.atan2(mean_sin, mean_cos)))
    resultant = max(1e-12, min(1.0, math.hypot(mean_sin, mean_cos)))
    std_deg = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(resultant))))
    return mean_deg, std_deg


def _mat_vec(matrix, vector):
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def body_to_oxyz_rotation(head_deg, pitch_deg, roll_deg):
    """计算车体系到O-XYZ世界系的旋转矩阵R_world_body。

    车体系定义为+x向前、+y向左、+z向上。项目航向角从世界+Y轴开始，
    因此标准绕+Z旋转矩阵所用的角度为theta = head + 90 deg。
    校正后的正俯仰角表示车头抬高；正横滚角遵循绕车体+x轴的右手定则。
    """
    theta = math.radians(head_deg + 90.0)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    ct, st = math.cos(theta), math.sin(theta)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    rz = ((ct, -st, 0.0), (st, ct, 0.0), (0.0, 0.0, 1.0))
    # 正俯仰角使车体+x方向朝世界+z方向抬升，即车头上仰。
    ry = ((cp, 0.0, -sp), (0.0, 1.0, 0.0), (sp, 0.0, cp))
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))

    def mm(a, b):
        return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))

    return mm(mm(rz, ry), rx)


def enu_velocity_to_oxyz(ve, vn, vu=0.0):
    psi = math.radians(qx.Cfg.YAW_ENU_TO_OXYZ_DEG)
    return (
        ve * math.cos(psi) + vn * math.sin(psi),
        -ve * math.sin(psi) + vn * math.cos(psi),
        vu,
    )


def relative_up_from_geodetic(lat, lon, alt):
    """计算相对POS_qianxun既定原点的ENU天向高度。"""
    if not qx._origin_ready:
        return 0.0
    xe, ye, ze = qx.geodetic_to_ecef(lat, lon, alt)
    dx, dy, dz = xe - qx._o_xe, ye - qx._o_ye, ze - qx._o_ze
    lat0 = math.radians(qx._o_lat)
    lon0 = math.radians(qx._o_lon)
    return math.cos(lat0) * math.cos(lon0) * dx + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz


def ecef_to_geodetic(xe, ye, ze):
    """把WGS84地心地固ECEF坐标转换为纬度、经度和椭球高。"""
    lon = math.atan2(ye, xe)
    p = math.hypot(xe, ye)
    lat = math.atan2(ze, p * (1.0 - qx.WGS84_E2))
    height = 0.0
    for _ in range(10):
        s = math.sin(lat)
        n = qx.WGS84_A / math.sqrt(1.0 - qx.WGS84_E2 * s * s)
        height = p / max(math.cos(lat), 1e-12) - n
        next_lat = math.atan2(ze, p * (1.0 - qx.WGS84_E2 * n / max(n + height, 1.0)))
        if abs(next_lat - lat) < 1e-13:
            lat = next_lat
            break
        lat = next_lat
    return math.degrees(lat), math.degrees(lon), height


def oxyz_to_geodetic(x, y, z):
    """POS_qianxun所用ECEF->ENU->O-XYZ局部变换的逆变换。"""
    if not qx._origin_ready:
        return 0.0, 0.0, 0.0
    psi = math.radians(qx.Cfg.YAW_ENU_TO_OXYZ_DEG)
    east = x * math.cos(psi) - y * math.sin(psi)
    north = x * math.sin(psi) + y * math.cos(psi)
    up = z

    lat0 = math.radians(qx._o_lat)
    lon0 = math.radians(qx._o_lon)
    slat, clat = math.sin(lat0), math.cos(lat0)
    slon, clon = math.sin(lon0), math.cos(lon0)
    dx = -slon * east - slat * clon * north + clat * clon * up
    dy = clon * east - slat * slon * north + clat * slon * up
    dz = clat * north + slat * up
    return ecef_to_geodetic(qx._o_xe + dx, qx._o_ye + dy, qx._o_ze + dz)


class ImuCalibrator:
    """把YIS525原始消息转换为标定后的逻辑车体系状态。"""

    def __init__(self, cfg=FusionCfg):
        self.c = cfg
        self.head = None
        self.roll = None
        self.pitch = None
        self.gx = None
        self.gy = None
        self.gz = None

    def update(self, raw):
        yaw_raw = float(raw["yaw"])
        head_enu_raw = wrap_deg(
            self.c.IMU_YAW_SIGN * yaw_raw
            + self.c.IMU_YAW_TRUE_NORTH_OFFSET_DEG
            + self.c.IMU_MOUNT_YAW_DEG
        )
        head_oxyz_raw = enu_heading_to_oxyz(head_enu_raw, qx.Cfg.YAW_ENU_TO_OXYZ_DEG)
        self.head = _angle_lpf(self.head, head_oxyz_raw, self.c.HEAD_ALPHA)

        roll_raw = self.c.IMU_ROLL_SIGN * float(raw["roll"]) - self.c.IMU_ROLL_ZERO_DEG
        pitch_raw = self.c.IMU_PITCH_SIGN * float(raw["pitch"]) - self.c.IMU_PITCH_ZERO_DEG
        self.roll = _scalar_lpf(self.roll, roll_raw, self.c.ATTITUDE_ALPHA)
        self.pitch = _scalar_lpf(self.pitch, pitch_raw, self.c.ATTITUDE_ALPHA)

        gx_raw = self.c.GYRO_X_SIGN * (float(raw["gyro_x"]) - self.c.GYRO_X_BIAS_DPS)
        gy_raw = self.c.GYRO_Y_SIGN * (float(raw["gyro_y"]) - self.c.GYRO_Y_BIAS_DPS)
        gz_raw = self.c.GYRO_Z_SIGN * (float(raw["gyro_z"]) - self.c.GYRO_Z_BIAS_DPS)
        self.gx = _scalar_lpf(self.gx, math.radians(gx_raw), self.c.OMEGA_ALPHA)
        self.gy = _scalar_lpf(self.gy, math.radians(gy_raw), self.c.OMEGA_ALPHA)
        self.gz = _scalar_lpf(self.gz, math.radians(gz_raw), self.c.OMEGA_ALPHA)

        out = dict(raw)
        out.update({
            "Head": self.head,
            "HeadRawProject": head_oxyz_raw,
            "Head_E": head_enu_raw,
            "HeadingMode": self.c.IMU_HEADING_MODE,
            "HeadingReference": {
                "AHRS": "magnetic",
                "VRU": "reference_free",
            }.get(self.c.IMU_HEADING_MODE, "unknown"),
            # 仅说明本帧含磁场字段，不代表航向解算模式一定是AHRS。
            "MagDataPresent": int(raw.get("HasMag", 0)),
            "RollBody": self.roll,
            "PitchBody": self.pitch,
            "GyroBodyX": self.gx,
            "GyroBodyY": self.gy,
            "GyroBodyZ": self.gz,
            "AccBodyX": self.c.ACC_X_SIGN * float(raw.get("acc_x", 0.0)),
            "AccBodyY": self.c.ACC_Y_SIGN * float(raw.get("acc_y", 0.0)),
            "AccBodyZ": self.c.ACC_Z_SIGN * float(raw.get("acc_z", 0.0)),
        })
        return out


class FusionRuntime:
    def __init__(self, cfg=FusionCfg):
        self.c = cfg
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.base_state = None
        self.imu_state = None
        imu_history_n = max(
            20,
            int(cfg.IMU_EXPECTED_HZ * cfg.IMU_HISTORY_SECONDS * 1.5),
        )
        self.imu_history = deque(maxlen=imu_history_n)
        self.last_gnss_mono = 0.0
        self.last_gnss_rx_mono = 0.0
        self.last_gnss_measurement_epoch = 0.0
        self.gnss_rate_window_start = time.monotonic()
        self.gnss_rate_window_frames = 0
        self.gnss_observed_hz = 0.0
        self.last_imu_mono = 0.0
        self.imu_rate_window_start = time.monotonic()
        self.imu_rate_window_frames = 0
        self.imu_observed_hz = 0.0
        self.calibrator = ImuCalibrator(cfg)
        self.course_bias = 0.0
        self.last_course_update = None
        self.last_course_bias_update = None
        self.course_history = deque(maxlen=cfg.COURSE_WINDOW)
        self.heading_fusion_active = False
        self.heading_fusion_status = (
            "collecting" if cfg.ENABLE_GNSS_COURSE_CORRECTION else "disabled"
        )
        self.heading_fusion_course_mean = float("nan")
        self.heading_fusion_course_std = float("nan")
        self.heading_fusion_error = float("nan")
        # COM2当前只输出NMEA RMC/GGA；NtripClient接收的RTCM在接收机内部
        # 送入RTK引擎，脚本无法直接统计RTCM包数。这里统计可真实观测的
        # 有效GGA报文数，以及当前GGA定位质量q持续了多长时间。
        self.gga_valid_count = 0
        self.last_gga_quality = None
        self.gga_quality_since = time.monotonic()
        slope_n = max(1, int(cfg.SLOPE_AVG_SECONDS * cfg.PUBLISH_HZ))
        self.slope_hist = deque(maxlen=slope_n)
        self.start_align_samples = deque(maxlen=cfg.START_ALIGN_SAMPLES)
        self.start_align_last_seq = None
        self.start_align_ref_signature = None
        self.start_align_dx = 0.0
        self.start_align_dy = 0.0
        self.start_align_ready = not cfg.START_ALIGN_ENABLE
        self.start_align_status = "disabled" if self.start_align_ready else "waiting_reference"
        self.start_align_candidate_dx = 0.0
        self.start_align_candidate_dy = 0.0
        self.start_align_candidate_scatter = float("nan")
        self.start_align_candidate_ready = False
        self.start_align_motion_established = False
        # 最后一次通过POS门控的物理CP。PositionValid=0期间，最终发布位置
        # 严格保持该值，不能被状态预测或姿态变化引起的杆臂项绕过门控。
        self.last_valid_physical_control_pos = None
        self.last_output_physical_control_pos = None
        self.last_control_output_mono = None
        self.position_invalid_since = None
        self.control_state_update_seq = 0
        # 速度滤波器只在GNSSSampleMonotonic变化时更新，融合发布到50 Hz时
        # 重复使用最近一次滤波结果，不会因重复滤同一RMC帧而改变等效alpha。
        self.speed_filter_samples = deque(
            maxlen=cfg.SPEED_FILTER_MEDIAN_WINDOW
        )
        self.speed_filter_value = None
        self.speed_filter_median = 0.0
        self.speed_filter_last_sample_mono = 0.0
        self.speed_filter_alpha_used = 1.0
        self.speed_filter_update_count = 0

    def _filter_longitudinal_speed(self, raw_speed, sample_mono):
        """Return causal GNSS longitudinal speed for prediction/control.

        The three-point median and EMA are updated once per new RMC sample.
        No reference speed or motor speed participates, so the filter cannot
        pull the measurement toward the desired trajectory artificially.
        """
        raw_speed = float(raw_speed)
        sample_mono = float(sample_mono)
        if not self.c.SPEED_FILTER_ENABLE:
            self.speed_filter_value = raw_speed
            self.speed_filter_median = raw_speed
            self.speed_filter_alpha_used = 1.0
            return raw_speed

        is_new_sample = (
            self.speed_filter_last_sample_mono <= 0.0
            or sample_mono > self.speed_filter_last_sample_mono + 1e-6
        )
        if not is_new_sample:
            return (
                raw_speed
                if self.speed_filter_value is None
                else float(self.speed_filter_value)
            )

        gap = (
            sample_mono - self.speed_filter_last_sample_mono
            if self.speed_filter_last_sample_mono > 0.0 else 0.0
        )
        self.speed_filter_last_sample_mono = sample_mono
        self.speed_filter_samples.append(raw_speed)
        ordered = sorted(float(value) for value in self.speed_filter_samples)
        median_speed = ordered[len(ordered) // 2]

        reset = (
            self.speed_filter_value is None
            or gap > max(0.0, self.c.SPEED_FILTER_RESET_GAP)
        )
        if reset:
            filtered = median_speed
            alpha = 1.0
        else:
            alpha = self.c.SPEED_FILTER_ALPHA
            if (
                self.c.SPEED_FILTER_ADAPTIVE_ENABLE
                and abs(median_speed - float(self.speed_filter_value))
                    >= self.c.SPEED_FILTER_DYNAMIC_DELTA
            ):
                alpha = max(alpha, min(1.0, self.c.SPEED_FILTER_DYNAMIC_ALPHA))
            filtered = (
                float(self.speed_filter_value)
                + alpha * (median_speed - float(self.speed_filter_value))
            )

        self.speed_filter_value = float(filtered)
        self.speed_filter_median = float(median_speed)
        self.speed_filter_alpha_used = float(alpha)
        self.speed_filter_update_count += 1
        return float(filtered)

    def _reset_reference_shift_candidate(self, status, clear_samples=True):
        if clear_samples:
            self.start_align_samples.clear()
        self.start_align_candidate_dx = 0.0
        self.start_align_candidate_dy = 0.0
        self.start_align_candidate_scatter = float("nan")
        self.start_align_candidate_ready = False
        self.start_align_status = status

    def _update_reference_start_shift(
            self, base, control_pos_aligned, control_speed, fusion_valid):
        """估计并锁存整条参考轨迹的平移量。

        采样点是GNSS采样时刻经IMU对齐和杆臂换算后的车辆CP，不是
        任意一个“最新单点”。只采集新的、静止的q=4位置帧。
        """
        if not self.c.START_ALIGN_ENABLE:
            return

        ref = qx._start_ref
        if ref is None:
            self._reset_reference_shift_candidate("waiting_reference")
            return

        signature = (float(ref["x"]), float(ref["y"]))
        if signature != self.start_align_ref_signature:
            self.start_align_ref_signature = signature
            self._reset_reference_shift_candidate("collecting")
            self.start_align_last_seq = None
            self.start_align_dx = 0.0
            self.start_align_dy = 0.0
            self.start_align_ready = False
            self.start_align_motion_established = False
            print("\n[控制状态平移] 原参考起点刷新，重新采集静止CP: "
                  "X={:+.3f} Y={:+.3f}".format(*signature))

        if self.start_align_ready:
            return
        if not fusion_valid or not int(base.get("PositionValid", 1)):
            self._reset_reference_shift_candidate("waiting_valid_position")
            return

        quality = int(base.get("GGA_quality", base.get("Status", 0)))
        if quality != self.c.START_ALIGN_REQUIRED_QUALITY:
            self._reset_reference_shift_candidate(
                "waiting_q{}".format(self.c.START_ALIGN_REQUIRED_QUALITY)
            )
            return
        if abs(float(control_speed)) > self.c.START_ALIGN_MAX_SPEED:
            # 遥控移动时立即清除旧候选。停车后重新使用最新20个静止帧，
            # 不会把移动前较远位置的样本混入最终锁定点。
            self._reset_reference_shift_candidate("waiting_stationary")
            return

        seq = int(base.get("PositionUpdateSeq", base.get("GGAValidCount", 0)))
        if seq == self.start_align_last_seq:
            return
        self.start_align_last_seq = seq

        cp_x = float(control_pos_aligned[0])
        cp_y = float(control_pos_aligned[1])
        distance = math.hypot(cp_x - signature[0], cp_y - signature[1])
        if distance > self.c.START_ALIGN_MAX_DISTANCE:
            self._reset_reference_shift_candidate("outside_start_tolerance")
            return

        self.start_align_samples.append((cp_x, cp_y))
        self.start_align_status = "collecting"
        if len(self.start_align_samples) < self.c.START_ALIGN_SAMPLES:
            self.start_align_candidate_ready = False
            return

        mean_x = sum(p[0] for p in self.start_align_samples) / len(self.start_align_samples)
        mean_y = sum(p[1] for p in self.start_align_samples) / len(self.start_align_samples)
        scatter = math.sqrt(sum(
            (p[0] - mean_x) ** 2 + (p[1] - mean_y) ** 2
            for p in self.start_align_samples
        ) / len(self.start_align_samples))
        if scatter > self.c.START_ALIGN_MAX_STD:
            self.start_align_status = "position_unstable"
            self.start_align_candidate_ready = False
            self.start_align_candidate_scatter = scatter
            return

        # 平移的是控制状态，因此符号为“原参考起点-实际CP”。
        # 物理CP和原始参考轨迹都不作任何修改。
        self.start_align_candidate_dx = signature[0] - mean_x
        self.start_align_candidate_dy = signature[1] - mean_y
        self.start_align_candidate_scatter = scatter
        self.start_align_candidate_ready = True
        # 候选稳定后立即自动锁定；全程不读取stdin、不等待Enter。
        self.start_align_dx = self.start_align_candidate_dx
        self.start_align_dy = self.start_align_candidate_dy
        self.start_align_ready = True
        self.start_align_status = "locked_auto"
        print("\n[控制状态平移] 已自动锁定: dXctrl={:+.3f}m dYctrl={:+.3f}m "
              "距离={:.3f}m 样本={} 散布={:.3f}m；航向不修正。".format(
                  self.start_align_dx, self.start_align_dy,
                  math.hypot(self.start_align_dx, self.start_align_dy),
                  len(self.start_align_samples), scatter,
              ))

    def qianxun_loop(self):
        """读取千寻RMC/GGA报文并更新滤波后的GNSS天线状态。"""
        ser = None
        buf = b""
        try:
            # 20 Hz GNSS报文必须逐帧低延迟进入融合，不等待固定长度缓冲区填满。
            ser = serial.Serial(qx.Cfg.QX_PORT, qx.Cfg.QX_BAUD, timeout=0.02)
            ser.dtr = qx.Cfg.DTR
            ser.rts = qx.Cfg.RTS
            ser.reset_input_buffer()
            time.sleep(0.2)
            qx.send_qianxun_config(ser, "startup-imu-fusion")
            print("[QX+IMU] 千寻串口已打开: {} @ {}".format(qx.Cfg.QX_PORT, qx.Cfg.QX_BAUD))

            while not self.stop_event.is_set() and not rospy.is_shutdown():
                waiting = getattr(ser, "in_waiting", 0)
                chunk = ser.read(max(1, min(waiting, 512)))
                if chunk:
                    qx.diag["bytes"] += len(chunk)
                    buf += chunk.replace(b"\r", b"\n")
                elif not buf:
                    if (qx.Cfg.AUTO_CONFIG and qx.diag["rmc_seen"] == 0 and
                            time.time() - qx._last_config_t >= qx.Cfg.CONFIG_RETRY_SEC):
                        qx.send_qianxun_config(ser, "retry-no-rmc-imu-fusion")
                    continue

                if len(buf) > 8192:
                    qx.diag["last_line"] = "<buffer too long; wrong baud/protocol>"
                    buf = b""

                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("ascii", "ignore").strip()
                    if not line:
                        continue
                    qx.diag["lines"] += 1
                    qx.diag["last_line"] = line
                    if qx.Cfg.DEBUG_RAW:
                        print("[QX-RAW] {}".format(line))
                    gga = qx.parse_gga(line)
                    if gga is not None:
                        with self.lock:
                            qx._last_gga.update(gga)
                            self.gga_valid_count += 1
                            quality = int(gga.get("quality", 0))
                            if quality != self.last_gga_quality:
                                self.last_gga_quality = quality
                                self.gga_quality_since = time.monotonic()
                        continue
                    rmc = qx.parse_rmc(line)
                    if rmc is None or (qx.Cfg.REQUIRE_VALID_RMC and not rmc["valid"]):
                        continue

                    gnss_rx_mono = time.monotonic()
                    gnss_rx_epoch = time.time()
                    gnss_sample_mono = (
                        gnss_rx_mono - max(0.0, self.c.GNSS_FIXED_LATENCY_S)
                    )
                    rmc_epoch = rmc_measurement_epoch(
                        rmc.get("utc", ""), rmc.get("date", "")
                    )
                    stamp_result = resolve_gnss_measurement_timestamp(
                        rmc_epoch,
                        gnss_rx_epoch,
                        fixed_latency_s=max(
                            0.0, self.c.GNSS_FIXED_LATENCY_S
                        ),
                        previous_epoch=self.last_gnss_measurement_epoch,
                        max_clock_residual_s=(
                            self.c.GNSS_CLOCK_MAX_RESIDUAL_S
                        ),
                    )
                    gnss_sample_epoch = stamp_result["measurement_epoch"]
                    gnss_stamp_source = stamp_result["source"]
                    if not stamp_result["clock_consistent"]:
                        rospy.logerr_throttle(
                            1.0,
                            "RMC UTC与主机时钟失配: offset=%.3fs "
                            "residual=%.3fs limit=%.3fs，融合已置为无效",
                            stamp_result["clock_offset_s"],
                            stamp_result["clock_residual_s"],
                            self.c.GNSS_CLOCK_MAX_RESIDUAL_S,
                        )
                    with self.lock:
                        self.gnss_rate_window_frames += 1
                        rate_elapsed = (
                            gnss_rx_mono - self.gnss_rate_window_start
                        )
                        if rate_elapsed >= 1.0:
                            self.gnss_observed_hz = (
                                self.gnss_rate_window_frames / rate_elapsed
                            )
                            self.gnss_rate_window_start = gnss_rx_mono
                            self.gnss_rate_window_frames = 0
                        qx.update_state(rmc)
                        state = copy.deepcopy(qx.current_state)
                        course_valid = bool(rmc["cog_deg"] is not None and
                                            rmc["speed_mps"] >= qx.Cfg.COG_MIN_SPEED and rmc["valid"])
                        if course_valid:
                            course_e = nmea_cog_to_enu_heading(rmc["cog_deg"])
                            course = enu_heading_to_oxyz(course_e, qx.Cfg.YAW_ENU_TO_OXYZ_DEG)
                        else:
                            course_e = float(state.get("Head_E", 0.0))
                            course = float(state.get("HeadRaw", 0.0))
                        state.update({
                            "Course": course,
                            "Course_E": course_e,
                            "CourseValid": int(course_valid),
                            "OmegaGNSS": float(state.get("Omega", 0.0)),
                            "RMC_utc": rmc.get("utc", ""),
                            "RMC_date": rmc.get("date", ""),
                            "GNSSRxMonotonic": gnss_rx_mono,
                            "GNSSSampleMonotonic": gnss_sample_mono,
                            "GNSSRxTimestamp": gnss_rx_epoch,
                            "GNSSMeasurementTimestamp": gnss_sample_epoch,
                            "GNSSMeasurementTimestampSource": gnss_stamp_source,
                            "GNSSRMCClockAvailable": int(
                                stamp_result["rmc_available"]
                            ),
                            "GNSSClockConsistent": int(
                                stamp_result["clock_consistent"]
                            ),
                            "GNSSClockOffset": float(
                                stamp_result["clock_offset_s"]
                            ),
                            "GNSSClockResidual": float(
                                stamp_result["clock_residual_s"]
                            ),
                        })
                        self.base_state = state
                        self.last_gnss_rx_mono = gnss_rx_mono
                        self.last_gnss_mono = gnss_sample_mono
                        self.last_gnss_measurement_epoch = gnss_sample_epoch
        except Exception as exc:
            print("\n[QX+IMU] 千寻线程停止: {}".format(exc))
            self.stop_event.set()
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

    def imu_callback(self, msg):
        """Consume the exact sensor_msgs/Imu message also consumed by LIO-SAM."""
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w,
                  msg.angular_velocity.x, msg.angular_velocity.y,
                  msg.angular_velocity.z, msg.linear_acceleration.x,
                  msg.linear_acceleration.y, msg.linear_acceleration.z)
        if not all(math.isfinite(float(value)) for value in values):
            rospy.logwarn_throttle(1.0, "收到非有限/imu/data，已丢弃")
            return
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            rospy.logwarn_throttle(1.0, "收到无效IMU四元数，已丢弃")
            return
        roll, pitch, yaw = _rpy_from_quaternion(
            q.x / norm, q.y / norm, q.z / norm, q.w / norm)
        imu_rx_mono = time.monotonic()
        self.imu_rate_window_frames += 1
        rate_elapsed = imu_rx_mono - self.imu_rate_window_start
        if rate_elapsed >= 1.0:
            self.imu_observed_hz = self.imu_rate_window_frames / rate_elapsed
            self.imu_rate_window_start = imu_rx_mono
            self.imu_rate_window_frames = 0
        raw = {
            "yaw": math.degrees(yaw),
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "gyro_x": math.degrees(msg.angular_velocity.x),
            "gyro_y": math.degrees(msg.angular_velocity.y),
            "gyro_z": math.degrees(msg.angular_velocity.z),
            "acc_x": msg.linear_acceleration.x,
            "acc_y": msg.linear_acceleration.y,
            "acc_z": msg.linear_acceleration.z,
            "HasAcc": 1,
            "HasGyro": 1,
            "HasEuler": 1,
            "HasQuaternion": 1,
            "HasMag": 0,
            "DataValid": 1,
            "Timestamp": msg.header.stamp.to_sec(),
            "IMUHeaderStamp": msg.header.stamp.to_sec(),
            "Monotonic": imu_rx_mono,
            "ObservedHz": self.imu_observed_hz,
        }
        try:
            transformed = self.calibrator.update(raw)
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(1.0, "IMU字段异常: %s", exc)
            return
        transformed["IMURxMonotonic"] = imu_rx_mono
        with self.lock:
            self.imu_state = transformed
            self.imu_history.append(copy.deepcopy(transformed))
            self.last_imu_mono = imu_rx_mono

    def _update_heading_fusion(self, imu_head_aligned, base, gyro_z_aligned):
        """用同一GNSS采样时刻的Course慢校正IMU航向偏置。

        这里只估计低频偏置；50 Hz连续航向及所有急转动态仍由YIS525提供。
        每个RMC采样最多更新一次，避免50 Hz发布循环重复利用同一Course。
        """
        if not self.c.ENABLE_GNSS_COURSE_CORRECTION:
            self.heading_fusion_active = False
            self.heading_fusion_status = "disabled"
            return self.course_bias

        stamp = float(self.last_gnss_mono)
        if stamp == self.last_course_update:
            return self.course_bias
        previous_stamp = self.last_course_update
        self.last_course_update = stamp
        self.heading_fusion_active = False
        self.heading_fusion_error = float("nan")

        quality = int(base.get("GGA_quality", base.get("Status", 0)))
        course_valid = bool(base.get("CourseValid", 0))
        speed = abs(float(base.get("Speed", 0.0)))
        if quality != self.c.COURSE_REQUIRED_QUALITY:
            self.course_history.clear()
            self.heading_fusion_course_mean = float("nan")
            self.heading_fusion_course_std = float("nan")
            self.heading_fusion_status = "waiting_q{}".format(
                self.c.COURSE_REQUIRED_QUALITY
            )
            return self.course_bias
        if not course_valid or speed < self.c.COURSE_MIN_SPEED:
            self.course_history.clear()
            self.heading_fusion_course_mean = float("nan")
            self.heading_fusion_course_std = float("nan")
            self.heading_fusion_status = "waiting_speed_course"
            return self.course_bias

        self.course_history.append(float(base["Course"]))
        course_mean, course_std = _circular_mean_std_deg(self.course_history)
        self.heading_fusion_course_mean = course_mean
        self.heading_fusion_course_std = course_std
        if len(self.course_history) < self.c.COURSE_WINDOW:
            self.heading_fusion_status = "collecting_course"
            return self.course_bias
        if course_std > self.c.COURSE_MAX_STD_DEG:
            self.heading_fusion_status = "course_unstable"
            return self.course_bias
        if abs(float(gyro_z_aligned)) > self.c.COURSE_MAX_OMEGA:
            self.heading_fusion_status = "turning_imu_only"
            return self.course_bias

        current_fused = wrap_deg(float(imu_head_aligned) + self.course_bias)
        difference = wrap_deg(course_mean - current_fused)
        self.heading_fusion_error = difference
        if abs(difference) > self.c.COURSE_MAX_DIFF_DEG:
            self.heading_fusion_status = "course_difference_rejected"
            return self.course_bias

        if previous_stamp is None:
            dt = 1.0 / max(1.0, float(self.c.IMU_EXPECTED_HZ))
        else:
            dt = max(1e-3, min(0.25, stamp - float(previous_stamp)))
        correction = self.c.COURSE_GAIN * difference
        max_step = max(0.0, self.c.COURSE_BIAS_RATE_DEGPS) * dt
        correction = max(-max_step, min(max_step, correction))
        bias_limit = max(0.0, self.c.COURSE_BIAS_LIMIT_DEG)
        self.course_bias = max(
            -bias_limit, min(bias_limit, self.course_bias + correction)
        )
        self.last_course_bias_update = stamp
        self.heading_fusion_active = True
        self.heading_fusion_status = "active"
        return self.course_bias

    @staticmethod
    def _interpolate_imu(history, target_mono):
        """在主机单调时间轴上把IMU状态插值到GNSS采样时刻。

        若GNSS帧到达时刻略晚于当前IMU缓存末端，则使用最近IMU帧，并通过
        IMUAlignmentError明确记录这一时间差；不对IMU状态盲目外推。
        """
        if not history:
            return None, float("inf")
        if len(history) == 1:
            sample = copy.deepcopy(history[0])
            stamp = float(sample.get("IMURxMonotonic", target_mono))
            sample["IMUAlignedMonotonic"] = stamp
            return sample, target_mono - stamp

        before = history[0]
        after = history[-1]
        if target_mono <= float(before.get("IMURxMonotonic", target_mono)):
            sample = copy.deepcopy(before)
            stamp = float(sample.get("IMURxMonotonic", target_mono))
            sample["IMUAlignedMonotonic"] = stamp
            return sample, target_mono - stamp
        if target_mono >= float(after.get("IMURxMonotonic", target_mono)):
            sample = copy.deepcopy(after)
            stamp = float(sample.get("IMURxMonotonic", target_mono))
            sample["IMUAlignedMonotonic"] = stamp
            return sample, target_mono - stamp

        for index in range(1, len(history)):
            candidate = history[index]
            candidate_t = float(candidate.get("IMURxMonotonic", target_mono))
            if candidate_t >= target_mono:
                before = history[index - 1]
                after = candidate
                break

        t0 = float(before.get("IMURxMonotonic", target_mono))
        t1 = float(after.get("IMURxMonotonic", target_mono))
        ratio = 0.0 if t1 <= t0 else max(0.0, min(1.0, (target_mono - t0) / (t1 - t0)))
        sample = copy.deepcopy(before)
        angle_keys = ("Head", "HeadRawProject", "Head_E")
        scalar_keys = (
            "RollBody", "PitchBody",
            "GyroBodyX", "GyroBodyY", "GyroBodyZ",
            "AccBodyX", "AccBodyY", "AccBodyZ",
        )
        for key in angle_keys:
            if key in before and key in after:
                sample[key] = wrap_deg(
                    float(before[key])
                    + ratio * wrap_deg(float(after[key]) - float(before[key]))
                )
        for key in scalar_keys:
            if key in before and key in after:
                sample[key] = (
                    float(before[key])
                    + ratio * (float(after[key]) - float(before[key]))
                )
        sample["IMUAlignedMonotonic"] = target_mono
        return sample, 0.0

    def build_fused_state(self):
        now_mono = time.monotonic()
        with self.lock:
            if self.base_state is None or self.imu_state is None:
                return None
            base = copy.deepcopy(self.base_state)
            imu = copy.deepcopy(self.imu_state)
            imu_history = list(self.imu_history)
            gnss_sample_mono = float(
                base.get("GNSSSampleMonotonic", self.last_gnss_mono)
            )
            gnss_rx_mono = float(
                base.get("GNSSRxMonotonic", self.last_gnss_rx_mono)
            )
            gnss_age = max(0.0, now_mono - gnss_sample_mono)
            gnss_rx_age = max(0.0, now_mono - gnss_rx_mono)
            imu_age = max(0.0, now_mono - self.last_imu_mono)
            gga_valid_count = self.gga_valid_count
            gnss_observed_hz = self.gnss_observed_hz
            gga_quality_age = max(0.0, now_mono - self.gga_quality_since)

        imu_valid = imu_age <= self.c.IMU_TIMEOUT
        gnss_clock_consistent = bool(
            int(base.get("GNSSClockConsistent", 1))
        )
        gnss_clock_valid = bool(
            gnss_clock_consistent
            or not self.c.REQUIRE_GNSS_CLOCK_CONSISTENCY
        )
        gnss_valid = bool(
            gnss_age <= self.c.GNSS_TIMEOUT
            and int(base.get("Status", 0)) > 0
            and gnss_clock_valid
        )
        position_valid = bool(int(base.get("PositionValid", 0)))

        imu_aligned, alignment_error = self._interpolate_imu(
            imu_history, gnss_sample_mono
        )
        if imu_aligned is None:
            return None
        alignment_valid = time_alignment_is_valid(
            alignment_error, self.c.IMU_ALIGNMENT_MAX_ERROR_S
        )
        fusion_valid = bool(imu_valid and gnss_valid and alignment_valid)

        # 先把IMU航向插值到GNSS采样时刻，再与同一时刻的RMC Course比较。
        # 这样不会再用“旧GNSS航迹角”直接修正“最新IMU航向”。
        omega_now_z = float(imu["GyroBodyZ"])
        omega_aligned_z = float(imu_aligned["GyroBodyZ"])
        filter_delay = 0.0
        if self.c.HEAD_FILTER_DELAY_COMP_ENABLE:
            if self.c.HEAD_FILTER_DELAY_S >= 0.0:
                filter_delay = self.c.HEAD_FILTER_DELAY_S
            else:
                alpha = max(1e-3, min(1.0, float(self.c.HEAD_ALPHA)))
                filter_delay = (
                    (1.0 - alpha)
                    / (alpha * max(1.0, float(self.c.IMU_EXPECTED_HZ)))
                )
        filter_delay = max(
            0.0, min(max(0.0, self.c.HEADING_PREDICT_MAX_DT), filter_delay)
        )
        aligned_lead_dt = filter_delay if self.c.HEADING_PREDICT_ENABLE else 0.0
        head_imu_aligned = wrap_deg(
            float(imu_aligned["Head"])
            + math.degrees(omega_aligned_z * aligned_lead_dt)
        )
        heading_bias = self._update_heading_fusion(
            head_imu_aligned, base, omega_aligned_z
        )

        # 把最新IMU航向由其采样时刻短时传播到本次POS发布时刻，同时补偿
        # 本脚本HEAD_ALPHA一阶低通造成的等效相位延迟。传播严格限于几十毫秒。
        heading_prediction_dt = 0.0
        if self.c.HEADING_PREDICT_ENABLE and imu_valid:
            heading_prediction_dt = min(
                max(0.0, self.c.HEADING_PREDICT_MAX_DT),
                max(0.0, imu_age) + filter_delay,
            )
        head_imu_now = wrap_deg(
            float(imu["Head"])
            + math.degrees(omega_now_z * heading_prediction_dt)
        )
        head = wrap_deg(head_imu_now + heading_bias)
        head_aligned = wrap_deg(head_imu_aligned + heading_bias)

        # 当前发布时刻使用预测后的融合航向；GNSS天线到CP的杆臂转换必须使用
        # GNSS采样时刻对齐后的融合航向，不能拿未来姿态旋转旧GNSS位置。
        roll = float(imu["RollBody"])
        pitch = float(imu["PitchBody"])
        rotation = body_to_oxyz_rotation(head, pitch, roll)
        roll_aligned = float(imu_aligned["RollBody"])
        pitch_aligned = float(imu_aligned["PitchBody"])
        rotation_aligned = body_to_oxyz_rotation(
            head_aligned, pitch_aligned, roll_aligned
        )
        lever_config = (self.c.LEVER_X, self.c.LEVER_Y, self.c.LEVER_Z)
        lever_body = lever_config if self.c.ENABLE_LEVER_COMPENSATION else (0.0, 0.0, 0.0)
        lever_world_aligned = _mat_vec(rotation_aligned, lever_body)
        lever_world = _mat_vec(rotation, lever_body)

        antenna_x = float(base["UTM_x"])
        antenna_y = float(base["UTM_y"])
        if self.c.USE_GGA_RELATIVE_Z:
            antenna_z = relative_up_from_geodetic(base["Lat"], base["Lon"], base["Alt"])
        else:
            antenna_z = 0.0
        control_pos_aligned = (
            antenna_x + lever_world_aligned[0],
            antenna_y + lever_world_aligned[1],
            antenna_z + lever_world_aligned[2],
        )

        antenna_vel_world = enu_velocity_to_oxyz(
            float(base.get("Ve", 0.0)),
            float(base.get("Vn", 0.0)),
            float(base.get("Vu", 0.0)) if self.c.USE_GGA_RELATIVE_Z else 0.0,
        )
        omega_body = (
            float(imu["GyroBodyX"]),
            float(imu["GyroBodyY"]),
            float(imu["GyroBodyZ"]),
        )
        omega_body_aligned = (
            float(imu_aligned["GyroBodyX"]),
            float(imu_aligned["GyroBodyY"]),
            float(imu_aligned["GyroBodyZ"]),
        )
        lever_velocity_world = _mat_vec(rotation, _cross(omega_body, lever_body))
        lever_velocity_world_aligned = _mat_vec(
            rotation_aligned, _cross(omega_body_aligned, lever_body)
        )
        control_vel_world = tuple(antenna_vel_world[i] + lever_velocity_world[i] for i in range(3))

        body_x_world = tuple(rotation[row][0] for row in range(3))
        body_y_world = tuple(rotation[row][1] for row in range(3))
        body_z_world = tuple(rotation[row][2] for row in range(3))
        v_body_x_raw = _dot(control_vel_world, body_x_world)
        v_body_x = self._filter_longitudinal_speed(
            v_body_x_raw, gnss_sample_mono
        )
        v_body_y = _dot(control_vel_world, body_y_world)
        v_body_z = _dot(control_vel_world, body_z_world)
        control_speed_raw = math.hypot(
            control_vel_world[0], control_vel_world[1]
        )
        control_speed = math.hypot(v_body_x, v_body_y)

        # 把GNSS时刻的CP锚点短时传播到本次50 Hz发布时刻。传播量严格受
        # PREDICT_MAX_AGE/速度上限约束；GNSS失联时不会无限积分漂移。
        prediction_raw_dt = max(0.0, now_mono - gnss_sample_mono)
        prediction_dt = 0.0
        prediction_dx = 0.0
        prediction_dy = 0.0
        prediction_dz = 0.0
        if self.c.STATE_PREDICT_ENABLE and fusion_valid and position_valid:
            prediction_dt = min(prediction_raw_dt, max(0.0, self.c.PREDICT_MAX_AGE))
            v_predict = max(
                -self.c.PREDICT_MAX_SPEED,
                min(self.c.PREDICT_MAX_SPEED, v_body_x),
            )
            # 用对齐航向到当前航向之间的中值方向传播，能够覆盖转弯期间的
            # 一阶弧线运动，同时避免重复积分IMU加速度。
            head_delta = wrap_deg(head - head_aligned)
            max_head_delta = math.degrees(
                max(0.0, self.c.PREDICT_MAX_OMEGA) * prediction_dt
            )
            head_delta = max(-max_head_delta, min(max_head_delta, head_delta))
            head_mid = wrap_deg(head_aligned + 0.5 * head_delta)
            prediction_dx = -v_predict * math.sin(math.radians(head_mid)) * prediction_dt
            prediction_dy = v_predict * math.cos(math.radians(head_mid)) * prediction_dt
            prediction_dz = max(
                -self.c.PREDICT_MAX_SPEED,
                min(self.c.PREDICT_MAX_SPEED, v_body_z),
            ) * prediction_dt
        control_pos = (
            control_pos_aligned[0] + prediction_dx,
            control_pos_aligned[1] + prediction_dy,
            control_pos_aligned[2] + prediction_dz,
        )
        position_output_held = False
        position_dead_reckoning = False
        position_dead_reckoning_age = 0.0
        invalid_prediction_dx = 0.0
        invalid_prediction_dy = 0.0
        if fusion_valid and position_valid:
            self.last_valid_physical_control_pos = tuple(control_pos)
            self.last_output_physical_control_pos = tuple(control_pos)
            self.last_control_output_mono = now_mono
            self.position_invalid_since = None
            self.control_state_update_seq += 1
        elif self.last_output_physical_control_pos is not None:
            if self.position_invalid_since is None:
                self.position_invalid_since = now_mono
            position_dead_reckoning_age = max(
                0.0, now_mono - self.position_invalid_since
            )
            short_predict_allowed = bool(
                self.c.STATE_PREDICT_ENABLE
                and fusion_valid
                and imu_valid
                and position_dead_reckoning_age
                <= max(0.0, self.c.INVALID_DEAD_RECKON_MAX_AGE)
            )
            if short_predict_allowed:
                last_output_mono = (
                    self.last_control_output_mono
                    if self.last_control_output_mono is not None
                    else now_mono
                )
                dead_reckon_dt = min(
                    max(0.0, now_mono - last_output_mono),
                    max(0.0, self.c.INVALID_DEAD_RECKON_MAX_DT),
                )
                v_dead_reckon = max(
                    -self.c.PREDICT_MAX_SPEED,
                    min(self.c.PREDICT_MAX_SPEED, v_body_x),
                )
                # 当前head位于积分区间末端，用陀螺角速度回退半个周期得到
                # 中点航向，一阶近似S弯中的短弧运动。
                head_mid = wrap_deg(
                    head - math.degrees(0.5 * omega_now_z * dead_reckon_dt)
                )
                step_dx = (
                    -v_dead_reckon
                    * math.sin(math.radians(head_mid))
                    * dead_reckon_dt
                )
                step_dy = (
                    v_dead_reckon
                    * math.cos(math.radians(head_mid))
                    * dead_reckon_dt
                )
                previous_output = self.last_output_physical_control_pos
                control_pos = (
                    previous_output[0] + step_dx,
                    previous_output[1] + step_dy,
                    previous_output[2],
                )
                self.last_output_physical_control_pos = tuple(control_pos)
                self.last_control_output_mono = now_mono
                self.control_state_update_seq += 1
                position_dead_reckoning = True
                invalid_prediction_dx = (
                    control_pos[0] - self.last_valid_physical_control_pos[0]
                )
                invalid_prediction_dy = (
                    control_pos[1] - self.last_valid_physical_control_pos[1]
                )
            else:
                # 超过0.20 s或IMU/GNSS速度状态不可信后，停在最后一个短时
                # 预测CP，不再继续积分；等待连续5帧q=4通过底层门控。
                control_pos = self.last_output_physical_control_pos
                position_output_held = True
                self.last_control_output_mono = now_mono
                if self.last_valid_physical_control_pos is not None:
                    invalid_prediction_dx = (
                        control_pos[0] - self.last_valid_physical_control_pos[0]
                    )
                    invalid_prediction_dy = (
                        control_pos[1] - self.last_valid_physical_control_pos[1]
                    )
        else:
            # 启动后尚未得到首批连续q=4，不能从无效坐标开始推算。
            position_output_held = True

        # PositionValid是下游“当前控制位置是否可用”的标志。短时惯推阶段
        # 置1，使规划和控制连续使用预测CP；原始GNSS门控结果另存为
        # GNSSPositionValid。PositionUpdateSeq仍只随有效GNSS校正递增，
        # 因此规划器不会把50 Hz惯推点误当成新的GNSS最近点匹配。
        control_position_valid = bool(
            position_valid or position_dead_reckoning
        )
        # 使用同一把锁保证候选dX/dY、散布和ready状态整体更新。
        with self.lock:
            self._update_reference_start_shift(
                base, control_pos_aligned, control_speed, fusion_valid
            )

        if control_speed >= 0.02:
            control_course = heading_from_delta(control_vel_world[0], control_vel_world[1])
        else:
            control_course = float(base.get("Course", head))

        total_slope = math.degrees(math.acos(max(-1.0, min(1.0,
            math.cos(math.radians(roll)) * math.cos(math.radians(pitch))))))
        self.slope_hist.append(total_slope)
        slope_avg = sum(self.slope_hist) / len(self.slope_hist)

        # 规划/MPC使用固定平移后的控制状态；地理坐标和物理控制点仍保持
        # 真实传感器坐标，便于滑移、杆臂和定位精度审计。
        if (self.start_align_ready and not self.start_align_motion_established
                and abs(float(base.get("Speed", 0.0)))
                >= self.c.START_ALIGN_RELEASE_SPEED):
            self.start_align_motion_established = True
            print("\n[控制状态平移] 已检测到车辆起步，解除起点静止保持。")
        startup_hold = bool(
            self.c.START_ALIGN_ENABLE and self.start_align_ready
            and not self.start_align_motion_established and qx._start_ref is not None
        )
        if startup_hold:
            # 锁定至真正起步前，控制状态精确保持在原参考起点，屏蔽q=4
            # 静止散布、杆臂姿态噪声和短时预测抖动；物理CP仍原样记录。
            control_frame_pos = (
                float(qx._start_ref["x"]),
                float(qx._start_ref["y"]),
                control_pos[2],
            )
        else:
            control_frame_pos = (
                control_pos[0] + self.start_align_dx,
                control_pos[1] + self.start_align_dy,
                control_pos[2],
            )
        control_lat, control_lon, control_alt_local = oxyz_to_geodetic(*control_pos)
        # 未启用GGA相对Z时，保留接收机实测海拔，只叠加刚体杆臂的竖直分量，
        # 避免把任意局部原点高度误当成新的GGA实测海拔。
        if self.c.USE_GGA_RELATIVE_Z:
            control_alt = control_alt_local
        else:
            control_alt = float(base["Alt"]) + lever_world[2]
        beta = wrap_deg(float(base.get("Course", head)) - head) if base.get("CourseValid") else 0.0

        fused = copy.deepcopy(base)
        fused.update({
            # 保留GNSS天线原始测量值，用于杆臂标定和现场诊断。
            "AntennaLat": float(base["Lat"]),
            "AntennaLon": float(base["Lon"]),
            "AntennaAlt": float(base["Alt"]),
            "AntennaX": antenna_x,
            "AntennaY": antenna_y,
            "AntennaZ": antenna_z,
            "ControlXAligned": control_pos_aligned[0],
            "ControlYAligned": control_pos_aligned[1],
            "ControlZAligned": control_pos_aligned[2],
            "PhysicalControlX": control_pos[0],
            "PhysicalControlY": control_pos[1],
            "PhysicalControlZ": control_pos[2],
            "ControlFrameX": control_frame_pos[0],
            "ControlFrameY": control_frame_pos[1],
            "ControlFrameZ": control_frame_pos[2],
            "ControlFrameAlignedX": control_pos_aligned[0],
            "ControlFrameAlignedY": control_pos_aligned[1],
            "ControlFrameAlignedZ": control_pos_aligned[2],
            "ControlFrameDx": self.start_align_dx,
            "ControlFrameDy": self.start_align_dy,
            "ControlFrameReady": int(self.start_align_ready),
            "ControlFrameEnable": int(self.c.START_ALIGN_ENABLE),
            "ControlFrameSamples": len(self.start_align_samples),
            "ControlFrameStatus": self.start_align_status,
            "ControlFrameStartupHold": int(startup_hold),
            # 参考轨迹不再移动；保留兼容字段但平移量固定为0。
            "ReferenceShiftDx": 0.0,
            "ReferenceShiftDy": 0.0,
            "ReferenceShiftReady": int(self.start_align_ready),
            "ReferenceShiftEnable": int(self.c.START_ALIGN_ENABLE),
            "ReferenceShiftSamples": len(self.start_align_samples),
            "ReferenceShiftStatus": self.start_align_status,
            "ReferenceShiftCandidateDx": self.start_align_candidate_dx,
            "ReferenceShiftCandidateDy": self.start_align_candidate_dy,
            "ReferenceShiftCandidateReady": int(self.start_align_candidate_ready),
            "ReferenceShiftCandidateScatter": self.start_align_candidate_scatter,
            "ReferenceShiftManualConfirm": int(self.c.START_ALIGN_REQUIRE_ENTER),
            # CurGNSS中的位置从此表示车辆控制点，不再表示GNSS天线点。
            "Lat": control_lat,
            "Lon": control_lon,
            "Alt": control_alt,
            "UTM_x": control_frame_pos[0],
            "UTM_y": control_frame_pos[1],
            "Z": control_frame_pos[2],
            "OriginLat": base.get("OriginLat"),
            "OriginLon": base.get("OriginLon"),
            "OriginAlt": base.get("OriginAlt"),
            "OriginMode": base.get("OriginMode", "unknown"),
            "YawEnuToOxyzDeg": base.get("YawEnuToOxyzDeg", 0.0),
            # 车体航向Head与GNSS地面速度方向Course始终分开保存。
            "Head": head,
            "HeadAligned": head_aligned,
            "HeadIMU": head_imu_now,
            "HeadIMUAligned": head_imu_aligned,
            "HeadGNSSCourse": float(base.get("Course", float("nan"))),
            "HeadRaw": float(imu["HeadRawProject"]),
            "Head_E": float(imu["Head_E"]),
            "Course": float(base.get("Course", 0.0)),
            "Course_E": float(base.get("Course_E", 0.0)),
            "CourseControl": control_course,
            "CourseValid": int(base.get("CourseValid", 0)),
            "Beta": beta,
            "HeadingSource": (
                "YIS525_{}_GNSS_COURSE_COMPLEMENTARY".format(
                    self.c.IMU_HEADING_MODE
                )
                if self.c.ENABLE_GNSS_COURSE_CORRECTION
                else "YIS525_{}".format(self.c.IMU_HEADING_MODE)
            ),
            "HeadingFusionEnable": int(self.c.ENABLE_GNSS_COURSE_CORRECTION),
            "HeadingFusionActive": int(self.heading_fusion_active),
            "HeadingFusionBias": float(self.course_bias),
            "HeadingFusionError": float(self.heading_fusion_error),
            "HeadingFusionCourseMean": float(self.heading_fusion_course_mean),
            "HeadingFusionCourseStd": float(self.heading_fusion_course_std),
            "HeadingFusionStatus": self.heading_fusion_status,
            "HeadingFusionSamples": len(self.course_history),
            "HeadingFusionRequiredQuality": int(self.c.COURSE_REQUIRED_QUALITY),
            "HeadingFusionMinSpeed": float(self.c.COURSE_MIN_SPEED),
            "HeadingFusionMaxOmega": float(self.c.COURSE_MAX_OMEGA),
            "HeadingFusionMaxDiff": float(self.c.COURSE_MAX_DIFF_DEG),
            "HeadingFusionGain": float(self.c.COURSE_GAIN),
            "HeadingFusionWindow": int(self.c.COURSE_WINDOW),
            "HeadingFusionMaxCourseStd": float(self.c.COURSE_MAX_STD_DEG),
            "HeadingFusionBiasLimit": float(self.c.COURSE_BIAS_LIMIT_DEG),
            "HeadingFusionBiasRate": float(self.c.COURSE_BIAS_RATE_DEGPS),
            "HeadingPredictionEnable": int(self.c.HEADING_PREDICT_ENABLE),
            "HeadingPredictionDt": float(heading_prediction_dt),
            "HeadingFilterDelayComp": float(filter_delay),
            "HeadingPredictionMaxDt": float(self.c.HEADING_PREDICT_MAX_DT),
            "HeadingFilterDelayCompEnable": int(
                self.c.HEAD_FILTER_DELAY_COMP_ENABLE
            ),
            "HeadingMode": str(imu.get("HeadingMode", self.c.IMU_HEADING_MODE)),
            "HeadingReference": str(imu.get("HeadingReference", "unknown")),
            "MagDataPresent": int(imu.get("MagDataPresent", 0)),
            # 姿态角和角速度来自标定后的IMU逻辑车体系。
            "Roll": roll,
            "Pitch": pitch,
            "Omega": omega_body[2] if imu_valid else 0.0,
            # This sample is interpolated onto the same instant as the GNSS
            # velocity and is the angular rate used by /vehicle/state.
            "OmegaAligned": omega_aligned_z if imu_valid else 0.0,
            "GyroBodyX": omega_body[0],
            "GyroBodyY": omega_body[1],
            "GyroBodyZ": omega_body[2],
            "AccBodyX": float(imu["AccBodyX"]),
            "AccBodyY": float(imu["AccBodyY"]),
            "AccBodyZ": float(imu["AccBodyZ"]),
            "IMU_tid": int(imu.get("tid", 0)),
            "IMU_smp_timestamp": int(imu.get("smp_timestamp", 0)),
            "IMUObservedHz": float(imu.get("ObservedHz", 0.0)),
            # 控制点速度包含omega叉乘杆臂产生的旋转速度补偿。
            "Speed": control_speed,
            "SpeedRaw": control_speed_raw,
            "SpeedFilt": control_speed,
            "V_x_raw": v_body_x_raw,
            "V_x_filt": v_body_x,
            "V_x": v_body_x,
            "V_y": v_body_y,
            "V_z": v_body_z,
            "V_X": control_vel_world[0],
            "V_Y": control_vel_world[1],
            "V_Z": control_vel_world[2],
            "SpeedFilterEnable": int(self.c.SPEED_FILTER_ENABLE),
            "SpeedFilterMedianWindow": int(
                self.c.SPEED_FILTER_MEDIAN_WINDOW
            ),
            "SpeedFilterAlpha": float(self.c.SPEED_FILTER_ALPHA),
            "SpeedFilterAlphaUsed": float(self.speed_filter_alpha_used),
            "SpeedFilterMedian": float(self.speed_filter_median),
            "SpeedFilterUpdateCount": int(self.speed_filter_update_count),
            # 斜坡规划器和控制器需要的坡度字段。
            "slope": total_slope,
            "slope_avg": slope_avg,
            "grade": pitch,
            "slope_longitudinal": pitch,
            "slope_lateral": roll,
            "grade_source": "imu_pitch",
            # GNSS、IMU数据新鲜度和组合状态健康诊断。
            "GNSSAge": gnss_age,
            "GNSSRxAge": gnss_rx_age,
            "GNSSObservedHz": gnss_observed_hz,
            "IMUAge": imu_age,
            "GNSSSampleMonotonic": gnss_sample_mono,
            "GNSSRxMonotonic": gnss_rx_mono,
            "GNSSMeasurementTimestamp": float(
                base.get("GNSSMeasurementTimestamp", time.time() - gnss_age)
            ),
            "GNSSRxTimestamp": float(
                base.get("GNSSRxTimestamp", time.time() - gnss_rx_age)
            ),
            "GNSSMeasurementTimestampSource": str(
                base.get("GNSSMeasurementTimestampSource", "ipc_age_fallback")
            ),
            "GNSSRMCClockAvailable": int(
                base.get("GNSSRMCClockAvailable", 0)
            ),
            "GNSSClockConsistent": int(gnss_clock_consistent),
            "GNSSClockRequired": int(
                self.c.REQUIRE_GNSS_CLOCK_CONSISTENCY
            ),
            "GNSSClockOffset": float(
                base.get("GNSSClockOffset", float("nan"))
            ),
            "GNSSClockResidual": float(
                base.get("GNSSClockResidual", float("nan"))
            ),
            "GNSSClockMaxResidual": float(
                self.c.GNSS_CLOCK_MAX_RESIDUAL_S
            ),
            "IMUAlignedMonotonic": float(
                imu_aligned.get("IMUAlignedMonotonic", gnss_sample_mono)
            ),
            "IMUAlignmentError": alignment_error,
            "IMUAlignmentValid": int(alignment_valid),
            "IMUAlignmentMaxError": float(
                self.c.IMU_ALIGNMENT_MAX_ERROR_S
            ),
            "PredictionRawDt": prediction_raw_dt,
            "PredictionDt": prediction_dt,
            "PredictionDx": prediction_dx,
            "PredictionDy": prediction_dy,
            "PredictionDz": prediction_dz,
            "StatePredicted": int(
                (self.c.STATE_PREDICT_ENABLE and prediction_dt > 0.0)
                or position_dead_reckoning
            ),
            "StatePredictionCapped": int(
                self.c.STATE_PREDICT_ENABLE
                and prediction_raw_dt > self.c.PREDICT_MAX_AGE
            ),
            "PositionOutputHeld": int(position_output_held),
            "GNSSPositionValid": int(position_valid),
            "PositionValid": int(control_position_valid),
            "PositionDeadReckoning": int(position_dead_reckoning),
            "PositionDeadReckoningAge": float(position_dead_reckoning_age),
            "InvalidPredictionDx": float(invalid_prediction_dx),
            "InvalidPredictionDy": float(invalid_prediction_dy),
            "ControlStateUpdateSeq": int(self.control_state_update_seq),
            "GGAValidCount": gga_valid_count,
            "GGAQualityAge": gga_quality_age,
            "GNSSValid": int(gnss_valid),
            "IMUValid": int(imu_valid),
            "FusionValid": int(fusion_valid),
            "Status": int(base.get("Status", 0)) if fusion_valid else 0,
            "sys_status": 2 if fusion_valid else 0,
            "Timestamp": time.time(),
            "Monotonic": now_mono,
            "StateEstimateMonotonic": now_mono,
            "LeverX": self.c.LEVER_X,
            "LeverY": self.c.LEVER_Y,
            "LeverZ": self.c.LEVER_Z,
            "LeverEnabled": int(self.c.ENABLE_LEVER_COMPENSATION),
            "LeverAppliedX": lever_body[0],
            "LeverAppliedY": lever_body[1],
            "LeverAppliedZ": lever_body[2],
            "LeverVx": lever_velocity_world[0],
            "LeverVy": lever_velocity_world[1],
            "LeverVz": lever_velocity_world[2],
            "LeverAlignedX": lever_world_aligned[0],
            "LeverAlignedY": lever_world_aligned[1],
            "LeverAlignedZ": lever_world_aligned[2],
            "LeverAlignedVx": lever_velocity_world_aligned[0],
            "LeverAlignedVy": lever_velocity_world_aligned[1],
            "LeverAlignedVz": lever_velocity_world_aligned[2],
        })
        return fused


def guide_text(state):
    if qx._start_ref is None:
        return ""
    # 参考起点始终是规划器生成的原始固定起点；UTM_x/Y已是控制状态坐标。
    ref_x = qx._start_ref["x"]
    ref_y = qx._start_ref["y"]
    dx = ref_x - state["UTM_x"]
    dy = ref_y - state["UTM_y"]
    distance = math.hypot(dx, dy)
    head_err = wrap_deg(state["Head"] - qx._start_ref["head"])
    if distance <= qx.Cfg.GUIDE_POS_TOL and (not qx.Cfg.GUIDE_REQUIRE_HEADING or
            abs(head_err) <= qx.Cfg.GUIDE_HEAD_TOL_DEG):
        return " | 已到参考起点"
    return " | toStart:{:.2f}m dHead:{:+.1f}deg".format(distance, head_err)


def callback_reference_start(msg):
    """用规划器本轮锁存消息刷新参考起点，避免读取不到或误读旧地图。"""
    try:
        data = json.loads(msg.data)
        x = float(data["x"])
        y = float(data["y"])
        head = wrap_deg(float(data["head"]))
        if not all(math.isfinite(v) for v in (x, y, head)):
            raise ValueError("reference start contains non-finite values")
        previous = qx._start_ref
        changed = (
            previous is None
            or abs(previous["x"] - x) > 1e-6
            or abs(previous["y"] - y) > 1e-6
            or abs(wrap_deg(previous["head"] - head)) > 1e-6
        )
        qx._start_ref = {
            "x": x,
            "y": y,
            "head": head,
            "source": str(data.get("source", "/control/reference_start")),
        }
        qx._start_arrived_reported = False
        if changed:
            print("\n[QX+IMU] 已从本轮规划刷新参考起点: "
                  "X={:+.3f} Y={:+.3f} Head={:+.2f}deg 点数={}".format(
                      x, y, head, int(data.get("path_points", 0))))
    except Exception as exc:
        print("\n[QX+IMU] 参考起点消息无效: {}".format(exc))


def publish_fused(pub, pub_ros_location, pub_ros_pose, pub_ros_sensor, state):
    pub.sendPro(b"CurGNSS", state)

    location_msg = std_msgs.msg.String()
    location_msg.data = json.dumps(state)
    pub_ros_location.publish(location_msg)

    pose_msg = geometry_msgs.msg.Pose2D()
    pose_msg.x = state["UTM_x"]
    pose_msg.y = state["UTM_y"]
    pose_msg.theta = math.radians(state["Head"])
    pub_ros_pose.publish(pose_msg)

    sensor_msg = std_msgs.msg.String()
    sensor_msg.data = json.dumps({
        "Roll": state["Roll"],
        "Pitch": state["Pitch"],
        "Alt": state["Alt"],
        "Omega": state["Omega"],
        "slope": state["slope"],
        "FusionValid": state["FusionValid"],
    })
    pub_ros_sensor.publish(sensor_msg)


def print_configuration():
    print("===== 千寻 + YIS525 组合定位（工程版，非ESKF） =====")
    print("千寻: {} @ {} | IMU订阅: {} | CurGNSS发布: {} @ {:.1f}Hz".format(
        qx.Cfg.QX_PORT, qx.Cfg.QX_BAUD, FusionCfg.IMU_TOPIC,
        qx.Cfg.GnssAddr, FusionCfg.PUBLISH_HZ))
    mode_text = {
        "AHRS": "磁参考航向角",
        "VRU": "无参考相对航向角",
        "UNKNOWN": "未声明，不能判断",
    }[FusionCfg.IMU_HEADING_MODE]
    print("航向模式: {} ({}) | 注意：HasMag只表示收到磁场字段，不能用来自动判断模式。".format(
        FusionCfg.IMU_HEADING_MODE, mode_text))
    print("航向: Head_ENU = {:+.1f}*YawRaw {:+.2f}(真北/磁偏) {:+.2f}(安装)；随后减地图旋转角{:+.2f}".format(
        FusionCfg.IMU_YAW_SIGN, FusionCfg.IMU_YAW_TRUE_NORTH_OFFSET_DEG,
        FusionCfg.IMU_MOUNT_YAW_DEG, qx.Cfg.YAW_ENU_TO_OXYZ_DEG))
    print("姿态: roll={:+.1f}*raw-{:+.3f}, pitch={:+.1f}*raw-{:+.3f}".format(
        FusionCfg.IMU_ROLL_SIGN, FusionCfg.IMU_ROLL_ZERO_DEG,
        FusionCfg.IMU_PITCH_SIGN, FusionCfg.IMU_PITCH_ZERO_DEG))
    print("角速度: sign xyz=({:+.0f},{:+.0f},{:+.0f}), bias dps=({:+.4f},{:+.4f},{:+.4f})".format(
        FusionCfg.GYRO_X_SIGN, FusionCfg.GYRO_Y_SIGN, FusionCfg.GYRO_Z_SIGN,
        FusionCfg.GYRO_X_BIAS_DPS, FusionCfg.GYRO_Y_BIAS_DPS, FusionCfg.GYRO_Z_BIAS_DPS))
    print("杆臂补偿={} | r_body=p_control-p_antenna=({:+.3f},{:+.3f},{:+.3f})m | GGA相对Z={}".format(
        int(FusionCfg.ENABLE_LEVER_COMPENSATION), FusionCfg.LEVER_X, FusionCfg.LEVER_Y,
        FusionCfg.LEVER_Z, int(FusionCfg.USE_GGA_RELATIVE_Z)))
    print(
        "时间对齐: IMU目标{:.0f}Hz 缓存{:.1f}s | 状态预测={} "
        "GNSS固定延迟={:.3f}s 对齐门限={:.3f}s 时钟残差门限={:.3f}s "
        "时钟门控={} 最大外推={:.2f}s | 融合发布={:.0f}Hz MPC目标=20Hz".format(
            FusionCfg.IMU_EXPECTED_HZ,
            FusionCfg.IMU_HISTORY_SECONDS,
            int(FusionCfg.STATE_PREDICT_ENABLE),
            FusionCfg.GNSS_FIXED_LATENCY_S,
            FusionCfg.IMU_ALIGNMENT_MAX_ERROR_S,
            FusionCfg.GNSS_CLOCK_MAX_RESIDUAL_S,
            int(FusionCfg.REQUIRE_GNSS_CLOCK_CONSISTENCY),
            FusionCfg.PREDICT_MAX_AGE,
            FusionCfg.PUBLISH_HZ,
        )
    )
    auto_filter_delay = (
        (1.0 - max(1e-3, min(1.0, FusionCfg.HEAD_ALPHA)))
        / (
            max(1e-3, min(1.0, FusionCfg.HEAD_ALPHA))
            * max(1.0, FusionCfg.IMU_EXPECTED_HZ)
        )
    )
    configured_filter_delay = (
        FusionCfg.HEAD_FILTER_DELAY_S
        if FusionCfg.HEAD_FILTER_DELAY_S >= 0.0
        else auto_filter_delay
    )
    print(
        "动态航向时序: 短时陀螺预测={} 最大{:.3f}s | Head低通延迟补偿={} {:.3f}s".format(
            int(FusionCfg.HEADING_PREDICT_ENABLE),
            FusionCfg.HEADING_PREDICT_MAX_DT,
            int(FusionCfg.HEAD_FILTER_DELAY_COMP_ENABLE),
            configured_filter_delay,
        )
    )
    print("控制状态起点平移: enable={} 样本={} q={} 静止速度<={:.2f}m/s "
          "散布<={:.2f}m 距原起点<={:.2f}m 起步释放速度={:.2f}m/s 自动锁定=1".format(
              int(FusionCfg.START_ALIGN_ENABLE), FusionCfg.START_ALIGN_SAMPLES,
              FusionCfg.START_ALIGN_REQUIRED_QUALITY,
              FusionCfg.START_ALIGN_MAX_SPEED, FusionCfg.START_ALIGN_MAX_STD,
              FusionCfg.START_ALIGN_MAX_DISTANCE,
              FusionCfg.START_ALIGN_RELEASE_SPEED,
          ))
    if qx.Cfg.ORIGIN_LAT is None or qx.Cfg.ORIGIN_LON is None:
        print("[地图严重提醒] ORIGIN_LAT/LON仍为None：本次第一帧天线位置会成为新原点。")
        print("               若继续使用旧四角点/旧参考轨迹，坐标会整体平移，必须填回采图时固定原点。")
    if math.sqrt(FusionCfg.LEVER_X ** 2 + FusionCfg.LEVER_Y ** 2 + FusionCfg.LEVER_Z ** 2) < 1e-6:
        print("[配置提醒] 杆臂仍为0。测量后设置QX_LEVER_X/Y/Z，并用控制点坐标重新核对规划点。")
    print("[安全标定] 手推逆时针时必须 Head增加且Omega>0；顺时针时必须 Head减小且Omega<0。")
    print("[安全标定] 车辆逻辑+x必须指向实际循迹前方；车尾在前且IMU箭头指车头时安装偏角通常接近180deg，但必须实测。")
    if FusionCfg.IMU_HEADING_MODE == "UNKNOWN":
        print("[航向严重提醒] 尚未声明AHRS/VRU。请先在YIS Manager确认模式，再设置IMU_HEADING_MODE。")
    elif FusionCfg.IMU_HEADING_MODE == "VRU":
        print("[VRU提醒] IMU重新上电/复位后必须在已知朝向重新标定IMU_YAW_TRUE_NORTH_OFFSET_DEG。")
    else:
        print("[AHRS提醒] 使用磁参考航向；必须完成整车安装状态下的磁场校准，并核对磁偏角/固定零偏。")
    print(
        "[当前策略] Head=YIS525 {}连续航向 + GNSS Course门控慢校正；"
        "enable={} q={} v>={:.2f}m/s |w|<={:.2f}rad/s 窗口={} std<={:.1f}deg".format(
            FusionCfg.IMU_HEADING_MODE,
            int(FusionCfg.ENABLE_GNSS_COURSE_CORRECTION),
            FusionCfg.COURSE_REQUIRED_QUALITY,
            FusionCfg.COURSE_MIN_SPEED,
            FusionCfg.COURSE_MAX_OMEGA,
            FusionCfg.COURSE_WINDOW,
            FusionCfg.COURSE_MAX_STD_DEG,
        )
    )


def main(init_ros_node=True):
    if init_ros_node:
        rospy.init_node("POS_qianxun_imu", anonymous=True)
    pub_ros_pose = rospy.Publisher("/bus/pose", geometry_msgs.msg.Pose2D, queue_size=10)
    pub_ros_location = rospy.Publisher("/fusion_location", std_msgs.msg.String, queue_size=10)
    pub_ros_sensor = rospy.Publisher("/bus/sensor", std_msgs.msg.String, queue_size=10)

    ctx = proContext()
    pub = ctx.socket(zmq.PUB)
    pub.bind(qx.Cfg.GnssAddr)

    print_configuration()
    qx._start_ref = qx.load_start_reference()
    if qx._start_ref is not None:
        print("[QX+IMU] 参考起点 X={:+.3f} Y={:+.3f} Head={:+.2f}deg".format(
            qx._start_ref["x"], qx._start_ref["y"], qx._start_ref["head"]))
        print("         当前为地图文件回退值；规划器启动后将由"
              "/control/reference_start自动刷新为本轮起点。")
    else:
        print("[QX+IMU] 暂无参考起点；等待规划器锁存发布"
              "/control/reference_start。")
    rospy.Subscriber(
        "/control/reference_start", std_msgs.msg.String,
        callback_reference_start, queue_size=1
    )

    runtime = FusionRuntime()
    rospy.Subscriber(
        FusionCfg.IMU_TOPIC, sensor_msgs.msg.Imu, runtime.imu_callback,
        queue_size=200, tcp_nodelay=True
    )
    threads = [
        threading.Thread(target=runtime.qianxun_loop, name="qianxun-reader", daemon=True),
    ]
    for thread in threads:
        thread.start()

    period = 1.0 / max(FusionCfg.PUBLISH_HZ, 1.0)
    last_print = 0.0
    try:
        while not rospy.is_shutdown() and not runtime.stop_event.is_set():
            cycle_start = time.monotonic()
            state = runtime.build_fused_state()
            if state is not None:
                publish_fused(pub, pub_ros_location, pub_ros_pose, pub_ros_sensor, state)
                now = time.monotonic()
                if now - last_print >= 1.0 / max(FusionCfg.PRINT_HZ, 0.1):
                    last_print = now
                    print(
                        "\r[QX+IMU] CP({x:+7.2f},{y:+7.2f},{z:+5.2f}) "
                        "CTRLFRAME {astatus} A:{ardy}/{ars:d} HOLD:{hold} C:{crdy} "
                        "dCand:{cdx:+.2f}/{cdy:+.2f} dCtrl:{adx:+.2f}/{ady:+.2f} "
                        "ANT({ax:+7.2f},{ay:+7.2f}) dCP:({dlx:+5.2f},{dly:+5.2f}) "
                        "H:{head:+7.2f} Hi:{himu:+7.2f} Hb:{hbias:+5.2f} "
                        "HF:{hactive}/{hstatus} C:{course:+7.2f} Cstd:{cstd:4.1f} "
                        "beta:{beta:+6.2f} Vx:{vx:+5.2f} Vy:{vy:+5.2f} "
                        "w:{w:+6.3f} R/P:{roll:+5.1f}/{pitch:+5.1f} "
                        "slope:{slope:4.1f} slope_avg:{slope_avg:4.1f} "
                        "q:{q:d} qAge:{qage:5.1f}s GGA#:{gga:d} sat:{sat:02d} hdop:{hdop:4.1f} "
                        "HM:{hm}/{href} Mag:{mag} Hz G/I:{ghz:.1f}/{ihz:.1f} "
                        "age G/I:{ga:.2f}/{ia:.2f}s "
                        "pred:{pred:.3f}s Hpred:{hpred:.3f}s dXY:{pdx:+.3f}/{pdy:+.3f} "
                        "align:{align:+.1f}ms/{aok} clk:{clk:+.1f}ms/{cok} "
                        "valid:{valid}{guide}   ".format(
                            x=state["UTM_x"], y=state["UTM_y"], z=state["Z"], head=state["Head"],
                            ardy=state["ControlFrameReady"], ars=state["ControlFrameSamples"],
                            hold=state["ControlFrameStartupHold"],
                            astatus=state["ControlFrameStatus"],
                            crdy=state["ReferenceShiftCandidateReady"],
                            cdx=state["ReferenceShiftCandidateDx"],
                            cdy=state["ReferenceShiftCandidateDy"],
                            adx=state["ControlFrameDx"], ady=state["ControlFrameDy"],
                            ax=state["AntennaX"], ay=state["AntennaY"],
                            dlx=state["PhysicalControlX"] - state["AntennaX"],
                            dly=state["PhysicalControlY"] - state["AntennaY"],
                            himu=state["HeadIMU"], hbias=state["HeadingFusionBias"],
                            hactive=state["HeadingFusionActive"],
                            hstatus=state["HeadingFusionStatus"],
                            course=state["Course"], beta=state["Beta"], vx=state["V_x"], vy=state["V_y"],
                            cstd=state["HeadingFusionCourseStd"],
                            w=state["Omega"], roll=state["Roll"], pitch=state["Pitch"],
                            slope=state["slope"], slope_avg=state["slope_avg"],
                            q=int(state.get("GGA_quality", 0)),
                            qage=float(state.get("GGAQualityAge", 0.0)),
                            gga=int(state.get("GGAValidCount", 0)),
                            sat=int(state.get("GGA_sats", 0)),
                            hdop=float(state.get("GGA_hdop", 99.9)),
                            hm=state["HeadingMode"], href=state["HeadingReference"],
                            mag=state["MagDataPresent"],
                            ghz=state["GNSSObservedHz"],
                            ihz=state["IMUObservedHz"],
                            ga=state["GNSSAge"], ia=state["IMUAge"],
                            pred=state["PredictionDt"],
                            hpred=state["HeadingPredictionDt"],
                            pdx=state["PredictionDx"], pdy=state["PredictionDy"],
                            align=1000.0 * state["IMUAlignmentError"],
                            aok=state["IMUAlignmentValid"],
                            clk=1000.0 * state["GNSSClockResidual"],
                            cok=state["GNSSClockConsistent"],
                            valid=state["FusionValid"],
                            guide=guide_text(state)),
                        end="",
                        flush=True,
                    )
            else:
                now = time.monotonic()
                if now - last_print >= 1.0:
                    last_print = now
                    print("\r[QX+IMU] 等待千寻和YIS525同时就绪...", end="", flush=True)

            elapsed = time.monotonic() - cycle_start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        runtime.stop_event.set()
        for thread in threads:
            thread.join(timeout=1.0)
        try:
            qx.save_position_analysis()
        except Exception as exc:
            print("\n[QX+IMU] 保存位置分析失败: {}".format(exc))
        pub.close(0)
        ctx.term()
        print("\n[QX+IMU] 已停止。")


if __name__ == "__main__":
    main()
