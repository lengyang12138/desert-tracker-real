"""Bridge the proven plan_try state/CAN contract to standard ROS navigation.

Input:
  /fusion_location (std_msgs/String JSON, O-XYZ convention)
  /cmd_vel       (geometry_msgs/Twist, ROS convention)

Output:
  /odometry/imu_incremental and TF map->odom->base_link
  /vehicle/state (measurement-time, base_link-twist control state)
  /localization/qianxun_seed (healthy absolute pose for NDT cold start)
  ZMQ PoliAcc/PoliSteer/PoliBrake for the existing SocketCAN bridge
"""

import json
import math
import threading
import time

import rospy
import tf2_ros
import zmq
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .contracts import (
    clamp,
    oxyz_heading_to_ros_yaw,
    quaternion_from_rpy,
    state_is_healthy,
)


def _finite_float(value, default=None):
    """Parse external numeric data without ever publishing NaN or infinity."""
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        if default is None:
            raise ValueError("invalid numeric value: {!r}".format(value))
        return float(default)
    if not math.isfinite(parsed):
        if default is None:
            raise ValueError("non-finite numeric value: {!r}".format(value))
        return float(default)
    return parsed


class RealInterfaceBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.last_state_mono = 0.0
        self.last_cmd_mono = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.state_topic = rospy.get_param("~state_topic", "/fusion_location")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/odometry/imu_incremental"
        )
        self.vehicle_state_topic = rospy.get_param(
            "~vehicle_state_topic", "/vehicle/state"
        )
        self.localization_seed_topic = rospy.get_param(
            "~localization_seed_topic", "/localization/qianxun_seed"
        )
        self.publish_odom = bool(rospy.get_param("~publish_odom", True))
        self.publish_vehicle_state = bool(
            rospy.get_param("~publish_vehicle_state", True)
        )
        self.publish_localization_seed = bool(
            rospy.get_param("~publish_localization_seed", True)
        )
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.publish_map_to_odom = bool(
            rospy.get_param("~publish_map_to_odom_identity", True)
        )

        self.require_fusion = bool(rospy.get_param("~require_fusion_valid", True))
        self.require_position = bool(
            rospy.get_param("~require_position_valid", True)
        )
        self.require_control_frame = bool(
            rospy.get_param("~require_control_frame_ready", True)
        )
        self.require_known_heading = bool(
            rospy.get_param("~require_known_heading_mode", True)
        )
        self.state_timeout = float(rospy.get_param("~state_timeout", 0.5))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.max_speed = float(rospy.get_param("~max_speed", 0.8))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 0.35))
        self.command_hz = float(rospy.get_param("~command_hz", 20.0))

        acc_addr = rospy.get_param("~policy_acc_bind", "tcp://*:8095")
        steer_addr = rospy.get_param("~policy_steer_bind", "tcp://*:8092")
        brake_addr = rospy.get_param("~policy_brake_bind", "tcp://*:8096")
        self.ctx = zmq.Context()
        self.pub_acc = self.ctx.socket(zmq.PUB)
        self.pub_steer = self.ctx.socket(zmq.PUB)
        self.pub_brake = self.ctx.socket(zmq.PUB)
        self.pub_acc.bind(acc_addr)
        self.pub_steer.bind(steer_addr)
        self.pub_brake.bind(brake_addr)

        self.odom_pub = None
        if self.publish_odom:
            self.odom_pub = rospy.Publisher(
                self.odom_topic, Odometry, queue_size=20
            )
        self.vehicle_state_pub = None
        if self.publish_vehicle_state:
            self.vehicle_state_pub = rospy.Publisher(
                self.vehicle_state_topic, Odometry, queue_size=50
            )
        self.localization_seed_pub = None
        if self.publish_localization_seed:
            self.localization_seed_pub = rospy.Publisher(
                self.localization_seed_topic,
                PoseWithCovarianceStamped,
                queue_size=10,
            )
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.static_tf_pub = tf2_ros.StaticTransformBroadcaster()

        if self.publish_tf and self.publish_map_to_odom:
            self._publish_identity_map_to_odom()

        rospy.Subscriber(self.state_topic, String, self._state_callback,
                         queue_size=20)
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_callback, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.command_hz, 1.0)),
            self._command_timer,
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "real interface bridge: %s -> %s, %s -> ZMQ policy topics",
            self.state_topic, self.odom_topic, self.cmd_topic,
        )

    def _publish_identity_map_to_odom(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(transform)

    def _state_callback(self, msg):
        try:
            state = json.loads(msg.data)
            x = _finite_float(state["UTM_x"])
            y = _finite_float(state["UTM_y"])
            z = _finite_float(state.get("Z", 0.0), 0.0)
            yaw = oxyz_heading_to_ros_yaw(state["Head"])
            if not math.isfinite(yaw):
                raise ValueError("non-finite heading")
            roll = math.radians(_finite_float(state.get("Roll", 0.0), 0.0))
            pitch = math.radians(_finite_float(state.get("Pitch", 0.0), 0.0))
            gnss_position_valid = bool(
                int(state.get("GNSSPositionValid", 0))
            )
            gga_quality = int(
                state.get("GGA_quality", state.get("Status", 0))
            )
            qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(1.0, "invalid /fusion_location payload: %s", exc)
            return

        stamp = rospy.Time.now()
        healthy = state_is_healthy(
            state,
            self.require_fusion,
            self.require_position,
            self.require_control_frame,
            self.require_known_heading,
        )
        measurement_s = _finite_float(
            state.get("GNSSMeasurementTimestamp", stamp.to_sec()),
            stamp.to_sec(),
        )
        measurement_stamp = rospy.Time.from_sec(measurement_s)
        measured_x = _finite_float(state.get("ControlFrameAlignedX", x), x)
        measured_x += _finite_float(state.get("ControlFrameDx", 0.0), 0.0)
        measured_y = _finite_float(state.get("ControlFrameAlignedY", y), y)
        measured_y += _finite_float(state.get("ControlFrameDy", 0.0), 0.0)
        measured_z = _finite_float(state.get("ControlFrameAlignedZ", z), z)
        measured_yaw = oxyz_heading_to_ros_yaw(
            state.get("HeadAligned", state["Head"])
        )
        mqx, mqy, mqz, mqw = quaternion_from_rpy(
            roll, pitch, measured_yaw
        )
        with self.lock:
            self.state = state
            self.last_state_mono = time.monotonic()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = _finite_float(state.get("V_x", 0.0), 0.0)
        odom.twist.twist.linear.y = _finite_float(state.get("V_y", 0.0), 0.0)
        odom.twist.twist.linear.z = _finite_float(state.get("V_z", 0.0), 0.0)
        odom.twist.twist.angular.z = _finite_float(state.get("Omega", 0.0), 0.0)
        covariance = 0.02 if healthy else 1000.0
        odom.pose.covariance[0] = covariance
        odom.pose.covariance[7] = covariance
        odom.pose.covariance[35] = covariance
        if self.odom_pub is not None:
            self.odom_pub.publish(odom)

        # The fixed-map localizer consumes this only once at cold start.  Do
        # not publish an invalid/high-covariance seed: absence keeps its
        # localization quality gate closed instead of guessing map origin.
        seed_healthy = healthy and gnss_position_valid and gga_quality == 4
        if seed_healthy and self.localization_seed_pub is not None:
            seed = PoseWithCovarianceStamped()
            seed.header.stamp = measurement_stamp
            seed.header.frame_id = self.map_frame
            seed.pose.pose.position.x = measured_x
            seed.pose.pose.position.y = measured_y
            seed.pose.pose.position.z = measured_z
            seed.pose.pose.orientation.x = mqx
            seed.pose.pose.orientation.y = mqy
            seed.pose.pose.orientation.z = mqz
            seed.pose.pose.orientation.w = mqw
            seed.pose.covariance[0] = covariance
            seed.pose.covariance[7] = covariance
            seed.pose.covariance[35] = covariance
            self.localization_seed_pub.publish(seed)

        # Unified controller-state contract.  Unlike the compatibility odom
        # above (whose pose is propagated to publication time), this message
        # represents the GNSS/IMU-aligned measurement instant.  Its twist is
        # always expressed in child_frame_id=base_link as required by
        # nav_msgs/Odometry and by both localization adapters.
        if self.vehicle_state_pub is not None:
            vehicle_state = Odometry()
            vehicle_state.header.stamp = measurement_stamp
            vehicle_state.header.frame_id = self.odom_frame
            vehicle_state.child_frame_id = self.base_frame
            vehicle_state.pose.pose.position.x = measured_x
            vehicle_state.pose.pose.position.y = measured_y
            vehicle_state.pose.pose.position.z = measured_z
            vehicle_state.pose.pose.orientation.x = mqx
            vehicle_state.pose.pose.orientation.y = mqy
            vehicle_state.pose.pose.orientation.z = mqz
            vehicle_state.pose.pose.orientation.w = mqw
            vehicle_state.twist.twist.linear.x = _finite_float(
                state.get("V_x", 0.0), 0.0
            )
            vehicle_state.twist.twist.linear.y = _finite_float(
                state.get("V_y", 0.0), 0.0
            )
            vehicle_state.twist.twist.linear.z = _finite_float(
                state.get("V_z", 0.0), 0.0
            )
            vehicle_state.twist.twist.angular.z = _finite_float(
                state.get("OmegaAligned", state.get("Omega", 0.0)), 0.0
            )
            vehicle_state.pose.covariance[0] = covariance
            vehicle_state.pose.covariance[7] = covariance
            vehicle_state.pose.covariance[35] = covariance
            vehicle_state.twist.covariance[0] = covariance
            vehicle_state.twist.covariance[7] = covariance
            vehicle_state.twist.covariance[35] = covariance
            self.vehicle_state_pub.publish(vehicle_state)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(transform)

    def _cmd_callback(self, msg):
        with self.lock:
            self.v_cmd = clamp(msg.linear.x, self.max_speed)
            self.w_cmd = clamp(msg.angular.z, self.max_yaw_rate)
            self.last_cmd_mono = time.monotonic()

    def _command_timer(self, _event):
        now = time.monotonic()
        with self.lock:
            state = self.state
            state_age = now - self.last_state_mono
            cmd_age = now - self.last_cmd_mono
            v_cmd = self.v_cmd
            w_cmd = self.w_cmd
        healthy = (
            state_age <= self.state_timeout
            and cmd_age <= self.cmd_timeout
            and state_is_healthy(
                state,
                self.require_fusion,
                self.require_position,
                self.require_control_frame,
                self.require_known_heading,
            )
        )
        brake = 0.0 if healthy else 1.0
        if not healthy:
            v_cmd = 0.0
            w_cmd = 0.0
        self._send_policy(v_cmd, w_cmd, brake)

    def _send_policy(self, v_cmd, w_cmd, brake):
        self.pub_acc.send_string("PoliAcc {}".format(json.dumps(float(v_cmd))))
        self.pub_steer.send_string(
            "PoliSteer {}".format(json.dumps(float(w_cmd)))
        )
        self.pub_brake.send_string(
            "PoliBrake {}".format(json.dumps(float(brake)))
        )

    def shutdown(self):
        for _ in range(10):
            try:
                self._send_policy(0.0, 0.0, 1.0)
            except zmq.ZMQError:
                break
            time.sleep(0.02)
        for socket in (self.pub_acc, self.pub_steer, self.pub_brake):
            socket.close(linger=0)
        self.ctx.term()


def main():
    rospy.init_node("vehicle_interface_bridge")
    RealInterfaceBridge()
    rospy.spin()
