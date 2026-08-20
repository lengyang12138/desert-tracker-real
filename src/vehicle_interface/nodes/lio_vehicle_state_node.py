#!/usr/bin/env python3
"""Normalize LIO-SAM odometry and the shared IMU onto /vehicle/state.

Contract:
  * header.stamp is the actual LIO/IMU measurement time;
  * pose is expressed in the odometry header frame;
  * twist is expressed in child_frame_id=base_link;
  * angular.z comes from the same /imu/data stream used by Qianxun and LIO-SAM.
"""

import math

import message_filters
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LioVehicleStateAdapter:
    def __init__(self):
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/odometry/imu_incremental"
        )
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.output_topic = rospy.get_param(
            "~output_topic", "/vehicle/state"
        )
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.imu_time_offset = float(
            rospy.get_param("~imu_time_offset_s", 0.0)
        )
        self.sync_tolerance = float(
            rospy.get_param("~sync_tolerance_s", 0.002)
        )
        self.imu_yaw_rate_sign = float(
            rospy.get_param("~imu_yaw_rate_sign", 1.0)
        )
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100
        )
        self.odom_subscriber = message_filters.Subscriber(
            self.odom_topic, Odometry, queue_size=300,
        )
        self.imu_subscriber = message_filters.Subscriber(
            self.imu_topic, Imu, queue_size=500,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.odom_subscriber, self.imu_subscriber],
            queue_size=300,
            slop=abs(self.imu_time_offset) + self.sync_tolerance,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._state_callback)
        rospy.loginfo(
            "LIO vehicle-state adapter: odom=%s imu=%s -> %s",
            self.odom_topic, self.imu_topic, self.output_topic,
        )

    def _state_callback(self, msg, imu_msg):
        if msg.header.stamp.is_zero():
            rospy.logwarn_throttle(1.0, "LIO state rejected: zero odom stamp")
            return
        corrected_imu_stamp = (
            imu_msg.header.stamp + rospy.Duration(self.imu_time_offset)
        )
        if abs((msg.header.stamp - corrected_imu_stamp).to_sec()) > \
                self.sync_tolerance:
            rospy.logwarn_throttle(
                1.0, "LIO/IMU corrected timestamps do not align"
            )
            return
        omega_z = (
            float(imu_msg.angular_velocity.z) * self.imu_yaw_rate_sign
        )
        if not math.isfinite(omega_z):
            return

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        vx_world = float(msg.twist.twist.linear.x)
        vy_world = float(msg.twist.twist.linear.y)
        forward = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world
        lateral = -math.sin(yaw) * vx_world + math.cos(yaw) * vy_world
        if not all(math.isfinite(value) for value in (forward, lateral)):
            return

        state = Odometry()
        # The odometry and corrected IMU stamps are synchronized above. The
        # odometry stamp is retained because its linear velocity is the slower
        # of the two measurements and defines the complete state instant.
        state.header = msg.header
        state.child_frame_id = self.base_frame
        state.pose = msg.pose
        state.twist = msg.twist
        state.twist.twist.linear.x = forward
        state.twist.twist.linear.y = lateral
        state.twist.twist.angular.z = omega_z
        # Mark the normalized axes with finite covariance even when upstream
        # LIO-SAM leaves its covariance array at all zeros.
        state.twist.covariance[0] = max(state.twist.covariance[0], 0.02)
        state.twist.covariance[7] = max(state.twist.covariance[7], 0.05)
        state.twist.covariance[35] = max(state.twist.covariance[35], 0.01)
        self.publisher.publish(state)


def main():
    rospy.init_node("lio_vehicle_state")
    LioVehicleStateAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
