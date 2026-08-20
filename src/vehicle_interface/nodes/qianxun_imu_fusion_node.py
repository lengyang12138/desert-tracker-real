#!/usr/bin/env python3
"""Load optional map-module coordinates before importing the fusion stack."""

import os

import rospy


MAP_PARAM_ENV = {
    "origin_lat": "QX_ORIGIN_LAT",
    "origin_lon": "QX_ORIGIN_LON",
    "origin_alt": "QX_ORIGIN_ALT",
    "yaw_enu_to_oxyz_deg": "QX_YAW_ENU_TO_OXYZ_DEG",
    "map_module": "QX_MAP_MODULE",
}


def apply_map_module_params():
    for parameter, environment in MAP_PARAM_ENV.items():
        private_name = "~" + parameter
        if not rospy.has_param(private_name):
            continue
        value = rospy.get_param(private_name)
        if value is None:
            continue
        os.environ[environment] = str(value)


def run():
    # qianxun_imu_fusion imports its static configuration from environment
    # variables, so initialize ROS and apply module.yaml before that import.
    rospy.init_node("POS_qianxun_imu", anonymous=True)
    apply_map_module_params()
    from vehicle_interface.qianxun_imu_fusion import main
    main(init_ros_node=False)


if __name__ == "__main__":
    run()
