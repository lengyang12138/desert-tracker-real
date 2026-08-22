import os
import importlib.util
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class ProjectContractTests(unittest.TestCase):
    def _path(self, *parts):
        return os.path.join(ROOT, *parts)

    def test_catkin_toplevel_uses_loaded_workspace_macro(self):
        with open(self._path("src", "CMakeLists.txt"),
                  encoding="utf-8") as stream:
            cmake = stream.read()
        self.assertIn("find_package(catkin", cmake)
        self.assertIn("set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)", cmake)
        self.assertIn("catkin_workspace()", cmake)
        self.assertIn("${catkin_EXTRAS_DIR}/all.cmake", cmake)

    def test_runtime_output_directories_are_precreated(self):
        expected = (
            ("bags",),
            ("bags", "site_a"),
            ("bags", "site_b"),
            ("maps", "site_a", "raw"),
            ("maps", "site_a", "aligned"),
            ("maps", "site_a", "planner"),
            ("maps", "site_b", "raw"),
            ("maps", "site_b", "aligned"),
            ("maps", "site_b", "planner"),
            ("results", "site_a", "qianxun"),
            ("results", "site_a", "lio_sam"),
            ("results", "site_a", "slam"),
            ("results", "site_b", "qianxun"),
            ("results", "site_b", "lio_sam"),
            ("results", "site_b", "slam"),
        )
        for parts in expected:
            self.assertTrue(os.path.isdir(self._path(*parts)), parts)
            self.assertTrue(
                os.path.isfile(self._path(*(parts + (".gitkeep",)))),
                parts)

        self.assertTrue(os.path.isfile(self._path(
            "maps", "site_a", "module.yaml")))
        self.assertTrue(os.path.isfile(self._path(
            "maps", "site_b", "module.yaml")))

    def test_named_map_module_launch_contract(self):
        full = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "full_system.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in full.findall("arg")}
        self.assertEqual(args["map_module"], "site_a")
        self.assertIn(
            "/maps/$(arg map_module_dir)/planner", args["map_dir"])
        self.assertIn(
            "/results/$(arg map_module_dir)/$(arg localization_mode)",
            args["evaluation_output_dir"])
        self.assertIn(
            "/maps/$(arg map_module_dir)/aligned/GlobalMap.pcd",
            args["fixed_map_pcd"])
        text = ET.tostring(full, encoding="unicode")
        self.assertIn('name="map_config_file"', text)
        self.assertIn('name="use_map_config" value="true"', text)
        self.assertIn('name="map_module" value="$(arg map_module_dir)"',
                      text)

        navigation = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "navigation.launch"
        )).getroot()
        guard = next(node for node in navigation.findall("node")
                     if node.get("name") == "map_module_guard")
        self.assertEqual(guard.get("required"), "true")
        self.assertIn("map_module.py", guard.get("type"))
        self.assertIn("--stage $(arg validation_stage)", guard.get("args"))

        record = ET.parse(self._path(
            "src", "vehicle_bringup", "launch",
            "record_mapping_bag.launch"
        )).getroot()
        record_args = {node.get("name"): node.get("default")
                       for node in record.findall("arg")}
        self.assertIn(
            "/bags/$(arg map_module_dir)/mapping_run",
            record_args["output_prefix"])

    def test_map_module_tool_creates_and_validates_isolated_layout(self):
        script_path = self._path(
            "src", "terrain_map_builder", "scripts", "map_module.py")
        spec = importlib.util.spec_from_file_location(
            "terrain_map_module_test", script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as temporary:
            root = os.path.abspath(temporary)
            create_args = SimpleNamespace(
                force=False, origin_lat="22.3", origin_lon="114.1",
                origin_alt="8.0", yaw_enu_to_oxyz_deg="17.5")
            module.create_module(root, "dune_a", create_args)
            for relative in (
                ("maps", "dune_a", "module.yaml"),
                ("maps", "dune_a", "raw"),
                ("maps", "dune_a", "aligned"),
                ("maps", "dune_a", "planner"),
                ("bags", "dune_a"),
                ("results", "dune_a", "qianxun"),
                ("results", "dune_a", "lio_sam"),
                ("results", "dune_a", "slam"),
            ):
                self.assertTrue(os.path.exists(os.path.join(root, *relative)))
            for relative in (
                ("bags", "dune_a", ".gitkeep"),
                ("results", "dune_a", "qianxun", ".gitkeep"),
                ("results", "dune_a", "lio_sam", ".gitkeep"),
                ("results", "dune_a", "slam", ".gitkeep"),
            ):
                self.assertTrue(os.path.isfile(os.path.join(root, *relative)))

            planner = os.path.join(root, "maps", "dune_a", "planner")
            for filename in module.PLANNER_FILES:
                with open(os.path.join(planner, filename), "wb") as stream:
                    stream.write(b"map")
            aligned = os.path.join(
                root, "maps", "dune_a", "aligned", "GlobalMap.pcd")
            with open(aligned, "wb") as stream:
                stream.write(b"pcd")
            module.validate_module(root, "dune_a", "qianxun")
            module.validate_module(root, "dune_a", "lio_sam")
            identity = module.map_identity(root, "dune_a")
            self.assertEqual(identity["map_module"], "dune_a")
            self.assertEqual(len(identity["fingerprint"]), 64)
            other_planner = os.path.join(root, "maps", "other", "planner")
            with self.assertRaises(RuntimeError):
                module.validate_module(
                    root, "dune_a", "planner", other_planner)
            with self.assertRaises(ValueError):
                module.validate_name("../escape")

    def test_evaluator_records_map_identity_in_every_run(self):
        with open(self._path(
            "src", "navigation_evaluation", "nodes",
            "real_nav_evaluator.py"
        ), encoding="utf-8") as stream:
            evaluator = stream.read()
        for contract in (
            "self.map_module = rospy.get_param('~map_module'",
            "self.map_identity = self._build_map_identity",
            "'map': self.map_identity",
            "'map_fingerprint': self.map_identity['fingerprint']",
            "'localization_mode': self.localization_mode",
        ):
            self.assertIn(contract, evaluator)

        launch = ET.parse(self._path(
            "src", "navigation_evaluation", "launch", "evaluation.launch"
        )).getroot()
        text = ET.tostring(launch, encoding="unicode")
        self.assertIn('name="map_module"', text)
        self.assertIn('name="map_dir" value="$(arg map_dir)"', text)
        self.assertIn("results/$(arg map_module_dir)/$(arg localization_mode)",
                      text)
        args = {node.get("name"): node.get("default")
                for node in launch.findall("arg")}
        self.assertIn("maps/$(arg map_module_dir)/planner", args["map_dir"])

    def test_map_module_coordinates_load_before_fusion_import(self):
        with open(self._path(
            "src", "vehicle_interface", "nodes",
            "qianxun_imu_fusion_node.py"
        ), encoding="utf-8") as stream:
            wrapper = stream.read()
        self.assertLess(
            wrapper.index("apply_map_module_params()"),
            wrapper.index("from vehicle_interface.qianxun_imu_fusion import main"))
        for environment in (
            "QX_ORIGIN_LAT", "QX_ORIGIN_LON", "QX_ORIGIN_ALT",
            "QX_YAW_ENU_TO_OXYZ_DEG"):
            self.assertIn(environment, wrapper)

    def test_lio_save_map_accepts_module_absolute_path(self):
        with open(self._path(
            "src", "LIO-SAM", "src", "mapOptmization.cpp"
        ), encoding="utf-8") as stream:
            implementation = stream.read()
        self.assertIn("req.destination.front() == '/'", implementation)
        self.assertIn('shellQuote(saveMapDirectory)', implementation)
        self.assertIn('saveMapDirectory == "/"', implementation)

    def test_functional_package_names_and_portable_launch_paths(self):
        expected_packages = {
            "navigation", "vehicle_interface", "pointcloud_preprocess",
            "terrain_map_builder", "fixed_map_localization",
            "navigation_evaluation", "slam_evaluation",
            "vehicle_bringup", "vehicle_description", "lio_sam",
        }
        actual_packages = set()
        for directory, _, files in os.walk(self._path("src")):
            if "package.xml" in files:
                package = ET.parse(
                    os.path.join(directory, "package.xml")).getroot()
                actual_packages.add(package.findtext("name"))
        self.assertEqual(actual_packages, expected_packages)

        old_prefix = "desert_tracker_"
        for directory, _, files in os.walk(self._path("src")):
            for filename in files:
                if not filename.endswith(".launch"):
                    continue
                launch_path = os.path.join(directory, filename)
                with open(launch_path, encoding="utf-8") as stream:
                    launch_text = stream.read()
                self.assertNotIn(old_prefix, launch_text, launch_path)
                self.assertNotIn("$(env HOME)", launch_text, launch_path)
                self.assertNotRegex(
                    launch_text, r"[A-Za-z]:[\\/]", launch_path)

    def test_localization_switch_defaults_to_qianxun(self):
        launch = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "full_system.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in launch.findall("arg")}
        self.assertEqual(args["localization_mode"], "qianxun")
        text = ET.tostring(launch, encoding="unicode")
        self.assertIn("localization_mode == 'qianxun'", text)
        self.assertIn("localization_mode == 'lio_sam'", text)

    def test_no_live_lidar_obstacle_layer(self):
        with open(self._path(
            "src", "vehicle_bringup", "config", "local_costmap.yaml"
        ), encoding="utf-8") as stream:
            config = stream.read()
        self.assertNotIn("ObstacleLayer", config)
        self.assertNotIn("points_raw", config)

    def test_real_evaluator_has_no_gazebo_message_dependency(self):
        with open(self._path(
            "src", "navigation_evaluation", "nodes",
            "real_nav_evaluator.py"
        ), encoding="utf-8") as stream:
            evaluator = stream.read()
        self.assertNotIn("from gazebo_msgs", evaluator)
        self.assertNotIn("/gazebo/model_states", evaluator)
        self.assertIn("/fusion_location", evaluator)

    def test_real_evaluator_records_low_bandwidth_sensor_diagnostics(self):
        with open(self._path(
            "src", "navigation_evaluation", "nodes",
            "real_nav_evaluator.py"
        ), encoding="utf-8") as stream:
            evaluator = stream.read()
        for contract in (
            "rospy.Subscriber(imu_topic, Imu",
            "rospy.Subscriber(motor_feedback_topic, String",
            "imu_raw.csv",
            "gps_antenna_measurements.csv",
            "motor_feedback_raw.csv",
            "imu_diagnostics.png",
            "gps_diagnostics.png",
            "motor_feedback.png",
        ):
            self.assertIn(contract, evaluator)
        self.assertNotIn("PointCloud2", evaluator)
        self.assertNotIn("/points_raw", evaluator)

    def test_real_vehicle_watchdogs_and_pivot_turn_contract(self):
        with open(self._path(
            "src", "navigation", "src", "mpc_local_planner.cpp"
        ), encoding="utf-8") as stream:
            controller = stream.read()
        for contract in (
            "controlInputsHealthy", "state_timeout", "tf_timeout",
            "max_position_covariance", "max_yaw_covariance",
            "max_control_cycle_interval", "solver_time_limit",
            "projection_segment_cache_", "reusing previous ",
            "repeated solver timeouts",
        ):
            self.assertIn(contract, controller)

        with open(self._path(
            "src", "vehicle_interface", "launch", "io.launch"
        ), encoding="utf-8") as stream:
            io_launch = stream.read()
        self.assertNotIn("--zero-steer-when-stopped", io_launch)

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "can_bridge.py"
        ), encoding="utf-8") as stream:
            can_bridge = stream.read()
        self.assertIn("There is no verified physical-brake field", can_bridge)
        self.assertIn("REQUIRED_POLICY_TOPICS", can_bridge)
        self.assertIn('"topic_times"', can_bridge)
        self.assertIn("missing_topics", can_bridge)
        self.assertIn("stale_topics", can_bridge)

        with open(self._path(
            "src", "fixed_map_localization", "src",
            "fixed_map_localizer.cpp"
        ), encoding="utf-8") as stream:
            localizer = stream.read()
        self.assertIn('"/localization/quality_ok"', localizer)
        self.assertIn("last_trusted_match_time_", localizer)

        # Fixed-map localization must cold-start from a fresh, healthy
        # Qianxun absolute pose rather than assuming map origin.
        self.assertIn('"/localization/qianxun_seed"', localizer)
        self.assertIn("require_qianxun_initialization_", localizer)
        self.assertIn("initial_pose_timeout_", localizer)
        self.assertIn("initialPoseCallback", localizer)

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "real_interface_bridge.py"
        ), encoding="utf-8") as stream:
            interface_bridge = stream.read()
        self.assertIn("PoseWithCovarianceStamped", interface_bridge)
        self.assertIn('"/localization/qianxun_seed"', interface_bridge)
        self.assertIn("seed_healthy = healthy and gnss_position_valid", interface_bridge)
        self.assertIn("gga_quality == 4", interface_bridge)
        self.assertIn('state.get("HeadAligned", state["Head"])', interface_bridge)

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "can_bridge.py"
        ), encoding="utf-8") as stream:
            can_bridge = stream.read()
        self.assertIn('"/motor_feedback", String', can_bridge)
        self.assertIn('"/joint_states", JointState', can_bridge)
        self.assertIn('"raw_left": int(raw_l)', can_bridge)

    def test_lidar_converter_preserves_native_timing_on_contract_topics(self):
        launch = ET.parse(self._path(
            "src", "pointcloud_preprocess", "launch", "points_raw.launch"
        )).getroot()
        text = ET.tostring(launch, encoding="unicode")
        self.assertIn('name="input_topic" default="/rslidar_points"', text)
        self.assertIn('name="output_topic" default="/points_raw"', text)
        self.assertIn('type="rs_to_lio_sam"', text)
        self.assertIn('type="rslidar_sdk_node"', text)
        self.assertNotIn("topic_tools", text)
        self.assertNotIn('name="convert"', text)

    def test_sensor_static_tf_tree_is_connected_without_replay_duplicates(self):
        io = ET.parse(self._path(
            "src", "vehicle_interface", "launch", "io.launch"
        )).getroot()
        io_args = {node.get("name"): node.get("default")
                   for node in io.findall("arg")}
        self.assertEqual(io_args["imu_xyz"], "0 0 0")
        self.assertEqual(io_args["imu_ypr"], "0 0 0")
        imu_tf = next(node for node in io.findall("node")
                      if node.get("name") == "base_to_imu_tf")
        self.assertEqual(imu_tf.get("if"), "$(arg publish_imu_tf)")
        self.assertEqual(
            imu_tf.get("args"),
            "$(arg imu_xyz) $(arg imu_ypr) base_link imu_link")

        points = ET.parse(self._path(
            "src", "pointcloud_preprocess", "launch", "points_raw.launch"
        )).getroot()
        lidar_tf = next(node for node in points.findall("node")
                        if node.get("name") == "base_to_velodyne_tf")
        self.assertEqual(lidar_tf.get("if"), "$(arg publish_lidar_tf)")
        self.assertEqual(
            lidar_tf.get("args"),
            "$(arg lidar_xyz) $(arg lidar_ypr) base_link velodyne")

        replay = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "bag_mapping.launch"
        )).getroot()
        replay_args = {node.get("name"): node.get("default")
                       for node in replay.findall("arg")}
        self.assertEqual(replay_args["bag_has_static_tf"], "true")
        replay_text = ET.tostring(replay, encoding="unicode")
        self.assertIn('name="publish_lidar_tf"', replay_text)
        self.assertIn("arg('bag_has_static_tf') != 'true'", replay_text)

    def test_real_vehicle_entry_launches_hide_fixed_hardware_parameters(self):
        config = ET.parse(self._path(
            "src", "vehicle_bringup", "config", "real_vehicle.launch"
        )).getroot()
        config_args = {node.get("name"): node.get("default")
                       for node in config.findall("arg")}
        for required in (
                "gnss_port", "imu_port", "imu_xyz", "imu_ypr",
                "lidar_xyz", "lidar_ypr", "lever_x", "lever_y", "lever_z",
                "can_iface", "speed_sign", "steer_sign", "feedback_sign"):
            self.assertIn(required, config_args)

        entries = {
            "real_calibration.launch": set(),
            "real_localization_check.launch": {"project_root", "map_module"},
            "real_navigation.launch": {"project_root", "map_module",
                                       "localization_mode"},
            "real_navigation_only.launch": {"project_root", "map_module"},
            "real_mapping.launch": {"project_root", "map_module"},
            "real_mapping_record.launch": {"project_root", "map_module"},
            "offline_mapping.launch": set(),
        }
        for filename, expected_args in entries.items():
            root = ET.parse(self._path(
                "src", "vehicle_bringup", "launch", filename
            )).getroot()
            actual_args = {node.get("name") for node in root.findall("arg")}
            self.assertEqual(actual_args, expected_args)
            text = ET.tostring(root, encoding="unicode")
            self.assertIn("config/real_vehicle.launch", text)
            for hidden in ("imu_xyz", "lidar_xyz", "gnss_port", "lever_x"):
                self.assertNotIn('name="{}"'.format(hidden), text)

    def test_dynamic_tf_publishers_have_one_owner_in_each_runtime_mode(self):
        full = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "full_system.launch"
        )).getroot()
        full_text = ET.tostring(full, encoding="unicode")
        self.assertIn(
            'name="bridge_publish_tf" '
            'value="$(eval localization_mode == \'qianxun\')"',
            full_text)
        self.assertIn(
            'name="bridge_publish_map_to_odom" '
            'value="$(eval localization_mode == \'qianxun\')"',
            full_text)

        lio_localization = ET.parse(self._path(
            "src", "vehicle_bringup", "launch",
            "lio_sam_localization.launch"
        )).getroot()
        localization_text = ET.tostring(
            lio_localization, encoding="unicode")
        self.assertIn(
            'name="publish_map_to_odom_tf" value="false"',
            localization_text)
        self.assertIn(
            'fixed_map_localization.launch', localization_text)

        live_mapping = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "mapping.launch"
        )).getroot()
        mapping_text = ET.tostring(live_mapping, encoding="unicode")
        self.assertIn('name="bridge_publish_tf" value="false"',
                      mapping_text)
        self.assertIn('name="bridge_publish_map_to_odom" value="false"',
                      mapping_text)
        self.assertNotIn(
            'name="publish_map_to_odom_tf" value="false"', mapping_text)

    def test_mapping_bag_records_processed_lidar_and_shared_imu(self):
        launch = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "record_mapping_bag.launch"
        )).getroot()
        text = ET.tostring(launch, encoding="unicode")
        self.assertIn('name="input_topic" value="/rslidar_points"', text)
        self.assertIn('name="output_topic" value="/points_raw"', text)
        self.assertIn('name="start_can" value="false"', text)
        self.assertIn('type="mapping_bag_recorder"', text)
        self.assertIn('name="warmup_seconds" value="$(arg warmup_seconds)"',
                      text)
        self.assertIn('name="points_topic" value="/points_raw"', text)
        self.assertIn('name="imu_topic" value="/imu/data"', text)

        with open(self._path(
            "src", "vehicle_bringup", "src", "mapping_bag_recorder.cpp"
        ), encoding="utf-8") as stream:
            recorder = stream.read()
        self.assertIn("ros::WallDuration(warmup_seconds).sleep()", recorder)
        self.assertIn("point_cloud_received && imu_received", recorder)
        self.assertIn('"rosbag", "record", "--lz4"', recorder)
        self.assertIn(
            'points_topic, imu_topic, "/fusion_location", "/tf", "/tf_static"',
            recorder)

        replay = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "bag_mapping.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in replay.findall("arg")}
        self.assertEqual(args["bag_point_topic"], "/points_raw")
        self.assertEqual(args["bag_has_processed_points"], "true")

        with open(self._path(
            "src", "pointcloud_preprocess", "src", "rs_to_lio_sam.cpp"
        ), encoding="utf-8") as stream:
            converter = stream.read()
        self.assertIn('findField(*input, "time")', converter)
        self.assertIn('findField(*input, "timestamp")', converter)
        self.assertIn("PointField::FLOAT32", converter)
        self.assertIn("PointField::FLOAT64", converter)

    def test_lio_sam_keeps_controller_odometry_topic(self):
        launch = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "lio_sam_mapping.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in launch.findall("arg")}
        self.assertEqual(args["odom_topic_base"], "/odometry/imu")
        text = ET.tostring(launch, encoding="unicode")
        self.assertIn("lio_sam/odomTopic", text)
        self.assertIn("lio_sam/imuTimeOffset", text)

    def test_unified_measurement_time_vehicle_state_contract(self):
        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "real_interface_bridge.py"
        ), encoding="utf-8") as stream:
            bridge = stream.read()
        self.assertIn('"~vehicle_state_topic", "/vehicle/state"', bridge)
        self.assertIn('state.get("GNSSMeasurementTimestamp"', bridge)
        self.assertIn('vehicle_state.child_frame_id = self.base_frame', bridge)
        self.assertIn('state.get("OmegaAligned"', bridge)

        with open(self._path(
            "src", "vehicle_interface", "nodes",
            "lio_vehicle_state_node.py"
        ), encoding="utf-8") as stream:
            adapter = stream.read()
        self.assertIn('self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")',
                      adapter)
        self.assertIn('message_filters.ApproximateTimeSynchronizer', adapter)
        self.assertIn('imu_msg.header.stamp + rospy.Duration(self.imu_time_offset)',
                      adapter)
        self.assertIn('state.twist.twist.linear.x = forward', adapter)
        self.assertIn('state.twist.twist.angular.z = omega_z', adapter)

        with open(self._path(
            "src", "navigation", "src", "mpc_local_planner.cpp"
        ), encoding="utf-8") as stream:
            controller = stream.read()
        self.assertIn('nh.param<std::string>("odom_topic", odom_topic, "/vehicle/state")',
                      controller)
        self.assertIn('const double forward_v = msg->twist.twist.linear.x;',
                      controller)
        self.assertNotIn('const double forward_v = std::cos(yaw)', controller)
        self.assertIn('commandAtMeasurement(measurement_stamp', controller)
        self.assertIn('measurement_stamp <= last_response_measurement_stamp_',
                      controller)
        self.assertIn('signed_cte - compared_predicted_cte', controller)
        self.assertIn('-cte_prediction_residual /', controller)

    def test_vehicle_lidar_calibration_is_not_devboard_calibration(self):
        with open(self._path(
            "src", "LIO-SAM", "config", "params.yaml"
        ), encoding="utf-8") as stream:
            config = stream.read()
        self.assertIn("N_SCAN: 32", config)
        self.assertIn("imuTimeOffset: 0.0", config)
        self.assertIn("extrinsicTrans: [0.0, 0.0, 0.0]", config)
        self.assertNotIn("0.045204", config)

        with open(self._path(
            "src", "pointcloud_preprocess", "config", "rslidar.yaml"
        ), encoding="utf-8") as stream:
            lidar = stream.read()
        self.assertIn("lidar_type: RSHELIOS", lidar)
        self.assertIn("ros_send_point_cloud_topic: /rslidar_points", lidar)
        self.assertIn("use_lidar_clock: false", lidar)
        self.assertIn("ros_frame_id: velodyne", lidar)
        self.assertIn("max_distance: 150.0", lidar)

    def test_one_shared_imu_publisher_for_both_localizers(self):
        full = ET.parse(self._path(
            "src", "vehicle_bringup", "launch", "full_system.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in full.findall("arg")}
        self.assertEqual(args["imu_data_topic"], "/imu/data")
        full_text = ET.tostring(full, encoding="unicode")
        self.assertIn('name="imu_data_topic"', full_text)
        self.assertIn('name="imu_topic" value="$(arg imu_data_topic)"',
                      full_text)

        lio = ET.parse(self._path(
            "src", "vehicle_bringup", "launch",
            "lio_sam_localization.launch"
        )).getroot()
        # LIO localization must not launch another physical or ROS IMU driver.
        lio_text = ET.tostring(lio, encoding="unicode")
        self.assertNotIn("yis525", lio_text.lower())
        self.assertNotIn("real_interface_bridge", lio_text)

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "real_interface_bridge.py"
        ), encoding="utf-8") as stream:
            bridge = stream.read()
        self.assertNotIn("Publisher(self.imu_topic, Imu", bridge)

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "yis525.py"
        ), encoding="utf-8") as stream:
            yis = stream.read()
        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "qianxun_imu_fusion.py"
        ), encoding="utf-8") as stream:
            fusion = stream.read()
        self.assertIn("rospy.Publisher(Cfg.ROS_TOPIC, Imu", yis)
        self.assertIn("FusionCfg.IMU_TOPIC, sensor_msgs.msg.Imu", fusion)
        self.assertNotIn("def imu_loop", fusion)
        self.assertIn('"OriginLat": base.get("OriginLat")', fusion)
        self.assertIn('"YawEnuToOxyzDeg": base.get(', fusion)

    def test_vehicle_imu_defaults_are_retained(self):
        launch = ET.parse(self._path(
            "src", "vehicle_interface", "launch", "io.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in launch.findall("arg")}
        self.assertEqual(args["imu_port"], "/dev/ttyUSB1")
        self.assertEqual(args["imu_baud"], "460800")
        self.assertEqual(args["imu_heading_mode"], "AHRS")
        self.assertEqual(args["imu_true_north_offset_deg"], "181.6")
        self.assertEqual(args["imu_pitch_sign"], "-1")
        self.assertEqual(args["imu_roll_zero_deg"], "3.6")
        self.assertEqual(args["imu_pitch_zero_deg"], "3.8")
        self.assertEqual(args["lever_x"], "-0.114")
        self.assertEqual(args["lever_z"], "-0.970")

    def test_qianxun_timing_faults_fail_closed(self):
        launch = ET.parse(self._path(
            "src", "vehicle_interface", "launch", "io.launch"
        )).getroot()
        args = {node.get("name"): node.get("default")
                for node in launch.findall("arg")}
        self.assertEqual(args["imu_alignment_max_error_s"], "0.06")
        self.assertEqual(args["gnss_clock_max_residual_s"], "0.30")

        fusion_node = next(
            node for node in launch.findall(".//node")
            if node.get("name") == "qianxun_imu_fusion"
        )
        env = {node.get("name"): node.get("value")
               for node in fusion_node.findall("env")}
        self.assertEqual(
            env["QX_IMU_ALIGNMENT_MAX_ERROR_S"],
            "$(arg imu_alignment_max_error_s)",
        )
        self.assertEqual(
            env["QX_GNSS_CLOCK_MAX_RESIDUAL_S"],
            "$(arg gnss_clock_max_residual_s)",
        )
        self.assertEqual(env["QX_REQUIRE_GNSS_CLOCK_CONSISTENCY"], "1")

        with open(self._path(
            "src", "vehicle_interface", "src", "vehicle_interface",
            "qianxun_imu_fusion.py"
        ), encoding="utf-8") as stream:
            fusion = stream.read()
        self.assertIn(
            "fusion_valid = bool(imu_valid and gnss_valid and alignment_valid)",
            fusion,
        )
        self.assertIn('"GNSSClockConsistent": int(gnss_clock_consistent)', fusion)
        self.assertIn('"IMUAlignmentValid": int(alignment_valid)', fusion)


if __name__ == "__main__":
    unittest.main()
