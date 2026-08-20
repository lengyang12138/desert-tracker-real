#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigation path quality evaluator.

Automatically generates evaluation plots after each navigation run:
  1. Top-down: planned path vs actual trajectory on costmap
  2. Elevation profile along path
  3. Pitch & Roll terrain angles
  4. Cross-track error over time

Each run is saved under a map-module and localization-mode-specific directory
(for example results/dune_a/qianxun/nav_1).  Every JSON/CSV manifest also
records the active module and a fingerprint of the exact planner-map files.
"""

import rospy
import numpy as np
import struct
import os
import json
import csv
import hashlib
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.ticker import MultipleLocator
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers '3d' projection

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tft


class NavEvaluator(object):
    MPC_DIAGNOSTIC_FIELDS = [
        'stamp_s', 'signed_cte_m', 'path_heading_rad',
        'heading_error_rad', 'curvature_1pm', 'omega_ff_radps',
        'linear_cmd_mps', 'omega_cmd_radps',
        'previous_omega_cmd_radps', 'measured_forward_mps',
        'odom_lateral_mps', 'measured_omega_radps',
        'observed_omega_gain', 'model_omega_gain',
        'filtered_cte_rate_mps', 'abs_cte_growth_rate_mps',
        'lateral_residual_mps', 'lateral_estimate_mps',
        'cte_heading_ref_rad', 'disturbance_heading_ref_rad',
        'rate_heading_ref_rad', 'assist_omega_radps',
        'assist_latched', 'assist_active', 'selected_cost',
        'future_omega_radps', 'predicted_v_next_mps',
        'predicted_omega_next_radps', 'predicted_x_next_m',
        'predicted_y_next_m', 'predicted_yaw_next_rad',
        'reachable_omega_min_radps', 'reachable_omega_max_radps',
        'linear_reference_mps', 'delta_v_mpc_mps',
        'delta_omega_mpc_radps', 'model_linear_gain',
        'model_tau_v_s', 'model_tau_omega_s',
        'reachable_v_min_mps', 'reachable_v_max_mps',
        'predicted_cte_at_measurement_m', 'cte_prediction_residual_m',
        'response_measurement_stamp_s', 'aligned_linear_cmd_mps',
        'aligned_omega_cmd_radps', 'command_alignment_age_s',
        'cte_prediction_residual_valid',
        'state_age_s', 'tf_age_s', 'control_cycle_interval_s',
        'solver_duration_ms', 'solver_timed_out',
        'evaluated_candidates', 'projection_cache_segment',
    ]

    def __init__(self):
        rospy.init_node('nav_evaluator')

        ros_home = os.environ.get(
            'ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
        evaluator_home = os.path.join(ros_home, 'navigation_evaluation')
        self.output_dir = rospy.get_param('~output_dir',
            os.path.join(evaluator_home, 'results', 'qianxun'))
        self.localization_mode = rospy.get_param(
            '~localization_mode', 'qianxun')
        self.map_module = rospy.get_param('~map_module', 'site_a')
        self.map_dir = os.path.realpath(os.path.abspath(rospy.get_param(
            '~map_dir', os.path.join(evaluator_home, 'maps')))
        )
        dem_file = rospy.get_param('~dem_file',
            os.path.join(evaluator_home, 'maps', 'elevation_map.dem'))
        pgm_file = rospy.get_param('~pgm_file',
            os.path.join(evaluator_home, 'maps', 'traversability_map.pgm'))
        yaml_file = rospy.get_param('~yaml_file',
            os.path.join(evaluator_home, 'maps', 'traversability_map.yaml'))
        obstacle_file = rospy.get_param(
            '~obstacle_file',
            os.path.join(os.path.dirname(pgm_file), 'obstacle_map.pgm'))
        terrain_cost_file = rospy.get_param(
            '~terrain_cost_file',
            os.path.join(self.map_dir, 'terrain_cost.bin'))
        terrain_cost_coarse_file = rospy.get_param(
            '~terrain_cost_coarse_file',
            os.path.join(self.map_dir, 'terrain_cost_coarse.bin'))
        module_dir = os.path.dirname(self.map_dir)
        actual_module = os.path.basename(module_dir)
        if actual_module != self.map_module:
            raise RuntimeError(
                'map module/path mismatch: map_module={!r}, map_dir={!r}'
                .format(self.map_module, self.map_dir))
        self.map_identity = self._build_map_identity({
            'module_config': os.path.join(module_dir, 'module.yaml'),
            'traversability_map.yaml': yaml_file,
            'traversability_map.pgm': pgm_file,
            'elevation_map.dem': dem_file,
            'obstacle_map.pgm': obstacle_file,
            'terrain_cost.bin': terrain_cost_file,
            'terrain_cost_coarse.bin': terrain_cost_coarse_file,
            'aligned_pcd': os.path.join(
                module_dir, 'aligned', 'GlobalMap.pcd'),
        })

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.map_data = None
        self.obstacle_data = None
        self.dem = None
        self._load_map(pgm_file, yaml_file)
        self._load_dem(dem_file)
        self._load_obstacle_map(obstacle_file)

        self.planned_path = None
        self.coarse_paths = []
        self.corridor_paths = []
        self.fine_candidate_paths = []
        self.pareto_candidates = []
        self.pareto_summary = {}
        self.planner_statistics = []
        self.planner_stage_batches = {}
        self.plan_stamp_ns = None
        self.actual_traj = []
        self.control_traj = []
        self.ndt_target_traj = []
        self.cmd_vel_traj = []
        self.mpc_cmd_vel_traj = []
        self.mpc_diagnostics = []
        self.mpc_prediction_horizons = []
        self.wheel_state_traj = []
        self.wheel_radius = max(
            1e-4, float(rospy.get_param('~wheel_radius', 0.181695)))
        self.wheel_separation = max(
            1e-4, float(rospy.get_param('~wheel_separation', 1.27665)))
        # Real CAN feedback is normalized by feedback_sign at the hardware
        # boundary, so positive means forward in this evaluator.
        self.wheel_velocity_sign = float(rospy.get_param(
            '~wheel_velocity_sign', 1.0))
        self.nav_active = False
        self.shutdown_plot_started = False
        self.start_time = None
        self.goal_pose = None
        existing = [d for d in os.listdir(self.output_dir)
                    if d.startswith('nav_') and d[4:].isdigit()]
        self.run_count = max((int(d[4:]) for d in existing), default=0)
        self.last_status = None
        self.vehicle_half_width = 0.64
        self.gt_traj = []
        self.gt_twist_traj = []
        self.reference_state_records = []
        self.vehicle_sensor_records = []
        self.imu_raw_records = []
        self.motor_feedback_records = []
        self.last_gt_time = 0.0
        self.latest_gt_pose = None
        self.gt_frame_anchor = None
        self.gt_anchor_attempted = False
        # Keep ground-truth samples even while navigation is inactive.  The
        # base-aligned map uses the initial base_link pose as its origin, so
        # the absolute world->map anchor must be estimated from stationary GT
        # alone rather than from the localization result being evaluated.
        self.gt_anchor_buffer = deque(maxlen=2000)
        self.gt_anchor_window = max(
            0.5, float(rospy.get_param('~gt_anchor_window', 2.0)))
        self.gt_anchor_min_samples = max(
            5, int(rospy.get_param('~gt_anchor_min_samples', 20)))
        self.gt_anchor_max_position_span = max(
            0.0, float(rospy.get_param(
                '~gt_anchor_max_position_span', 0.03)))
        self.gt_anchor_max_yaw_span = np.radians(max(
            0.0, float(rospy.get_param(
                '~gt_anchor_max_yaw_span_deg', 0.3))))
        self.progress_period = max(
            0.5, float(rospy.get_param('~progress_period', 2.0)))
        self.cte_plot_min_half_range = max(
            0.5, float(rospy.get_param('~cte_plot_min_half_range', 0.5)))
        self.align_gt_to_control_frame = bool(rospy.get_param(
            '~align_gt_to_control_frame', False))
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal,
                         self._action_goal_cb)
        rospy.Subscriber('/hybrid_astar/plan', Path, self._plan_cb)
        rospy.Subscriber('/hybrid_astar/coarse_paths', MarkerArray,
                         self._coarse_paths_cb)
        rospy.Subscriber('/hybrid_astar/corridors', MarkerArray,
                         self._corridors_cb)
        rospy.Subscriber('/hybrid_astar/fine_candidates', MarkerArray,
                         self._fine_candidates_cb)
        rospy.Subscriber('/hybrid_astar/pareto_decision', MarkerArray,
                         self._pareto_decision_cb)
        rospy.Subscriber('/hybrid_astar/planner_statistics', String,
                         self._planner_statistics_cb)
        mapping_odom_topic = rospy.get_param(
            '~mapping_odom_topic', '/odometry/imu_incremental')
        control_odom_topic = rospy.get_param(
            '~control_odom_topic', '/odometry/imu_incremental')
        ndt_target_odom_topic = rospy.get_param(
            '~ndt_target_odom_topic',
            '/fixed_map_localizer/target_odometry')
        rospy.Subscriber(mapping_odom_topic, Odometry, self._odom_cb)
        # Same fused pose used to publish odom->base_link TF.  move_base/MPC
        # obtains robot pose from that TF, so this is the controller's view.
        rospy.Subscriber(control_odom_topic, Odometry, self._control_odom_cb)
        rospy.Subscriber(ndt_target_odom_topic, Odometry,
                         self._ndt_target_odom_cb)
        applied_cmd_topic = rospy.get_param('~applied_cmd_topic', '/cmd_vel')
        mpc_cmd_topic = rospy.get_param('~mpc_cmd_topic', '/cmd_vel')
        rospy.Subscriber(applied_cmd_topic, Twist, self._cmd_vel_cb)
        rospy.Subscriber(mpc_cmd_topic, Twist, self._mpc_cmd_vel_cb)
        rospy.Subscriber('/move_base/MpcLocalPlanner/diagnostics',
                         Float64MultiArray, self._mpc_diagnostics_cb)
        rospy.Subscriber('/move_base/MpcLocalPlanner/prediction_diagnostics',
                         Float64MultiArray,
                         self._mpc_prediction_diagnostics_cb)
        joint_states_topic = rospy.get_param(
            '~joint_states_topic', '/joint_states')
        rospy.Subscriber(joint_states_topic, JointState,
                         self._joint_states_cb)
        imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        motor_feedback_topic = rospy.get_param(
            '~motor_feedback_topic', '/motor_feedback')
        rospy.Subscriber(imu_topic, Imu, self._imu_cb, queue_size=500)
        rospy.Subscriber(motor_feedback_topic, String,
                         self._motor_feedback_cb, queue_size=200)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self._status_cb)
        reference_state_topic = rospy.get_param(
            '~reference_state_topic', '/bus/location')
        rospy.Subscriber(reference_state_topic, String, self._reference_cb)
        vehicle_sensor_topic = rospy.get_param(
            '~vehicle_sensor_topic', '/bus/sensor')
        rospy.Subscriber(vehicle_sensor_topic, String,
                         self._vehicle_sensor_cb)
        self.nav_output_pub = rospy.Publisher(
            '/navigation/progress', String, queue_size=20, latch=True)
        self.progress_timer = rospy.Timer(
            rospy.Duration(self.progress_period), self._progress_cb)
        rospy.on_shutdown(self._shutdown_cb)

        rospy.loginfo(
            'Real NavEvaluator ready. map=%s fingerprint=%s mode=%s '
            'output=%s mapping=%s control=%s reference=%s',
            self.map_module, self.map_identity['fingerprint'][:16],
            self.localization_mode, self.output_dir, mapping_odom_topic,
            control_odom_topic, reference_state_topic)

    def _nav_output(self, fmt, *args):
        message = fmt % args if args else fmt
        rospy.loginfo(message)
        self.nav_output_pub.publish(String(data=message))

    @staticmethod
    def _quick_file_identity(path):
        """Hash complete normal files and bounded samples of very large maps."""
        record = {'path': os.path.abspath(path)}
        if not os.path.isfile(path):
            record['missing'] = True
            return record
        size = os.path.getsize(path)
        digest = hashlib.sha256()
        digest.update(str(size).encode('ascii'))
        with open(path, 'rb') as stream:
            if size <= 64 * 1024 * 1024:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
                method = 'sha256-full'
            else:
                digest.update(stream.read(1024 * 1024))
                stream.seek(max(0, size - 1024 * 1024))
                digest.update(stream.read(1024 * 1024))
                method = 'sha256-size-first-last-1MiB'
        record.update({
            'size': int(size),
            'digest': digest.hexdigest(),
            'method': method,
        })
        return record

    def _build_map_identity(self, files):
        records = {
            name: self._quick_file_identity(path)
            for name, path in sorted(files.items())
        }
        combined = hashlib.sha256()
        for name in sorted(records):
            combined.update(name.encode('utf-8'))
            combined.update(records[name].get(
                'digest', 'missing').encode('ascii'))
        return {
            'map_module': self.map_module,
            'map_dir': self.map_dir,
            'fingerprint': combined.hexdigest(),
            'files': records,
        }

    # ---- data loading ----

    def _load_map(self, pgm_file, yaml_file):
        try:
            with open(pgm_file, 'rb') as f:
                assert f.readline().strip() == b'P5'
                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()
                w, h = [int(x) for x in line.strip().split()]
                f.readline()  # maxval
                data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)

            self.map_res = 0.2
            self.map_ox = 0.0
            self.map_oy = 0.0
            with open(yaml_file, 'r') as f:
                for line in f:
                    if line.startswith('resolution:'):
                        self.map_res = float(line.split(':')[1])
                    elif line.startswith('origin:'):
                        parts = line.split('[')[1].split(']')[0].split(',')
                        self.map_ox = float(parts[0])
                        self.map_oy = float(parts[1])

            self.map_data = data
            self.map_width = w
            self.map_height = h
            rospy.loginfo('Map loaded: %dx%d', w, h)
        except Exception as e:
            rospy.logwarn('Could not load map: %s', str(e))

    def _load_dem(self, dem_file):
        try:
            with open(dem_file, 'rb') as f:
                magic = f.read(4)
                if magic != b'DEM1':
                    raise ValueError('Invalid DEM magic')
                rows = struct.unpack('i', f.read(4))[0]
                cols = struct.unpack('i', f.read(4))[0]
                res = struct.unpack('f', f.read(4))[0]
                ox = struct.unpack('f', f.read(4))[0]
                oy = struct.unpack('f', f.read(4))[0]
                data = np.frombuffer(f.read(), dtype=np.float32).reshape(rows, cols)
            self.dem = data
            self.dem_rows, self.dem_cols = rows, cols
            self.dem_res, self.dem_ox, self.dem_oy = res, ox, oy
            rospy.loginfo('DEM loaded: %dx%d, Z=[%.2f, %.2f]',
                          cols, rows, np.nanmin(data), np.nanmax(data))
        except Exception as e:
            rospy.logwarn('Could not load DEM: %s', str(e))

    def _load_obstacle_map(self, obstacle_file):
        try:
            with open(obstacle_file, 'rb') as f:
                if f.readline().strip() != b'P5':
                    raise ValueError('Only binary P5 PGM is supported')
                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()
                width, height = [int(x) for x in line.strip().split()]
                maximum = int(f.readline().strip())
                if maximum != 255:
                    raise ValueError('Expected 8-bit obstacle PGM')
                data = np.frombuffer(f.read(), dtype=np.uint8)
            if data.size != width * height:
                raise ValueError('Incomplete obstacle PGM')
            self.obstacle_data = data.reshape(height, width)
            rospy.loginfo('Obstacle mask loaded: %dx%d', width, height)
        except Exception as e:
            rospy.logwarn('Could not load obstacle mask: %s', str(e))

    def _draw_terrain_background(self, ax):
        """Draw elevation colors and gray hard obstacles on a trajectory axis."""
        if (self.dem is not None and self.obstacle_data is not None
                and self.obstacle_data.shape == self.dem.shape):
            extent = [
                self.dem_ox,
                self.dem_ox + self.dem_cols * self.dem_res,
                self.dem_oy,
                self.dem_oy + self.dem_rows * self.dem_res]
            valid = np.isfinite(self.dem)
            terrain = np.ma.array(self.dem, mask=~valid)
            ax.set_facecolor('#f5f5f5')
            image = ax.imshow(
                terrain, cmap='YlGn', origin='lower', extent=extent,
                interpolation='bilinear', aspect='equal', alpha=0.90,
                zorder=0)

            values = self.dem[valid]
            if values.size:
                low, high = np.nanpercentile(values, [2.0, 98.0])
                if high > low:
                    image.set_clim(low, high)
                    levels = np.linspace(low, high, 11)
                    ax.contour(
                        np.linspace(extent[0], extent[1], self.dem_cols),
                        np.linspace(extent[2], extent[3], self.dem_rows),
                        terrain, levels=levels, colors='#26352c',
                        linewidths=0.35, alpha=0.32, zorder=1)

            # PGM storage is top-to-bottom; DEM row indices grow with world Y.
            hard_obstacle = np.flipud(self.obstacle_data) == 0
            obstacle_overlay = np.ma.masked_where(
                ~hard_obstacle, hard_obstacle)
            ax.imshow(
                obstacle_overlay, origin='lower', extent=extent,
                cmap=ListedColormap(['#8f9398']), interpolation='nearest',
                aspect='equal', zorder=2)
            if np.any(hard_obstacle):
                ax.contour(
                    np.linspace(extent[0], extent[1], self.dem_cols),
                    np.linspace(extent[2], extent[3], self.dem_rows),
                    hard_obstacle.astype(np.uint8), levels=[0.5],
                    colors='#55585c', linewidths=0.45, zorder=3)

            colorbar = ax.figure.colorbar(
                image, ax=ax, pad=0.025, shrink=0.84)
            colorbar.set_label('Elevation Z (m)')
            return

        # Preserve the original display for maps generated before the
        # independent obstacle layer was introduced.
        if self.map_data is not None:
            extent = [self.map_ox,
                      self.map_ox + self.map_width * self.map_res,
                      self.map_oy,
                      self.map_oy + self.map_height * self.map_res]
            ax.imshow(np.flipud(self.map_data), cmap='gray', origin='lower',
                      extent=extent, alpha=0.5, zorder=0)

    # ---- ROS callbacks ----

    def _action_goal_cb(self, msg):
        self._goal_cb(msg.goal.target_pose)

    def _goal_cb(self, msg):
        self.planned_path = None
        self.coarse_paths = []
        self.corridor_paths = []
        self.fine_candidate_paths = []
        self.pareto_candidates = []
        self.pareto_summary = {}
        self.planner_statistics = []
        self.planner_stage_batches = {}
        self.plan_stamp_ns = None
        self.actual_traj = []
        self.control_traj = []
        self.ndt_target_traj = []
        self.cmd_vel_traj = []
        self.mpc_cmd_vel_traj = []
        self.mpc_diagnostics = []
        self.mpc_prediction_horizons = []
        self.wheel_state_traj = []
        self.gt_traj = []
        self.gt_twist_traj = []
        self.reference_state_records = []
        self.vehicle_sensor_records = []
        self.imu_raw_records = []
        self.motor_feedback_records = []
        self.last_gt_time = 0.0
        # Freeze an estimator-independent world->map transform before motion.
        # Never align GT to the controller pose: doing so would absorb the
        # initial NDT error into the reference and invalidate absolute ATE.
        if (self.align_gt_to_control_frame and
                not self.gt_anchor_attempted):
            self.gt_anchor_attempted = True
            self.gt_frame_anchor = self._make_gt_frame_anchor()
            if self.gt_frame_anchor is not None:
                anchor = self.gt_frame_anchor
                rospy.loginfo(
                    'GT absolute anchor locked from %d stationary samples: '
                    'translation=(%+.3f,%+.3f,%+.3f)m yaw=%+.2fdeg '
                    'span=%.3fm/%.2fdeg',
                    anchor['samples'], anchor['x'], anchor['y'], anchor['z'],
                    np.degrees(anchor['yaw']), anchor['position_span'],
                    np.degrees(anchor['yaw_span']))
        if self.align_gt_to_control_frame and self.gt_frame_anchor is None:
            rospy.logwarn(
                'GT absolute anchor unavailable; this run will omit GT/ATE '
                'metrics instead of fitting GT to localization after motion')
        self.nav_active = True
        self.shutdown_plot_started = False
        self.start_time = rospy.Time.now()
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.last_status = None
        self._nav_output('[NAV] 开始导航 | 目标=(%.2f, %.2f)',
                         self.goal_pose[0], self.goal_pose[1])
        rospy.loginfo('Navigation goal: (%.1f, %.1f)', *self.goal_pose)

    def _plan_cb(self, msg):
        if (self.nav_active and self.planned_path is None
                and len(msg.poses) > 0):
            self.plan_stamp_ns = msg.header.stamp.to_nsec()
            self.planned_path = []
            for path_pose in msg.poses:
                orientation = path_pose.pose.orientation
                yaw = tft.euler_from_quaternion([
                    orientation.x, orientation.y,
                    orientation.z, orientation.w])[2]
                self.planned_path.append((
                    path_pose.pose.position.x,
                    path_pose.pose.position.y,
                    path_pose.pose.position.z,
                    yaw))
            self._adopt_planner_batch(self.plan_stamp_ns)
            plan_length = sum(
                np.sqrt(
                    (self.planned_path[i + 1][0] - self.planned_path[i][0]) ** 2
                    + (self.planned_path[i + 1][1]
                       - self.planned_path[i][1]) ** 2)
                for i in range(len(self.planned_path) - 1))
            self._nav_output('[NAV] 全局路径 | 长度=%.2fm | 路点=%d',
                             plan_length, len(self.planned_path))
            rospy.loginfo('Plan received: %d points, %.1fm',
                          len(self.planned_path), plan_length)

    @staticmethod
    def _paths_from_markers(msg):
        paths = []
        for marker in sorted(msg.markers, key=lambda item: item.id):
            if (marker.action != Marker.ADD
                    or marker.type != Marker.LINE_STRIP
                    or len(marker.points) < 2):
                continue
            points = np.asarray([(p.x, p.y) for p in marker.points],
                                dtype=float)
            color = (marker.color.r, marker.color.g, marker.color.b)
            paths.append({
                'points': points,
                'width': float(marker.scale.x),
                'color': color,
            })
        return paths

    @staticmethod
    def _corridors_from_markers(msg):
        corridors = []
        for marker in sorted(msg.markers, key=lambda item: item.id):
            if (marker.action != Marker.ADD
                    or marker.type != Marker.POINTS
                    or len(marker.points) == 0):
                continue
            points = np.asarray([(p.x, p.y) for p in marker.points],
                                dtype=float)
            corridors.append({
                'points': points,
                'cell_size': float(marker.scale.x),
                'color': (marker.color.r, marker.color.g, marker.color.b),
            })
        return corridors

    @staticmethod
    def _marker_stamp_ns(msg):
        stamps = [marker.header.stamp.to_nsec() for marker in msg.markers
                  if marker.header.stamp.to_nsec() > 0]
        return max(stamps) if stamps else None

    def _store_planner_stage(self, stamp_ns, key, value):
        if not self.nav_active or stamp_ns is None:
            return
        batch = self.planner_stage_batches.setdefault(stamp_ns, {})
        batch[key] = value
        # A navigation request normally has one batch. Retain a few batches
        # for asynchronous callback ordering without accumulating old plans.
        if len(self.planner_stage_batches) > 8:
            for old_stamp in sorted(self.planner_stage_batches)[:-8]:
                del self.planner_stage_batches[old_stamp]
        if self.plan_stamp_ns == stamp_ns:
            self._adopt_planner_batch(stamp_ns)

    def _adopt_planner_batch(self, stamp_ns):
        batch = self.planner_stage_batches.get(stamp_ns, {})
        if 'coarse' in batch:
            self.coarse_paths = batch['coarse']
        if 'corridors' in batch:
            self.corridor_paths = batch['corridors']
        if 'fine' in batch:
            self.fine_candidate_paths = batch['fine']
        if 'pareto' in batch:
            self.pareto_candidates = batch['pareto']['candidates']
            self.pareto_summary = batch['pareto']['summary']

    def _coarse_paths_cb(self, msg):
        self._store_planner_stage(
            self._marker_stamp_ns(msg), 'coarse',
            self._paths_from_markers(msg))

    def _corridors_cb(self, msg):
        self._store_planner_stage(
            self._marker_stamp_ns(msg), 'corridors',
            self._corridors_from_markers(msg))

    def _fine_candidates_cb(self, msg):
        self._store_planner_stage(
            self._marker_stamp_ns(msg), 'fine',
            self._paths_from_markers(msg))

    def _pareto_decision_cb(self, msg):
        candidates = []
        summary = {}
        for marker in msg.markers:
            if marker.action != Marker.ADD or not marker.text:
                continue
            try:
                record = json.loads(marker.text)
            except (TypeError, ValueError):
                rospy.logwarn_throttle(
                    5.0, 'Invalid Pareto audit marker: %s', marker.text)
                continue
            if record.get('kind') == 'candidate':
                candidates.append(record)
            elif record.get('kind') == 'summary':
                summary = record
        candidates.sort(key=lambda item: int(item.get('index', 0)))
        self._store_planner_stage(
            self._marker_stamp_ns(msg), 'pareto',
            {'candidates': candidates, 'summary': summary})

    def _planner_statistics_cb(self, msg):
        if not self.nav_active:
            return
        try:
            record = json.loads(msg.data)
        except (TypeError, ValueError):
            rospy.logwarn_throttle(
                5.0, 'Invalid planner statistics JSON: %s', msg.data)
            return
        if not isinstance(record, dict):
            return
        record = record.copy()
        record['received_time_s'] = rospy.Time.now().to_sec()
        self.planner_statistics.append(record)

    @staticmethod
    def _write_csv_rows(path, header, rows):
        with open(path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)

    @staticmethod
    def _write_csv_dicts(path, records, preferred_fields=None):
        records = [record for record in records if isinstance(record, dict)]
        preferred_fields = list(preferred_fields or [])
        discovered = sorted({key for record in records for key in record})
        fields = preferred_fields + [
            key for key in discovered if key not in preferred_fields]
        if not fields:
            fields = ['no_data']
        with open(path, 'w', newline='') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            for record in records:
                writer.writerow(record)

    @staticmethod
    def _run_artifact_dirs(run_dir):
        """Create and return the three artifact groups for one navigation."""
        directories = {
            'figures': os.path.join(run_dir, 'figures'),
            'csv': os.path.join(run_dir, 'csv'),
            'json': os.path.join(run_dir, 'json'),
        }
        os.makedirs(run_dir, exist_ok=True)
        for directory in directories.values():
            os.makedirs(directory, exist_ok=True)
        return directories

    def _slip_observation_rows(self):
        """Synchronize wheel kinematics with the external vehicle reference.

        The returned slip ratios are observations, not controller states.
        Samples close to zero longitudinal or yaw motion are marked invalid
        because the corresponding ratio is numerically unobservable.
        """
        if (len(self.wheel_state_traj) < 2 or
                len(self.gt_twist_traj) < 2):
            return []
        wheel = np.asarray(self.wheel_state_traj, dtype=float)
        truth = np.asarray(self.gt_twist_traj, dtype=float)
        gt_v = np.interp(wheel[:, 0], truth[:, 0], truth[:, 1])
        gt_omega = np.interp(wheel[:, 0], truth[:, 0], truth[:, 2])

        roll = np.full(len(wheel), np.nan)
        pitch = np.full(len(wheel), np.nan)
        if len(self.gt_traj) >= 2:
            attitude = np.asarray(self.gt_traj, dtype=float)
            roll = np.interp(wheel[:, 0], attitude[:, 0], attitude[:, 4])
            pitch = np.interp(wheel[:, 0], attitude[:, 0], attitude[:, 5])

        rows = []
        for index, sample in enumerate(wheel):
            v_kin = sample[5]
            omega_kin = sample[6]
            v_valid = abs(v_kin) >= 0.05
            omega_valid = abs(omega_kin) >= 0.03
            longitudinal_slip = (1.0 - gt_v[index] / v_kin
                                 if v_valid else float('nan'))
            turning_slip = (1.0 - gt_omega[index] / omega_kin
                            if omega_valid else float('nan'))
            rows.append((
                sample[0], sample[1], sample[2], sample[3], sample[4],
                v_kin, omega_kin, gt_v[index], gt_omega[index],
                longitudinal_slip, turning_slip,
                1 if v_valid else 0, 1 if omega_valid else 0,
                np.degrees(pitch[index]), np.degrees(roll[index]),
                int(sample[7]), int(sample[8])))
        return rows

    def _export_raw_csv(self, run_dir):
        """Export lossless per-topic samples and planner internals.

        Files are intentionally separated by source because ROS topics have
        different rates.  Offline scripts can synchronize them explicitly
        without hidden interpolation or resampling by the evaluator.
        """
        artifact_dirs = self._run_artifact_dirs(run_dir)
        csv_dir = artifact_dirs['csv']
        json_dir = artifact_dirs['json']
        saved = []

        def rows_file(filename, header, rows):
            path = os.path.join(csv_dir, filename)
            self._write_csv_rows(path, header, rows)
            saved.append(path)

        plan_rows = []
        for index, point in enumerate(self.planned_path or []):
            plan_rows.append((
                index, point[0], point[1], point[2],
                point[3] if len(point) > 3 else float('nan')))
        rows_file(
            'planned_path.csv', ['point_index', 'x_m', 'y_m', 'z_m',
                                 'yaw_rad'], plan_rows)
        rows_file(
            'mapping_odometry.csv',
            ['stamp_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
             'yaw_rad'], self.actual_traj)
        rows_file(
            'controller_odometry.csv',
            ['stamp_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
             'yaw_rad', 'linear_speed_mps', 'yaw_rate_radps'],
            self.control_traj)
        rows_file(
            'ndt_target_odometry.csv',
            ['stamp_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
             'yaw_rad'], self.ndt_target_traj)
        rows_file(
            'qianxun_reference.csv',
            ['stamp_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
             'yaw_rad'], self.gt_traj)
        gt_map_rows = []
        if self.gt_traj:
            gt_map, _ = self._gt_in_control_frame(
                np.asarray(self.gt_traj, dtype=float))
            if gt_map is not None:
                gt_map_rows = gt_map.tolist()
        rows_file(
            'qianxun_reference_map.csv',
            ['stamp_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad',
             'yaw_rad'], gt_map_rows)
        rows_file(
            'qianxun_reference_twist.csv',
            ['stamp_s', 'forward_speed_mps', 'yaw_rate_radps'],
            self.gt_twist_traj)
        self._write_csv_dicts(
            os.path.join(csv_dir, 'qianxun_state_full.csv'),
            self.reference_state_records,
            preferred_fields=['record_stamp_s', 'UTM_x', 'UTM_y', 'Z',
                              'Head', 'V_x', 'V_y', 'Omega', 'Roll',
                              'Pitch', 'FusionValid', 'PositionValid',
                              'ControlFrameReady'])
        saved.append(os.path.join(csv_dir, 'qianxun_state_full.csv'))
        gps_fields = [
            'record_stamp_s', 'Timestamp', 'AntennaLat', 'AntennaLon',
            'AntennaAlt', 'AntennaX', 'AntennaY', 'AntennaZ',
            'GGA_quality', 'GGA_sats', 'GGA_hdop', 'Status', 'GNSSAge',
            'GNSSRxAge', 'GNSSObservedHz', 'PositionValid']
        rows_file(
            'gps_antenna_measurements.csv', gps_fields,
            [[record.get(field, '') for field in gps_fields]
             for record in self.reference_state_records
             if isinstance(record, dict)])
        self._write_csv_dicts(
            os.path.join(csv_dir, 'vehicle_sensor_full.csv'),
            self.vehicle_sensor_records,
            preferred_fields=['record_stamp_s'])
        saved.append(os.path.join(csv_dir, 'vehicle_sensor_full.csv'))
        rows_file(
            'imu_raw.csv',
            ['record_stamp_s', 'header_stamp_s', 'sequence', 'frame_id',
             'orientation_x', 'orientation_y', 'orientation_z',
             'orientation_w', 'roll_rad', 'pitch_rad', 'yaw_rad',
             'angular_velocity_x_radps', 'angular_velocity_y_radps',
             'angular_velocity_z_radps', 'linear_acceleration_x_mps2',
             'linear_acceleration_y_mps2', 'linear_acceleration_z_mps2'],
            self.imu_raw_records)
        self._write_csv_dicts(
            os.path.join(csv_dir, 'motor_feedback_raw.csv'),
            self.motor_feedback_records,
            preferred_fields=['record_stamp_s', 'stamp_s', 'can_id',
                              'raw_left', 'raw_right', 'left_rpm',
                              'right_rpm'])
        saved.append(os.path.join(csv_dir, 'motor_feedback_raw.csv'))
        rows_file(
            'cmd_vel.csv', ['stamp_s', 'linear_x_mps', 'angular_z_radps'],
            self.cmd_vel_traj)
        rows_file(
            'cmd_vel_raw.csv',
            ['stamp_s', 'linear_x_mps', 'angular_z_radps'],
            self.mpc_cmd_vel_traj)
        rows_file(
            'mpc_diagnostics.csv', self.MPC_DIAGNOSTIC_FIELDS,
            self.mpc_diagnostics)
        rows_file(
            'mpc_prediction_horizons.csv',
            ['stamp_s', 'horizon_index', 'horizon_time_s',
             'predicted_x_m', 'predicted_y_m', 'predicted_yaw_rad',
             'predicted_v_mps', 'predicted_omega_radps',
             'predicted_cte_m', 'predicted_heading_error_rad'],
            self.mpc_prediction_horizons)
        rows_file(
            'wheel_joint_kinematics.csv',
            ['stamp_s', 'left_mean_radps', 'right_mean_radps',
             'left_track_mps', 'right_track_mps',
             'kinematic_forward_mps', 'kinematic_yaw_rate_radps',
             'left_joint_count', 'right_joint_count'],
            self.wheel_state_traj)
        rows_file(
            'slip_observation.csv',
            ['stamp_s', 'left_mean_radps', 'right_mean_radps',
             'left_track_mps', 'right_track_mps',
             'kinematic_forward_mps', 'kinematic_yaw_rate_radps',
             'gt_forward_mps', 'gt_yaw_rate_radps',
             'longitudinal_slip_ratio', 'turning_slip_ratio',
             'longitudinal_valid', 'turning_valid',
             'gt_pitch_deg', 'gt_roll_deg',
             'left_joint_count', 'right_joint_count'],
            self._slip_observation_rows())

        coarse_rows = []
        for path_index, path in enumerate(self.coarse_paths, start=1):
            for point_index, point in enumerate(path['points']):
                coarse_rows.append((
                    path_index, point_index, point[0], point[1],
                    path.get('width', float('nan')),
                    path['color'][0], path['color'][1], path['color'][2]))
        rows_file(
            'coarse_paths.csv',
            ['path_index', 'point_index', 'x_m', 'y_m', 'line_width_m',
             'color_r', 'color_g', 'color_b'], coarse_rows)

        fine_rows = []
        for path_index, path in enumerate(
                self.fine_candidate_paths, start=1):
            for point_index, point in enumerate(path['points']):
                fine_rows.append((
                    path_index, point_index, point[0], point[1],
                    path.get('width', float('nan')),
                    path['color'][0], path['color'][1], path['color'][2]))
        rows_file(
            'fine_candidate_paths.csv',
            ['candidate_index', 'point_index', 'x_m', 'y_m',
             'line_width_m', 'color_r', 'color_g', 'color_b'], fine_rows)

        corridor_rows = []
        for corridor_index, corridor in enumerate(
                self.corridor_paths, start=1):
            for cell_index, point in enumerate(corridor['points']):
                corridor_rows.append((
                    corridor_index, cell_index, point[0], point[1],
                    corridor.get('cell_size', float('nan')),
                    corridor['color'][0], corridor['color'][1],
                    corridor['color'][2]))
        rows_file(
            'corridor_cells.csv',
            ['corridor_index', 'cell_index', 'x_m', 'y_m', 'cell_size_m',
             'color_r', 'color_g', 'color_b'], corridor_rows)

        pareto_path = os.path.join(csv_dir, 'pareto_candidates.csv')
        self._write_csv_dicts(
            pareto_path, self.pareto_candidates,
            ['index', 'channel', 'role', 'length', 'mean_risk',
             'peak_risk', 'tail_risk', 'high_risk_fraction',
             'combined_risk_objective',
             'mean_abs_pitch_deg', 'p95_abs_pitch_deg',
             'max_abs_pitch_deg', 'pitch_exposure_fraction',
             'pitch_objective',
             'mean_abs_roll_deg', 'p95_abs_roll_deg',
             'max_abs_roll_deg', 'roll_exposure_fraction',
             'roll_objective', 'attitude_feasible',
              'safety_feasible', 'safety_violation', 'pareto',
              'decision_eligible',
             'distance_regret', 'terrain_regret', 'compromise_score',
             'ideal_distance', 'selected'])
        saved.append(pareto_path)
        summary_path = os.path.join(csv_dir, 'pareto_summary.csv')
        self._write_csv_dicts(summary_path, [self.pareto_summary])
        saved.append(summary_path)

        statistics_path = os.path.join(csv_dir, 'planner_performance.csv')
        self._write_csv_dicts(
            statistics_path, self.planner_statistics,
            ['algorithm', 'success', 'failure_reason', 'planning_time_ms',
             'iterations', 'expanded_nodes', 'generated_nodes',
             'coarse_search_calls', 'coarse_iterations',
             'coarse_expanded_nodes', 'coarse_generated_nodes',
             'reopened_states', 'stale_queue_pops',
             'primitive_attempts', 'collision_checks',
             'collision_rejections', 'physical_obstacle_rejections',
             'attitude_limit_rejections', 'dominated_state_rejections',
             'max_open_size', 'stored_nodes',
             'max_stored_nodes', 'search_calls', 'path_points',
             'path_length_m', 'terrain_cost_integral',
             'mean_abs_curvature_1pm', 'max_abs_curvature_1pm',
             'mean_terrain_risk', 'peak_terrain_risk', 'coarse_channels',
             'mean_abs_pitch_deg', 'p95_abs_pitch_deg',
             'max_abs_pitch_deg', 'pitch_exposure_fraction',
             'pitch_objective', 'mean_abs_roll_deg',
             'p95_abs_roll_deg', 'max_abs_roll_deg',
             'roll_exposure_fraction', 'roll_objective',
             'fine_candidates', 'selected_candidate'])
        saved.append(statistics_path)

        goal_rows = []
        if self.goal_pose is not None:
            goal_rows.append((self.goal_pose[0], self.goal_pose[1]))
        rows_file('navigation_goal.csv', ['goal_x_m', 'goal_y_m'], goal_rows)

        curvature_s, curvature, _, _ = self._path_curvature_metrics(
            self.planned_path or [])
        rows_file(
            'path_curvature.csv',
            ['arc_length_m', 'signed_curvature_1pm',
             'abs_curvature_1pm'],
            [(float(distance), float(value), float(abs(value)))
             for distance, value in zip(curvature_s, curvature)])

        manifest = {
            'schema_version': 4,
            'map': self.map_identity,
            'localization_mode': self.localization_mode,
            'description': ('Raw ROS-topic samples are not resampled; '
                            'timestamps retain their original rates.'),
            'files': [os.path.basename(path) for path in saved],
            'planner_comparison': {
                'basic': 'basic_hybrid_astar',
                'risk_weighted': 'risk_weighted_hybrid_astar',
                'proposed': 'multi_corridor_pareto_hybrid_astar',
            },
        }
        manifest_path = os.path.join(json_dir, 'csv_manifest.json')
        with open(manifest_path, 'w') as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
        saved.append(manifest_path)
        return saved

    def _generate_mpc_diagnostic_plots(self, run_dir, time_origin):
        """Create controller-diagnostic figures without changing control."""
        artifact_dirs = self._run_artifact_dirs(run_dir)
        figure_dir = artifact_dirs['figures']
        json_dir = artifact_dirs['json']
        saved = []
        metrics = {
            'map': self.map_identity,
            'localization_mode': self.localization_mode,
            'note': ('All slip quantities are evaluator-only external-reference '
                     'observations and are not fed back into MPC.'),
        }

        if len(self.mpc_diagnostics) >= 2:
            diagnostic = np.asarray(self.mpc_diagnostics, dtype=float)
            field = {name: index for index, name in enumerate(
                     self.MPC_DIAGNOSTIC_FIELDS)}
            time_s = diagnostic[:, field['stamp_s']] - time_origin

            # Online yaw-response identification.
            fig_gain, axes_gain = plt.subplots(
                2, 1, figsize=(12, 7), sharex=True)
            axes_gain[0].plot(
                time_s, diagnostic[:, field['omega_cmd_radps']],
                color='#6a3d9a', lw=1.1, label='MPC command')
            axes_gain[0].plot(
                time_s, diagnostic[:, field['measured_omega_radps']],
                color='#009e73', lw=1.1, label='Measured response')
            axes_gain[0].set_ylabel('Yaw rate (rad/s)')
            axes_gain[0].legend(fontsize=8)
            axes_gain[0].grid(True, alpha=0.3)
            observed_gain = diagnostic[:, field['observed_omega_gain']]
            valid_gain = np.isfinite(observed_gain)
            if np.any(valid_gain):
                axes_gain[1].scatter(
                    time_s[valid_gain], observed_gain[valid_gain], s=8,
                    color='#e69f00', alpha=0.45, label='Observed gain')
            axes_gain[1].plot(
                time_s, diagnostic[:, field['model_omega_gain']],
                color='#0072b2', lw=1.5, label='Identified model gain')
            axes_gain[1].set_xlabel('Time (s)')
            axes_gain[1].set_ylabel('Yaw response gain')
            axes_gain[1].legend(fontsize=8)
            axes_gain[1].grid(True, alpha=0.3)
            fig_gain.suptitle('Online Yaw-Response Identification')
            fig_gain.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            gain_path = os.path.join(
                figure_dir, 'omega_gain_identification.png')
            fig_gain.savefig(gain_path, dpi=150, bbox_inches='tight')
            plt.close(fig_gain)
            saved.append(gain_path)

            # Joint longitudinal/yaw adaptive response model. This plot is
            # deliberately separate from equivalent-slip observations: these
            # values are the parameters actually used by MPC prediction.
            fig_model, axes_model = plt.subplots(
                3, 1, figsize=(12, 9), sharex=True)
            axes_model[0].plot(
                time_s, diagnostic[:, field['linear_reference_mps']],
                color='#0072b2', lw=1.0, label='Speed reference')
            axes_model[0].plot(
                time_s, diagnostic[:, field['linear_cmd_mps']],
                color='#6a3d9a', lw=1.0, label='MPC command')
            axes_model[0].plot(
                time_s, diagnostic[:, field['measured_forward_mps']],
                color='#009e73', lw=1.0, label='Measured response')
            axes_model[0].set_ylabel('Linear speed (m/s)')
            axes_model[0].legend(fontsize=8, ncol=3)
            axes_model[0].grid(True, alpha=0.3)
            axes_model[1].plot(
                time_s, diagnostic[:, field['model_linear_gain']],
                color='#0072b2', lw=1.2, label=r'$\hat{\lambda}_v$')
            axes_model[1].plot(
                time_s, diagnostic[:, field['model_omega_gain']],
                color='#d55e00', lw=1.2, label=r'$\hat{K}_\omega$')
            axes_model[1].set_ylabel('Response gain')
            axes_model[1].legend(fontsize=8)
            axes_model[1].grid(True, alpha=0.3)
            axes_model[2].plot(
                time_s, diagnostic[:, field['model_tau_v_s']],
                color='#0072b2', lw=1.2, label=r'$\hat{\tau}_v$')
            axes_model[2].plot(
                time_s, diagnostic[:, field['model_tau_omega_s']],
                color='#d55e00', lw=1.2, label=r'$\hat{\tau}_\omega$')
            axes_model[2].set_xlabel('Time (s)')
            axes_model[2].set_ylabel('Time constant (s)')
            axes_model[2].legend(fontsize=8)
            axes_model[2].grid(True, alpha=0.3)
            fig_model.suptitle('Adaptive MPC Response Model')
            fig_model.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            model_path = os.path.join(
                figure_dir, 'adaptive_response_model.png')
            fig_model.savefig(model_path, dpi=150, bbox_inches='tight')
            plt.close(fig_model)
            saved.append(model_path)

            # Curvature feedforward and the resulting tracking error.
            fig_ff, axes_ff = plt.subplots(
                3, 1, figsize=(12, 9), sharex=True)
            axes_ff[0].plot(
                time_s, diagnostic[:, field['curvature_1pm']],
                color='#0072b2', lw=1.2)
            axes_ff[0].set_ylabel(r'Curvature (m$^{-1}$)')
            axes_ff[0].grid(True, alpha=0.3)
            axes_ff[1].plot(
                time_s, diagnostic[:, field['omega_ff_radps']],
                color='#e69f00', lw=1.2, label='Curvature feedforward')
            axes_ff[1].plot(
                time_s, diagnostic[:, field['omega_cmd_radps']],
                color='#6a3d9a', lw=1.0, label='Selected command')
            axes_ff[1].plot(
                time_s, diagnostic[:, field['measured_omega_radps']],
                color='#009e73', lw=1.0, label='Measured response')
            axes_ff[1].set_ylabel('Yaw rate (rad/s)')
            axes_ff[1].legend(fontsize=8, ncol=3)
            axes_ff[1].grid(True, alpha=0.3)
            axes_ff[2].plot(
                time_s, diagnostic[:, field['signed_cte_m']],
                color='#d55e00', lw=1.2)
            axes_ff[2].axhline(0.0, color='black', lw=0.6)
            axes_ff[2].set_xlabel('Time (s)')
            axes_ff[2].set_ylabel('Signed CTE (m)')
            axes_ff[2].grid(True, alpha=0.3)
            fig_ff.suptitle('Curvature Feedforward and Path Response')
            fig_ff.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            ff_path = os.path.join(
                figure_dir, 'curvature_feedforward_response.png')
            fig_ff.savefig(ff_path, dpi=150, bbox_inches='tight')
            plt.close(fig_ff)
            saved.append(ff_path)

            # CTE-residual lateral-disturbance observer.
            fig_lat, axes_lat = plt.subplots(
                3, 1, figsize=(12, 9), sharex=True)
            axes_lat[0].plot(
                time_s, diagnostic[:, field['signed_cte_m']],
                color='#d55e00', lw=1.2, label='Signed CTE')
            axes_lat[0].plot(
                time_s,
                diagnostic[:, field['predicted_cte_at_measurement_m']],
                color='#0072b2', lw=1.0, alpha=0.75,
                label='Previous-model CTE prediction')
            axes_lat[0].legend(fontsize=8)
            axes_lat[0].set_ylabel('CTE (m)')
            axes_lat[0].grid(True, alpha=0.3)
            axes_lat[1].plot(
                time_s, diagnostic[:, field['lateral_residual_mps']],
                color='#cc79a7', lw=0.9, alpha=0.65,
                label='Observed residual')
            axes_lat[1].plot(
                time_s, diagnostic[:, field['lateral_estimate_mps']],
                color='#009e73', lw=1.4, label='Filtered estimate')
            axes_lat[1].set_ylabel('Lateral velocity (m/s)')
            axes_lat[1].legend(fontsize=8)
            axes_lat[1].grid(True, alpha=0.3)
            residual_valid = diagnostic[
                :, field['cte_prediction_residual_valid']] > 0.5
            residual_m = diagnostic[:, field['cte_prediction_residual_m']]
            for name, color, label in [
                    ('cte_heading_ref_rad', '#0072b2', 'CTE reference'),
                    ('disturbance_heading_ref_rad', '#009e73',
                     'Disturbance reference'),
                    ('rate_heading_ref_rad', '#e69f00', 'Rate reference')]:
                axes_lat[2].plot(
                    time_s, np.degrees(diagnostic[:, field[name]]),
                    color=color, lw=1.1, label=label)
            axes_lat[2].set_xlabel('Time (s)')
            axes_lat[2].set_ylabel('Heading correction (deg)')
            axes_lat[2].legend(fontsize=8, ncol=3)
            axes_lat[2].grid(True, alpha=0.3)
            fig_lat.suptitle('CTE-Residual Lateral-Disturbance Observer')
            fig_lat.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            lateral_path = os.path.join(
                figure_dir, 'lateral_disturbance_observer.png')
            fig_lat.savefig(lateral_path, dpi=150, bbox_inches='tight')
            plt.close(fig_lat)
            saved.append(lateral_path)

            model_gain = diagnostic[:, field['model_omega_gain']]
            metrics['yaw_response_identification'] = {
                'valid_observed_gain_samples': int(np.sum(valid_gain)),
                'model_gain_min': float(np.nanmin(model_gain)),
                'model_gain_max': float(np.nanmax(model_gain)),
                'model_gain_final': float(model_gain[-1]),
            }
            linear_gain = diagnostic[:, field['model_linear_gain']]
            tau_v = diagnostic[:, field['model_tau_v_s']]
            tau_omega = diagnostic[:, field['model_tau_omega_s']]
            metrics['adaptive_response_model'] = {
                'linear_gain_min': float(np.nanmin(linear_gain)),
                'linear_gain_max': float(np.nanmax(linear_gain)),
                'linear_gain_final': float(linear_gain[-1]),
                'tau_v_min_s': float(np.nanmin(tau_v)),
                'tau_v_max_s': float(np.nanmax(tau_v)),
                'tau_v_final_s': float(tau_v[-1]),
                'tau_omega_min_s': float(np.nanmin(tau_omega)),
                'tau_omega_max_s': float(np.nanmax(tau_omega)),
                'tau_omega_final_s': float(tau_omega[-1]),
                'command_alignment_age_mean_s': float(np.nanmean(
                    diagnostic[:, field['command_alignment_age_s']])),
                'cte_prediction_residual_valid_samples': int(
                    np.sum(residual_valid)),
                'cte_prediction_residual_rmse_m': (
                    float(np.sqrt(np.nanmean(
                        residual_m[residual_valid] *
                        residual_m[residual_valid])))
                    if np.any(residual_valid) else float('nan')),
            }
            solver_duration = diagnostic[:, field['solver_duration_ms']]
            solver_timeout = diagnostic[:, field['solver_timed_out']] > 0.5
            metrics['controller_watchdog'] = {
                'state_age_max_s': float(np.nanmax(
                    diagnostic[:, field['state_age_s']])),
                'tf_age_max_s': float(np.nanmax(
                    diagnostic[:, field['tf_age_s']])),
                'control_cycle_interval_max_s': float(np.nanmax(
                    diagnostic[:, field['control_cycle_interval_s']])),
                'solver_duration_mean_ms': float(np.nanmean(
                    solver_duration)),
                'solver_duration_max_ms': float(np.nanmax(
                    solver_duration)),
                'solver_timeout_samples': int(np.sum(solver_timeout)),
                'evaluated_candidates_mean': float(np.nanmean(
                    diagnostic[:, field['evaluated_candidates']])),
            }

            # Validate the model's next-step velocity prediction against GT.
            if len(self.gt_twist_traj) >= 2:
                truth = np.asarray(self.gt_twist_traj, dtype=float)
                prediction_time = diagnostic[:, field['stamp_s']] + 0.1
                gt_v_next = np.interp(
                    prediction_time, truth[:, 0], truth[:, 1])
                gt_omega_next = np.interp(
                    prediction_time, truth[:, 0], truth[:, 2])
                predicted_v = diagnostic[:, field['predicted_v_next_mps']]
                predicted_omega = diagnostic[
                    :, field['predicted_omega_next_radps']]
                fig_pred, axes_pred = plt.subplots(
                    2, 1, figsize=(12, 7), sharex=True)
                axes_pred[0].plot(
                    time_s, predicted_v, color='#0072b2', lw=1.1,
                    label='One-step prediction')
                axes_pred[0].plot(
                    time_s, gt_v_next, color='#009e73', lw=1.1,
                    label='Qianxun reference at t+dt')
                axes_pred[0].set_ylabel('Linear speed (m/s)')
                axes_pred[0].legend(fontsize=8)
                axes_pred[0].grid(True, alpha=0.3)
                axes_pred[1].plot(
                    time_s, predicted_omega, color='#6a3d9a', lw=1.1,
                    label='One-step prediction')
                axes_pred[1].plot(
                    time_s, gt_omega_next, color='#009e73', lw=1.1,
                    label='Qianxun reference at t+dt')
                axes_pred[1].set_xlabel('Time (s)')
                axes_pred[1].set_ylabel('Yaw rate (rad/s)')
                axes_pred[1].legend(fontsize=8)
                axes_pred[1].grid(True, alpha=0.3)
                fig_pred.suptitle('MPC One-Step Prediction Validation')
                fig_pred.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
                prediction_path = os.path.join(
                    figure_dir, 'mpc_prediction_validation.png')
                fig_pred.savefig(
                    prediction_path, dpi=150, bbox_inches='tight')
                plt.close(fig_pred)
                saved.append(prediction_path)
                metrics['one_step_prediction'] = {
                    'linear_speed_rmse_mps': float(np.sqrt(np.mean(
                        (predicted_v - gt_v_next) ** 2))),
                    'yaw_rate_rmse_radps': float(np.sqrt(np.mean(
                        (predicted_omega - gt_omega_next) ** 2))),
                    'samples': int(len(predicted_v)),
                }

        slip_rows = self._slip_observation_rows()
        if len(slip_rows) >= 2:
            slip = np.asarray(slip_rows, dtype=float)
            time_s = slip[:, 0] - time_origin
            fig_slip, axes_slip = plt.subplots(
                3, 1, figsize=(12, 9), sharex=True)
            axes_slip[0].plot(
                time_s, slip[:, 5], color='#0072b2', lw=1.0,
                label='Wheel kinematic speed')
            axes_slip[0].plot(
                time_s, slip[:, 7], color='#009e73', lw=1.1,
                label='Qianxun reference speed')
            axes_slip[0].set_ylabel('Linear speed (m/s)')
            axes_slip[0].legend(fontsize=8)
            axes_slip[0].grid(True, alpha=0.3)
            long_valid = slip[:, 11] > 0.5
            turn_valid = slip[:, 12] > 0.5
            if np.any(long_valid):
                axes_slip[1].plot(
                    time_s[long_valid], np.clip(slip[long_valid, 9], -2, 2),
                    color='#0072b2', lw=0.9,
                    label='Longitudinal equivalent slip')
            if np.any(turn_valid):
                axes_slip[1].plot(
                    time_s[turn_valid], np.clip(slip[turn_valid, 10], -2, 2),
                    color='#d55e00', lw=0.9,
                    label='Turning equivalent slip')
            axes_slip[1].axhline(0.0, color='black', lw=0.6)
            axes_slip[1].set_ylabel('Slip ratio (display clipped)')
            axes_slip[1].legend(fontsize=8)
            axes_slip[1].grid(True, alpha=0.3)
            axes_slip[2].plot(
                time_s, slip[:, 13], color='#0072b2', lw=1.0,
                label='Qianxun reference pitch')
            axes_slip[2].plot(
                time_s, slip[:, 14], color='#009e73', lw=1.0,
                label='Qianxun reference roll')
            axes_slip[2].set_xlabel('Time (s)')
            axes_slip[2].set_ylabel('Terrain attitude (deg)')
            axes_slip[2].legend(fontsize=8)
            axes_slip[2].grid(True, alpha=0.3)
            fig_slip.suptitle(
                'Evaluator-Only Equivalent Slip Observation')
            fig_slip.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            slip_path = os.path.join(
                figure_dir, 'equivalent_slip_observation.png')
            fig_slip.savefig(slip_path, dpi=150, bbox_inches='tight')
            plt.close(fig_slip)
            saved.append(slip_path)
            metrics['equivalent_slip_observation'] = {
                'longitudinal_valid_samples': int(np.sum(long_valid)),
                'turning_valid_samples': int(np.sum(turn_valid)),
                'longitudinal_median': (
                    float(np.nanmedian(slip[long_valid, 9]))
                    if np.any(long_valid) else float('nan')),
                'turning_median': (
                    float(np.nanmedian(slip[turn_valid, 10]))
                    if np.any(turn_valid) else float('nan')),
            }

        metrics_path = os.path.join(
            json_dir, 'mpc_diagnostics_metrics.json')
        with open(metrics_path, 'w') as stream:
            json.dump(metrics, stream, indent=2, sort_keys=True)
        saved.append(metrics_path)
        return saved

    def _generate_sensor_diagnostic_plots(self, run_dir):
        """Plot low-bandwidth raw IMU, GNSS and motor feedback samples."""
        figure_dir = self._run_artifact_dirs(run_dir)['figures']
        saved = []

        if len(self.imu_raw_records) >= 2:
            imu = np.asarray([
                [row[index] for index in range(len(row)) if index != 3]
                for row in self.imu_raw_records], dtype=float)
            time_s = imu[:, 1] - imu[0, 1]
            fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
            for offset, label in enumerate(('x', 'y', 'z')):
                axes[0].plot(time_s, imu[:, 13 + offset], lw=0.8,
                             label='a_' + label)
                axes[1].plot(time_s, imu[:, 10 + offset], lw=0.8,
                             label='gyro_' + label)
                axes[2].plot(time_s, np.degrees(imu[:, 7 + offset]),
                             lw=0.8, label=label)
            axes[0].set_ylabel('Acceleration (m/s2)')
            axes[1].set_ylabel('Angular rate (rad/s)')
            axes[2].set_ylabel('RPY (deg)')
            for axis in axes[:3]:
                axis.legend(fontsize=8, ncol=3)
                axis.grid(True, alpha=0.3)
            dt_ms = np.diff(imu[:, 1]) * 1000.0
            axes[3].plot(time_s[1:], dt_ms, color='#444444', lw=0.8)
            axes[3].set_ylabel('Header dt (ms)')
            axes[3].set_xlabel('Time (s)')
            axes[3].grid(True, alpha=0.3)
            fig.suptitle('Raw /imu/data Diagnostics')
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
            path = os.path.join(figure_dir, 'imu_diagnostics.png')
            fig.savefig(path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            saved.append(path)

        gps_records = [record for record in self.reference_state_records
                       if isinstance(record, dict)]
        if len(gps_records) >= 2:
            def values(key, fallback=float('nan')):
                result = []
                for record in gps_records:
                    try:
                        result.append(float(record.get(key, fallback)))
                    except (TypeError, ValueError):
                        result.append(float('nan'))
                return np.asarray(result, dtype=float)

            stamp = values('record_stamp_s')
            time_s = stamp - stamp[0]
            antenna_x = values('AntennaX')
            antenna_y = values('AntennaY')
            altitude = values('AntennaAlt')
            quality = values('GGA_quality')
            satellites = values('GGA_sats')
            hdop = values('GGA_hdop')
            age = values('GNSSAge')
            fig, axes = plt.subplots(2, 2, figsize=(14, 9))
            valid_xy = np.isfinite(antenna_x) & np.isfinite(antenna_y)
            if np.any(valid_xy):
                axes[0, 0].plot(
                    antenna_x[valid_xy] - antenna_x[valid_xy][0],
                    antenna_y[valid_xy] - antenna_y[valid_xy][0], lw=1.0)
            axes[0, 0].set_xlabel('Relative antenna X (m)')
            axes[0, 0].set_ylabel('Relative antenna Y (m)')
            axes[0, 0].set_aspect('equal', adjustable='datalim')
            axes[0, 1].plot(time_s, altitude, lw=0.9)
            axes[0, 1].set_ylabel('Antenna altitude (m)')
            axes[1, 0].step(time_s, quality, where='post', label='GGA quality')
            axes[1, 0].plot(time_s, satellites, label='Satellites')
            axes[1, 0].plot(time_s, hdop, label='HDOP')
            axes[1, 0].set_ylabel('GNSS quality')
            axes[1, 0].legend(fontsize=8)
            axes[1, 1].plot(time_s, age, color='#d55e00', lw=0.9,
                            label='GNSS age')
            axes[1, 1].set_ylabel('Age (s)')
            axes[1, 1].legend(fontsize=8)
            for axis in axes.flat:
                axis.grid(True, alpha=0.3)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 1].set_xlabel('Time (s)')
            fig.suptitle('Qianxun GNSS Diagnostics')
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            path = os.path.join(figure_dir, 'gps_diagnostics.png')
            fig.savefig(path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            saved.append(path)

        motor_records = [record for record in self.motor_feedback_records
                         if isinstance(record, dict)]
        if len(motor_records) >= 2:
            def motor_values(key):
                return np.asarray([
                    float(record.get(key, float('nan')))
                    for record in motor_records], dtype=float)

            stamp = motor_values('stamp_s')
            time_s = stamp - stamp[0]
            fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            axes[0].plot(time_s, motor_values('raw_left'), label='raw left')
            axes[0].plot(time_s, motor_values('raw_right'), label='raw right')
            axes[0].set_ylabel('CAN int16')
            axes[1].plot(time_s, motor_values('left_rpm'), label='left rpm')
            axes[1].plot(time_s, motor_values('right_rpm'), label='right rpm')
            axes[1].set_ylabel('Signed RPM')
            if len(self.wheel_state_traj) >= 2:
                wheel = np.asarray(self.wheel_state_traj, dtype=float)
                wheel_time = wheel[:, 0] - stamp[0]
                axes[2].plot(wheel_time, wheel[:, 5], label='track speed')
                axes[2].plot(wheel_time, wheel[:, 6], label='yaw rate')
            axes[2].set_ylabel('m/s or rad/s')
            axes[2].set_xlabel('Time (s)')
            for axis in axes:
                axis.legend(fontsize=8)
                axis.grid(True, alpha=0.3)
            fig.suptitle('Raw Motor Feedback and Track Kinematics')
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
            path = os.path.join(figure_dir, 'motor_feedback.png')
            fig.savefig(path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            saved.append(path)

        return saved

    def _odom_cb(self, msg):
        if not self.nav_active:
            return
        p = msg.pose.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        self.actual_traj.append((
            msg.header.stamp.to_sec(),
            p.position.x, p.position.y, p.position.z,
            roll, pitch, yaw))

    def _control_odom_cb(self, msg):
        p = msg.pose.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        x = p.position.x
        y = p.position.y
        z = p.position.z
        # LIO-SAM incremental odometry is expressed in odom, while the plan
        # and move_base robot pose are expressed in map after fixed-map NDT.
        # Use the selected map-frame localization pose with the high-rate
        # incremental twist so CTE and response metrics share correct frames.
        if self.localization_mode == 'lio_sam' and self.actual_traj:
            selected = self.actual_traj[-1]
            x, y, z = selected[1], selected[2], selected[3]
            roll, pitch, yaw = selected[4], selected[5], selected[6]
        sample = (
            msg.header.stamp.to_sec(),
            x, y, z,
            roll, pitch, yaw,
            msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self.nav_active:
            self.control_traj.append(sample)

    def _ndt_target_odom_cb(self, msg):
        if not self.nav_active:
            return
        p = msg.pose.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        roll, pitch, yaw = tft.euler_from_quaternion(q)
        self.ndt_target_traj.append((
            msg.header.stamp.to_sec(), p.position.x, p.position.y,
            p.position.z, roll, pitch, yaw))

    def _cmd_vel_cb(self, msg):
        if self.nav_active:
            self.cmd_vel_traj.append((
                rospy.Time.now().to_sec(), msg.linear.x, msg.angular.z))

    def _mpc_cmd_vel_cb(self, msg):
        if self.nav_active:
            self.mpc_cmd_vel_traj.append((
                rospy.Time.now().to_sec(), msg.linear.x, msg.angular.z))

    def _mpc_diagnostics_cb(self, msg):
        if not self.nav_active:
            return
        if len(msg.data) != len(self.MPC_DIAGNOSTIC_FIELDS):
            rospy.logwarn_throttle(
                2.0, 'MPC diagnostics schema mismatch: received %d, '
                'expected %d', len(msg.data),
                len(self.MPC_DIAGNOSTIC_FIELDS))
            return
        self.mpc_diagnostics.append(tuple(float(value) for value in msg.data))

    def _mpc_prediction_diagnostics_cb(self, msg):
        if not self.nav_active:
            return
        fields_per_point = 8
        if len(msg.data) == 0 or len(msg.data) % fields_per_point != 0:
            rospy.logwarn_throttle(
                2.0, 'MPC prediction diagnostics schema mismatch: '
                'received %d values', len(msg.data))
            return
        stamp = rospy.Time.now().to_sec()
        for index in range(len(msg.data) // fields_per_point):
            begin = index * fields_per_point
            values = tuple(float(value) for value in
                           msg.data[begin:begin + fields_per_point])
            self.mpc_prediction_horizons.append(
                (stamp, index) + values)

    def _imu_cb(self, msg):
        """Retain every native /imu/data sample during the navigation run."""
        if not self.nav_active:
            return
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            roll, pitch, yaw = tft.euler_from_quaternion(quaternion)
        except (TypeError, ValueError, ZeroDivisionError):
            roll = pitch = yaw = float('nan')
        header_stamp = msg.header.stamp.to_sec()
        self.imu_raw_records.append((
            rospy.Time.now().to_sec(), header_stamp,
            int(msg.header.seq), str(msg.header.frame_id),
            q.x, q.y, q.z, q.w, roll, pitch, yaw,
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z, msg.linear_acceleration.x,
            msg.linear_acceleration.y, msg.linear_acceleration.z))

    def _motor_feedback_cb(self, msg):
        """Retain raw CAN int16 feedback and its signed-rpm conversion."""
        if not self.nav_active:
            return
        try:
            record = json.loads(msg.data)
            if not isinstance(record, dict):
                return
            record['record_stamp_s'] = rospy.Time.now().to_sec()
            self.motor_feedback_records.append(record)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _joint_states_cb(self, msg):
        """Aggregate actual left/right joint rates for offline slip analysis.

        This observer is deliberately evaluator-only.  Neither the wheel
        rates nor the derived kinematic velocity feed back into MPC.
        """
        if not self.nav_active or not msg.velocity:
            return
        left = []
        right = []
        for name, velocity in zip(msg.name, msg.velocity):
            if not np.isfinite(velocity):
                continue
            lowered = name.lower()
            if 'left' in lowered:
                left.append(float(velocity))
            elif 'right' in lowered:
                right.append(float(velocity))
        if not left or not right:
            return

        message_stamp = msg.header.stamp.to_sec()
        stamp = (message_stamp if message_stamp > 0.0
                 else rospy.Time.now().to_sec())
        left_radps = self.wheel_velocity_sign * float(np.mean(left))
        right_radps = self.wheel_velocity_sign * float(np.mean(right))
        left_mps = left_radps * self.wheel_radius
        right_mps = right_radps * self.wheel_radius
        kinematic_v = 0.5 * (left_mps + right_mps)
        kinematic_omega = ((right_mps - left_mps) /
                           self.wheel_separation)
        self.wheel_state_traj.append((
            stamp, left_radps, right_radps, left_mps, right_mps,
            kinematic_v, kinematic_omega, len(left), len(right)))

    def _make_gt_frame_anchor(self):
        """Build an absolute world->base0 map anchor from stationary GT.

        ``base_aligned_auto`` expresses the saved map in the initial
        ``base_link`` frame.  Consequently the initial vehicle base pose must
        map to position (0, 0, 0) and yaw 0.  Localization is intentionally
        absent from this calculation so its initial bias remains measurable.
        """
        if len(self.gt_anchor_buffer) < self.gt_anchor_min_samples:
            rospy.logwarn(
                'GT anchor rejected: only %d/%d buffered samples',
                len(self.gt_anchor_buffer), self.gt_anchor_min_samples)
            return None

        buffered = np.asarray(self.gt_anchor_buffer, dtype=float)
        end_time = float(buffered[-1, 0])
        samples = buffered[
            buffered[:, 0] >= end_time - self.gt_anchor_window]
        if len(samples) < self.gt_anchor_min_samples:
            rospy.logwarn(
                'GT anchor rejected: only %d/%d samples in %.1fs window',
                len(samples), self.gt_anchor_min_samples,
                self.gt_anchor_window)
            return None

        position_xy = np.median(samples[:, 1:3], axis=0)
        position_span = float(np.max(np.linalg.norm(
            samples[:, 1:3] - position_xy, axis=1)))
        yaw_samples = samples[:, 6]
        yaw0 = float(np.arctan2(
            np.mean(np.sin(yaw_samples)), np.mean(np.cos(yaw_samples))))
        yaw_errors = np.arctan2(
            np.sin(yaw_samples - yaw0), np.cos(yaw_samples - yaw0))
        yaw_span = float(np.max(np.abs(yaw_errors)))
        if (position_span > self.gt_anchor_max_position_span or
                yaw_span > self.gt_anchor_max_yaw_span):
            rospy.logwarn(
                'GT anchor rejected: vehicle not stationary '
                '(position span %.3fm > %.3fm or yaw span %.2fdeg > %.2fdeg)',
                position_span, self.gt_anchor_max_position_span,
                np.degrees(yaw_span),
                np.degrees(self.gt_anchor_max_yaw_span))
            return None

        yaw_offset = -yaw0
        c, s = np.cos(yaw_offset), np.sin(yaw_offset)
        rotation = np.array([[c, -s], [s, c]])
        translation_xy = -rotation.dot(position_xy)
        return {
            'x': float(translation_xy[0]),
            'y': float(translation_xy[1]),
            'z': -float(np.median(samples[:, 3])),
            'yaw': yaw_offset,
            'samples': int(len(samples)),
            'position_span': position_span,
            'yaw_span': yaw_span,
            'source': 'stationary_qianxun_reference',
        }

    def _gt_in_control_frame(self, gt):
        """Apply the fixed, estimator-independent world->map GT anchor.

        The external Qianxun reference is expressed in its local world frame while the plan and fixed
        map odometry are expressed in the initial ``base_link`` frame.  The
        same transform is held for the complete run, so both initial bias and
        subsequent localization drift remain visible.
        """
        if not self.align_gt_to_control_frame:
            return gt, None
        if self.gt_frame_anchor is None or len(gt) < 2:
            return None, None

        correction = self.gt_frame_anchor.copy()
        c, s = np.cos(correction['yaw']), np.sin(correction['yaw'])
        rotation = np.array([[c, -s], [s, c]])
        aligned = gt.copy()
        aligned[:, 1:3] = (
            gt[:, 1:3].dot(rotation.T) +
            np.array([correction['x'], correction['y']]))
        aligned[:, 3] = gt[:, 3] + correction['z']
        aligned[:, 6] = np.arctan2(
            np.sin(gt[:, 6] + correction['yaw']),
            np.cos(gt[:, 6] + correction['yaw']))
        return aligned, correction

    @staticmethod
    def _align_reference_at_start(reference_xy, reference_yaw,
                                  estimate_xy, estimate_yaw):
        """Remove only the initial SE(2) offset from a reference trajectory.

        Unlike Umeyama alignment, this transform uses exactly the first
        synchronized position and heading.  Consequently later disagreement
        remains visible and is reported as relative localization drift.
        """
        if (len(reference_xy) == 0 or len(reference_xy) != len(estimate_xy)
                or len(reference_yaw) != len(reference_xy)
                or len(estimate_yaw) != len(estimate_xy)):
            return None, None

        yaw_offset = float(np.arctan2(
            np.sin(estimate_yaw[0] - reference_yaw[0]),
            np.cos(estimate_yaw[0] - reference_yaw[0])))
        c, s = np.cos(yaw_offset), np.sin(yaw_offset)
        rotation = np.array([[c, -s], [s, c]])
        translation = estimate_xy[0] - rotation.dot(reference_xy[0])
        aligned = reference_xy.dot(rotation.T) + translation
        return aligned, {
            'translation_x': float(translation[0]),
            'translation_y': float(translation[1]),
            'yaw_deg': float(np.degrees(yaw_offset)),
            'method': 'first_synchronized_pose_only',
        }

    @staticmethod
    def _position_error_metrics(error):
        return {
            'mean_m': float(np.mean(error)),
            'rmse_m': float(np.sqrt(np.mean(error ** 2))),
            'max_m': float(np.max(error)),
        }

    @staticmethod
    def _derive_twist_from_pose(trajectory):
        """Estimate signed forward speed and yaw rate from pose differences."""
        trajectory = np.asarray(trajectory, dtype=float)
        count = len(trajectory)
        speed = np.zeros(count, dtype=float)
        yaw_rate = np.zeros(count, dtype=float)
        if count < 2:
            return speed, yaw_rate

        dt = np.diff(trajectory[:, 0])
        valid = dt > 1e-4
        dx = np.diff(trajectory[:, 1])
        dy = np.diff(trajectory[:, 2])
        yaw = np.unwrap(trajectory[:, 6])
        midpoint_yaw = 0.5 * (yaw[:-1] + yaw[1:])
        interval_speed = np.zeros(count - 1, dtype=float)
        interval_yaw_rate = np.zeros(count - 1, dtype=float)
        interval_speed[valid] = (
            (dx[valid] * np.cos(midpoint_yaw[valid])
             + dy[valid] * np.sin(midpoint_yaw[valid])) / dt[valid])
        interval_yaw_rate[valid] = np.diff(yaw)[valid] / dt[valid]

        speed[0] = interval_speed[0]
        speed[-1] = interval_speed[-1]
        yaw_rate[0] = interval_yaw_rate[0]
        yaw_rate[-1] = interval_yaw_rate[-1]
        if count > 2:
            speed[1:-1] = 0.5 * (interval_speed[:-1] + interval_speed[1:])
            yaw_rate[1:-1] = 0.5 * (
                interval_yaw_rate[:-1] + interval_yaw_rate[1:])
        return speed, yaw_rate

    def _reference_cb(self, msg):
        """Use Qianxun/YIS525 fused state as the real-vehicle reference.

        This is the real-vehicle reference source. In qianxun mode it records the same
        estimator used for control; in lio_sam mode it provides an independent
        GNSS-based trajectory for localization-error reporting.
        """
        t = rospy.Time.now().to_sec()
        try:
            state = json.loads(msg.data)
            if self.nav_active:
                record = dict(state)
                record['record_stamp_s'] = t
                self.reference_state_records.append(record)
            if not int(state.get('PositionValid', 0)):
                return
            x = float(state['UTM_x'])
            y = float(state['UTM_y'])
            z = float(state.get('Z', 0.0))
            roll = np.radians(float(state.get('Roll', 0.0)))
            pitch = np.radians(float(state.get('Pitch', 0.0)))
            yaw = np.radians(float(state['Head']) + 90.0)
            yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
            yaw_rate = float(state.get('Omega', 0.0))
            velocity_x = float(state.get('V_x', 0.0))
            velocity_y = float(state.get('V_y', 0.0))
            forward_speed = (velocity_x * np.cos(yaw)
                             + velocity_y * np.sin(yaw))
            values = (x, y, z, roll, pitch, yaw, yaw_rate, forward_speed)
            if not all(np.isfinite(value) for value in values):
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        sample = (t, x, y, z, roll, pitch, yaw)
        self.latest_gt_pose = sample
        if not self.nav_active or t - self.last_gt_time < 0.1:
            return
        self.last_gt_time = t
        self.gt_traj.append(sample)
        self.gt_twist_traj.append((t, forward_speed, yaw_rate))

    def _vehicle_sensor_cb(self, msg):
        if not self.nav_active:
            return
        try:
            record = json.loads(msg.data)
            if not isinstance(record, dict):
                return
            record['record_stamp_s'] = rospy.Time.now().to_sec()
            self.vehicle_sensor_records.append(record)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def _progress_cb(self, _event):
        if (not self.nav_active or not self.planned_path
                or not self.control_traj):
            return

        plan = np.asarray(self.planned_path, dtype=float)[:, :2]
        position = np.asarray(self.control_traj[-1][1:3], dtype=float)
        elapsed = ((rospy.Time.now() - self.start_time).to_sec()
                   if self.start_time else 0.0)

        if len(plan) < 2:
            progress = 100.0
            current_cte = float(np.linalg.norm(position - plan[0]))
        else:
            seg_v = plan[1:] - plan[:-1]
            seg_len2 = np.sum(seg_v * seg_v, axis=1)
            seg_len = np.sqrt(seg_len2)
            rel = position - plan[:-1]
            t = np.zeros(len(seg_v))
            valid = seg_len2 > 1e-12
            t[valid] = (np.sum(rel[valid] * seg_v[valid], axis=1)
                        / seg_len2[valid])
            t = np.clip(t, 0.0, 1.0)
            nearest = plan[:-1] + t[:, None] * seg_v
            distance2 = np.sum((nearest - position) ** 2, axis=1)
            idx = int(np.argmin(distance2))
            current_cte = float(np.sqrt(distance2[idx]))
            total = float(np.sum(seg_len))
            along = float(np.sum(seg_len[:idx]) + t[idx] * seg_len[idx])
            progress = 100.0 * along / total if total > 1e-9 else 100.0

        goal = (np.asarray(self.goal_pose, dtype=float)
                if self.goal_pose is not None else plan[-1])
        remaining = float(np.linalg.norm(position - goal))
        self._nav_output(
            '[NAV] 进度=%5.1f%% | 用时=%5.1fs | 距目标=%.2fm | 当前CTE=%.2fm',
            np.clip(progress, 0.0, 100.0), elapsed, remaining, current_cte)

    def _status_cb(self, msg):
        if not self.nav_active or not msg.status_list:
            return

        current = msg.status_list[-1].status
        if current in (3, 4, 5, 2) and self.last_status == 1:
            elapsed = (rospy.Time.now() - self.start_time).to_sec() \
                if self.start_time else 0
            names = {3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED', 2: 'PREEMPTED'}
            status_text = names.get(current, 'STATUS_%d' % current)

            self.nav_active = False
            self.run_count += 1

            self._nav_output(
                '[NAV] 导航结束 | 状态=%s | 用时=%.1fs | 正在统计误差',
                status_text, elapsed)
            rospy.loginfo('Navigation %s (%.1fs). Generating plot #%d...',
                          status_text, elapsed, self.run_count)

            if (self.planned_path and len(self.actual_traj) > 10
                    and len(self.control_traj) > 10):
                self._generate_plots(status_text, elapsed)
            else:
                run_dir = os.path.join(
                    self.output_dir, 'nav_%d' % self.run_count)
                os.makedirs(run_dir, exist_ok=True)
                raw_csv_files = self._export_raw_csv(run_dir)
                diagnostic_files = self._generate_mpc_diagnostic_plots(
                    run_dir, self.start_time.to_sec()
                    if self.start_time else 0.0)
                sensor_files = self._generate_sensor_diagnostic_plots(run_dir)
                if self.planned_path:
                    stage_files = self._generate_planner_stage_plots(
                        run_dir,
                        'Nav #%d | %s' % (self.run_count, status_text),
                        np.asarray(self.planned_path, dtype=float)[:, :2])
                    for stage_file in stage_files:
                        rospy.loginfo('Saved: %s', stage_file)
                for raw_file in raw_csv_files:
                    rospy.loginfo('Saved: %s', raw_file)
                for diagnostic_file in diagnostic_files:
                    rospy.loginfo('Saved: %s', diagnostic_file)
                for sensor_file in sensor_files:
                    rospy.loginfo('Saved: %s', sensor_file)
                self._nav_output(
                    '[NAV] 无法生成完整结果 | 路径=%s | 定位点=%d | 控制点=%d',
                    '有' if self.planned_path else '无',
                    len(self.actual_traj), len(self.control_traj))
                rospy.logwarn('Insufficient data (plan=%s, mapping=%d, control=%d pts)',
                              'yes' if self.planned_path else 'no',
                              len(self.actual_traj), len(self.control_traj))

        self.last_status = current

    def _shutdown_cb(self):
        """Generate the current run's plots when roslaunch receives Ctrl+C."""
        if (not self.nav_active or self.shutdown_plot_started):
            return

        self.shutdown_plot_started = True
        self.nav_active = False
        elapsed = ((rospy.Time.now() - self.start_time).to_sec()
                   if self.start_time else 0.0)
        self.run_count += 1
        status_text = 'INTERRUPTED'

        try:
            rospy.loginfo(
                'Navigation interrupted (%.1fs). Generating plot #%d...',
                elapsed, self.run_count)
            if (self.planned_path and len(self.actual_traj) > 10
                    and len(self.control_traj) > 10):
                self._generate_plots(status_text, elapsed)
            else:
                run_dir = os.path.join(
                    self.output_dir, 'nav_%d' % self.run_count)
                os.makedirs(run_dir, exist_ok=True)
                raw_csv_files = self._export_raw_csv(run_dir)
                diagnostic_files = self._generate_mpc_diagnostic_plots(
                    run_dir, self.start_time.to_sec()
                    if self.start_time else 0.0)
                sensor_files = self._generate_sensor_diagnostic_plots(run_dir)
                if self.planned_path:
                    stage_files = self._generate_planner_stage_plots(
                        run_dir,
                        'Nav #%d | %s' % (self.run_count, status_text),
                        np.asarray(self.planned_path, dtype=float)[:, :2])
                    for stage_file in stage_files:
                        rospy.loginfo('Saved: %s', stage_file)
                else:
                    rospy.logwarn(
                        'Ctrl+C: no global path, planner/raw CSV retained')
                for raw_file in raw_csv_files:
                    rospy.loginfo('Saved: %s', raw_file)
                for diagnostic_file in diagnostic_files:
                    rospy.loginfo('Saved: %s', diagnostic_file)
                for sensor_file in sensor_files:
                    rospy.loginfo('Saved: %s', sensor_file)
        except Exception as exc:
            rospy.logerr(
                'Failed to save Ctrl+C navigation evaluation: %s', exc)

    # ---- terrain queries ----

    def _query_dem(self, x, y):
        if self.dem is None:
            return 0.0
        c = int(np.clip((x - self.dem_ox) / self.dem_res, 0, self.dem_cols - 1))
        r = int(np.clip((y - self.dem_oy) / self.dem_res, 0, self.dem_rows - 1))
        v = self.dem[r, c]
        return float(v) if not np.isnan(v) else 0.0

    def _path_metrics(self, path_xyz):
        pts = np.array(path_xyz)
        dx = np.diff(pts[:, 0])
        dy = np.diff(pts[:, 1])
        dz = np.diff(pts[:, 2])
        ds = np.sqrt(dx ** 2 + dy ** 2)
        dist = np.concatenate([[0], np.cumsum(ds)])

        # ROS uses a right-handed body frame (x forward, y left, z up), so a
        # positive rotation about +y points the vehicle nose downward. A path
        # segment with dz < 0 is therefore positive vehicle pitch. Keep the
        # DEM-derived plan convention consistent with the external reference.
        pitch = -np.arctan2(dz, np.maximum(ds, 1e-6))

        roll = np.zeros(len(pts) - 1)
        if self.dem is not None:
            hw = self.vehicle_half_width
            for i in range(len(pts) - 1):
                theta = np.arctan2(dy[i], dx[i])
                px, py = -np.sin(theta) * hw, np.cos(theta) * hw
                x, y = pts[i + 1, 0], pts[i + 1, 1]
                zl = self._query_dem(x + px, y + py)
                zr = self._query_dem(x - px, y - py)
                roll[i] = np.arctan2(zl - zr, 2 * hw)

        return dist, pitch, roll

    @staticmethod
    def _path_curvature_metrics(path_xy):
        """Return arc-length samples, signed curvature, mean |k| and max |k|.

        Curvature is computed from each three-point circumcircle after removing
        consecutive duplicate XY samples.  The mean is weighted by the local
        support arc length, so all three planners are compared independently
        of their output waypoint density.  Units are 1/m.
        """
        points = np.asarray(path_xy, dtype=float)
        if points.ndim != 2 or points.shape[0] < 3:
            return np.asarray([]), np.asarray([]), 0.0, 0.0
        points = points[:, :2]
        keep = np.ones(points.shape[0], dtype=bool)
        keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-6
        points = points[keep]
        if points.shape[0] < 3:
            return np.asarray([]), np.asarray([]), 0.0, 0.0

        # Resample all planners at the same 0.10 m arc-length interval before
        # differentiating.  Otherwise a planner publishing more waypoints can
        # appear artificially rougher than an identical sparse polyline.
        original_segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
        original_s = np.concatenate(([0.0], np.cumsum(original_segments)))
        total_length = float(original_s[-1])
        if total_length <= 0.20:
            return np.asarray([]), np.asarray([]), 0.0, 0.0
        sample_s = np.arange(0.0, total_length, 0.10)
        if sample_s.size == 0 or total_length - sample_s[-1] > 1e-6:
            sample_s = np.append(sample_s, total_length)
        points = np.column_stack((
            np.interp(sample_s, original_s, points[:, 0]),
            np.interp(sample_s, original_s, points[:, 1])))

        first = points[1:-1] - points[:-2]
        second = points[2:] - points[1:-1]
        chord = points[2:] - points[:-2]
        first_length = np.linalg.norm(first, axis=1)
        second_length = np.linalg.norm(second, axis=1)
        chord_length = np.linalg.norm(chord, axis=1)
        denominator = first_length * second_length * chord_length
        cross = first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]
        curvature = np.zeros_like(cross)
        valid = denominator > 1e-9
        curvature[valid] = 2.0 * cross[valid] / denominator[valid]

        support = 0.5 * (first_length + second_length)
        total_support = float(np.sum(support))
        mean_abs = (float(np.sum(np.abs(curvature) * support) / total_support)
                    if total_support > 1e-9 else 0.0)
        max_abs = (float(np.max(np.abs(curvature)))
                   if curvature.size else 0.0)
        segment_length = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_length)))
        curvature_s = cumulative[1:-1]
        return curvature_s, curvature, mean_abs, max_abs

    def _cross_track_error(self, plan_xy, traj):
        """Return lateral distance to the nearest path segment centerline.

        Segment selection uses the finite segment distance, but the reported
        error is perpendicular distance to that segment's supporting line.
        This prevents motion slightly beyond the final waypoint from being
        misreported as lateral CTE; endpoint overshoot is reported separately.
        """
        plan = np.array(plan_xy)
        errs = np.empty(len(traj))
        if len(plan) < 2:
            for i in range(len(traj)):
                errs[i] = np.hypot(plan[0, 0] - traj[i, 1],
                                   plan[0, 1] - traj[i, 2])
            return errs

        seg_a = plan[:-1]
        seg_v = plan[1:] - plan[:-1]
        seg_len2 = np.sum(seg_v * seg_v, axis=1)
        valid = seg_len2 > 1e-12
        for i in range(len(traj)):
            p = np.array([traj[i, 1], traj[i, 2]])
            rel = p - seg_a
            t = np.zeros(len(seg_a))
            t[valid] = np.sum(rel[valid] * seg_v[valid], axis=1) / seg_len2[valid]
            t_clamped = np.clip(t, 0.0, 1.0)
            nearest = seg_a + t_clamped[:, None] * seg_v
            seg_idx = int(np.argmin(np.sum((nearest - p) ** 2, axis=1)))

            if not valid[seg_idx]:
                errs[i] = np.linalg.norm(p - seg_a[seg_idx])
            else:
                cross = (seg_v[seg_idx, 0] * rel[seg_idx, 1]
                         - seg_v[seg_idx, 1] * rel[seg_idx, 0])
                errs[i] = abs(cross) / np.sqrt(seg_len2[seg_idx])
        return errs

    @staticmethod
    def _path_progress_series(plan_xy, trajectory_xy):
        """Project every trajectory point onto the plan arc length."""
        plan = np.asarray(plan_xy, dtype=float)
        trajectory = np.asarray(trajectory_xy, dtype=float)
        if len(plan) < 2:
            return np.full(len(trajectory), 100.0)

        segment = plan[1:] - plan[:-1]
        segment_len2 = np.sum(segment * segment, axis=1)
        segment_len = np.sqrt(segment_len2)
        cumulative = np.concatenate([[0.0], np.cumsum(segment_len)])
        total = cumulative[-1]
        progress = np.zeros(len(trajectory), dtype=float)
        valid = segment_len2 > 1e-12
        for index, point in enumerate(trajectory):
            relative = point - plan[:-1]
            ratio = np.zeros(len(segment), dtype=float)
            ratio[valid] = (np.sum(relative[valid] * segment[valid], axis=1)
                            / segment_len2[valid])
            ratio = np.clip(ratio, 0.0, 1.0)
            projected = plan[:-1] + ratio[:, None] * segment
            nearest_index = int(np.argmin(np.sum(
                (projected - point) ** 2, axis=1)))
            along = (cumulative[nearest_index]
                     + ratio[nearest_index] * segment_len[nearest_index])
            progress[index] = 100.0 * along / total if total > 1e-9 else 100.0
        return np.clip(progress, 0.0, 100.0)

    def _goal_metrics(self, plan_xy, final_xy, goal_xy=None):
        """Return position, along-track and lateral error at the goal."""
        plan = np.asarray(plan_xy)
        final = np.asarray(final_xy)
        goal = plan[-1] if goal_xy is None else np.asarray(goal_xy)
        delta = final - goal

        tangent = plan[-1] - plan[-2]
        norm = np.linalg.norm(tangent)
        if norm < 1e-9:
            return np.linalg.norm(delta), 0.0, np.linalg.norm(delta)
        tangent /= norm
        along = float(np.dot(delta, tangent))
        lateral = float(abs(tangent[0] * delta[1] - tangent[1] * delta[0]))
        return float(np.linalg.norm(delta)), along, lateral

    @staticmethod
    def _directed_path_gap(points, polyline):
        """Maximum point-to-polyline distance for a path-chain audit."""
        points = np.asarray(points, dtype=float)
        polyline = np.asarray(polyline, dtype=float)
        if len(points) == 0 or len(polyline) == 0:
            return float('inf')
        if len(polyline) == 1:
            return float(np.max(np.linalg.norm(points - polyline[0], axis=1)))
        start = polyline[:-1]
        segment = polyline[1:] - start
        length2 = np.sum(segment * segment, axis=1)
        maximum = 0.0
        for point in points:
            relative = point - start
            projection = np.zeros(len(segment), dtype=float)
            valid = length2 > 1e-12
            projection[valid] = (
                np.sum(relative[valid] * segment[valid], axis=1)
                / length2[valid])
            projection = np.clip(projection, 0.0, 1.0)
            nearest = start + projection[:, None] * segment
            maximum = max(
                maximum,
                float(np.min(np.linalg.norm(nearest - point, axis=1))))
        return maximum

    @classmethod
    def _path_chain_gap(cls, first, second):
        """Symmetric geometric gap, insensitive to path resampling."""
        return max(cls._directed_path_gap(first, second),
                   cls._directed_path_gap(second, first))

    def _generate_planner_stage_plots(self, run_dir, title, final_plan):
        """Save coarse channels, corridor candidates and final path alone."""
        artifact_dirs = self._run_artifact_dirs(run_dir)
        figure_dir = artifact_dirs['figures']
        json_dir = artifact_dirs['json']
        stage_title = title.split('\n')[0]
        saved = []
        metrics_by_index = {
            int(item.get('index', 0)): item
            for item in self.pareto_candidates}
        selected_index = int(self.pareto_summary.get('selected', 0) or 0)
        selected_path = None
        if 1 <= selected_index <= len(self.fine_candidate_paths):
            selected_path = self.fine_candidate_paths[selected_index - 1]['points']
        chain_gap = (self._path_chain_gap(selected_path, final_plan)
                     if selected_path is not None else None)
        # Keep detailed planner audit information out of the title.  A single
        # very long title becomes part of Matplotlib's tight bounding box and
        # compresses the equal-aspect map into a tiny panel (observed in
        # nav_7).  Three bounded lines preserve the data without changing the
        # map/colorbar geometry.  Spatial-signature counts make it explicit
        # whether a missing corridor was never explored, was merged as a
        # geometric duplicate, or was rejected later by the fine search.
        search_mode = self.pareto_summary.get(
            'risk_anchor_status', 'unavailable')
        coarse_audit = (
            'search=%s | channels=%d | goal: labels=%d, signatures=%d, '
            'topologies=%d | pool=%d -> %d\n'
            'labels: generated=%d, dominance-pruned=%d, cap-pruned=%d, '
            'signature-admitted=%d, signature-protected=%d\n'
            'fine-refine=%d/%d, pruned=%d | duplicates=%d | detours=%d | stop=%s | '
            'elapsed=%.2fs / %.2fs' %
            (search_mode,
             int(self.pareto_summary.get('coarse_channel_count', 0)),
             int(self.pareto_summary.get('goal_labels', 0)),
             int(self.pareto_summary.get('goal_signature_count', 0)),
             int(self.pareto_summary.get('goal_topology_count', 0)),
             int(self.pareto_summary.get('generated_pool_count', 0)),
             int(self.pareto_summary.get('coarse_channel_count', 0)),
             int(self.pareto_summary.get('label_generated', 0)),
             int(self.pareto_summary.get('label_dominance_pruned', 0)),
             int(self.pareto_summary.get('label_cap_pruned', 0)),
             int(self.pareto_summary.get(
                 'topology_signature_labels_admitted', 0)),
             int(self.pareto_summary.get(
                 'topology_signature_cap_protected', 0)),
             int(self.pareto_summary.get('fine_refinement_selected', 0)),
             int(self.pareto_summary.get('fine_refinement_limit', 0)),
             int(self.pareto_summary.get('fine_budget_pruned', 0)),
             int(self.pareto_summary.get('duplicate_rejections', 0)),
             int(self.pareto_summary.get('detour_rejections', 0)),
             self.pareto_summary.get('coarse_stop_reason', 'unavailable'),
             float(self.pareto_summary.get('coarse_elapsed_s', 0.0)),
             float(self.pareto_summary.get('coarse_time_budget_s', 0.0))))

        # 1) Accepted coarse-search channels.
        fig, ax = plt.subplots(figsize=(12, 8))
        self._draw_terrain_background(ax)
        for index, path in enumerate(self.coarse_paths):
            points = path['points']
            ax.plot(points[:, 0], points[:, 1], lw=2.2,
                    color=path['color'], zorder=7,
                    label='Coarse channel %d' % (index + 1))
        if not self.coarse_paths:
            ax.text(0.5, 0.5, 'No coarse channel generated',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=12, color='crimson')
        ax.plot(final_plan[0, 0], final_plan[0, 1], 'o', color='#009e73',
                ms=9, label='Start', zorder=9)
        ax.plot(final_plan[-1, 0], final_plan[-1, 1], '*', color='#d62728',
                ms=13, label='Goal', zorder=9)
        ax.set_title(stage_title + '\nCoarse Search Channels',
                     fontsize=11, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.text(0.01, -0.16, coarse_audit, transform=ax.transAxes,
                ha='left', va='top', fontsize=8, family='monospace',
                linespacing=1.35,
                bbox=dict(boxstyle='round,pad=0.35', facecolor='white',
                          edgecolor='#59636e', alpha=0.92))
        fig.subplots_adjust(left=0.08, right=0.90, bottom=0.25, top=0.88)
        coarse_file = os.path.join(figure_dir, 'coarse_search_paths.png')
        fig.savefig(coarse_file, dpi=150)
        plt.close(fig)
        saved.append(coarse_file)

        # 2) Coarse centre-lines, independent metric corridors and the
        # corresponding fine-search candidates.  Keeping all three stages in
        # one figure makes the coarse -> corridor -> fine -> Pareto chain
        # directly auditable.
        fig, ax = plt.subplots(figsize=(12, 8))
        self._draw_terrain_background(ax)
        for index, corridor in enumerate(self.corridor_paths):
            points = corridor['points']
            ax.scatter(points[:, 0], points[:, 1], s=5.0, marker='s',
                       linewidths=0.0, color=corridor['color'], alpha=0.16,
                       zorder=2)
        corridor_half_width = float(
            self.pareto_summary.get('corridor_max_width_m', 0.0))
        for index, path in enumerate(self.coarse_paths):
            points = path['points']
            width_text = ((' (half-width %.1fm)' % corridor_half_width)
                          if corridor_half_width > 0.0 else '')
            ax.plot(points[:, 0], points[:, 1], '--', lw=1.6,
                    color=path['color'], alpha=0.95, zorder=6,
                    label='Coarse corridor %d%s' %
                          (index + 1, width_text))
        for index, path in enumerate(self.fine_candidate_paths):
            points = path['points']
            candidate_id = index + 1
            audit = metrics_by_index.get(candidate_id, {})
            state = []
            if audit.get('pareto'):
                state.append('Pareto')
            zero_exposure = audit.get(
                'zero_high_risk_exposure', audit.get('safe', False))
            if zero_exposure:
                state.append('zero-exposure')
            if not audit.get('safety_feasible', True):
                state.append('safety-rejected')
            if audit.get('selected'):
                state.append('selected')
            suffix = ' [%s]' % ', '.join(state) if state else ''
            selected = bool(audit.get('selected'))
            ax.plot(points[:, 0], points[:, 1],
                    lw=3.0 if selected else 2.0,
                    color=path['color'], zorder=8,
                    label='Fine candidate %d%s' % (candidate_id, suffix))
        if not self.corridor_paths:
            ax.text(0.5, 0.5, 'No independent corridor generated',
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=12, color='crimson')
        ax.plot(final_plan[0, 0], final_plan[0, 1], 'o', color='#009e73',
                ms=9, label='Start', zorder=9)
        ax.plot(final_plan[-1, 0], final_plan[-1, 1], '*', color='#d62728',
                ms=13, label='Goal', zorder=9)
        ax.set_title(stage_title +
                     '\nCoarse Corridors and Fine Candidates',
                     fontsize=10, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=2)
        if self.pareto_candidates:
            audit_lines = []
            for item in self.pareto_candidates:
                score = float(item.get(
                    'compromise_score', item.get('ideal_distance', -1.0)))
                score_text = 'n/a' if score < 0.0 else '%.3f' % score
                audit_lines.append(
                    'P%d/C%d [%s]: L=%.2fm, meanR=%.3f, peakR=%.3f, '
                    'tailR=%.3f, highR=%.1f%%, P95=(%.1f,%.1f)deg, '
                    'Dreg=%.3f, Rreg=%.3f, '
                    'front=%s, score=%s%s' %
                    (int(item.get('index', 0)), int(item.get('channel', -1)),
                     item.get('role', 'unknown').replace('_', ' '),
                     float(item.get('length', 0.0)),
                     float(item.get('mean_risk', 0.0)),
                     float(item.get('peak_risk', 0.0)),
                       float(item.get('tail_risk', 0.0)),
                       100.0 * float(item.get('high_risk_fraction', 0.0)),
                       float(item.get('p95_abs_pitch_deg', 0.0)),
                       float(item.get('p95_abs_roll_deg', 0.0)),
                      float(item.get('distance_regret', 0.0)),
                      float(item.get('terrain_regret', 0.0)),
                      'yes' if item.get('pareto') else 'no', score_text,
                      ', SELECTED' if item.get('selected') else ''))
            ax.text(
                0.01, 0.01, '\n'.join(audit_lines), transform=ax.transAxes,
                ha='left', va='bottom', fontsize=7,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.82),
                zorder=12)
        fig.tight_layout()
        corridor_file = os.path.join(
            figure_dir, 'multi_corridor_results.png')
        fig.savefig(corridor_file, dpi=150, bbox_inches='tight')
        plt.close(fig)
        saved.append(corridor_file)

        # 3) Quantitative comparison of every retained fine candidate. Length
        # has its own axis because metres and normalized risk are not directly
        # commensurate; putting them on one grouped axis would visually hide
        # the risk differences that drive the Pareto decision.
        if self.pareto_candidates:
            candidates = self.pareto_candidates
            candidate_labels = [
                'P%d/C%d%s%s%s' %
                (int(item.get('index', index + 1)),
                 int(item.get('channel', -1)),
                  ' [zero-exposure]' if item.get(
                      'zero_high_risk_exposure',
                      item.get('safe', False)) else '',
                  ' [safety-rejected]' if not item.get(
                      'safety_feasible', True) else '',
                  ' (selected)' if item.get('selected') else '')
                for index, item in enumerate(candidates)]
            candidate_colors = []
            for index, item in enumerate(candidates):
                candidate_id = int(item.get('index', index + 1))
                if 1 <= candidate_id <= len(self.fine_candidate_paths):
                    candidate_colors.append(
                        self.fine_candidate_paths[candidate_id - 1]['color'])
                else:
                    candidate_colors.append(
                        plt.get_cmap('tab10')(index % 10))

            fig, axes = plt.subplots(2, 2, figsize=(18, 10.5))
            axes = axes.ravel()
            lengths = [float(item.get('length', 0.0))
                       for item in candidates]
            length_bars = axes[0].bar(
                np.arange(len(candidates)), lengths,
                color=candidate_colors, alpha=0.88)
            axes[0].set_xticks(np.arange(len(candidates)))
            axes[0].set_xticklabels(candidate_labels, rotation=15,
                                    ha='right')
            axes[0].set_ylabel('Path length (m)')
            axes[0].set_title('Fine-candidate path length')
            axes[0].grid(True, axis='y', alpha=0.25)
            for bar, value in zip(length_bars, lengths):
                axes[0].text(bar.get_x() + 0.5 * bar.get_width(),
                             bar.get_height(), '%.2f' % value,
                             ha='center', va='bottom', fontsize=8)

            risk_names = ['Mean risk', 'Peak risk', 'Tail CVaR',
                          'High-risk fraction']
            risk_values = np.asarray([
                [float(item.get('mean_risk', 0.0)),
                 float(item.get('peak_risk', 0.0)),
                 float(item.get('tail_risk', 0.0)),
                 float(item.get('high_risk_fraction', 0.0))]
                for item in candidates], dtype=float)
            x_positions = np.arange(len(risk_names), dtype=float)
            group_width = 0.82
            bar_width = group_width / max(1, len(candidates))
            for index, (label, color) in enumerate(
                    zip(candidate_labels, candidate_colors)):
                positions = (x_positions - 0.5 * group_width +
                             (index + 0.5) * bar_width)
                bars = axes[1].bar(
                    positions, risk_values[index], width=bar_width,
                    color=color, alpha=0.88, label=label)
                for bar, value in zip(bars, risk_values[index]):
                    axes[1].text(
                        bar.get_x() + 0.5 * bar.get_width(),
                        bar.get_height(), '%.3f' % value,
                        ha='center', va='bottom', fontsize=6, rotation=90)
            axes[1].set_xticks(x_positions)
            axes[1].set_xticklabels(risk_names)
            axes[1].set_ylim(
                0.0, max(1.05, 1.15 * float(np.max(risk_values))))
            axes[1].set_ylabel('Normalized terrain risk')
            axes[1].set_title('Mean, local and exposure risk')
            axes[1].grid(True, axis='y', alpha=0.25)
            axes[1].legend(fontsize=8)

            regret_names = ['Distance regret', 'Terrain regret',
                            'Compromise score']
            regret_values = np.asarray([
                [float(item.get('distance_regret', 0.0)),
                 float(item.get('terrain_regret', 0.0)),
                 max(0.0, float(item.get(
                     'compromise_score', item.get('ideal_distance', -1.0))))]
                for item in candidates], dtype=float)
            regret_x = np.arange(len(regret_names), dtype=float)
            for index, (label, color) in enumerate(
                    zip(candidate_labels, candidate_colors)):
                positions = (regret_x - 0.5 * group_width +
                             (index + 0.5) * bar_width)
                bars = axes[2].bar(
                    positions, regret_values[index], width=bar_width,
                    color=color, alpha=0.88, label=label)
                for bar, value in zip(bars, regret_values[index]):
                    axes[2].text(
                        bar.get_x() + 0.5 * bar.get_width(),
                        bar.get_height(), '%.3f' % value,
                        ha='center', va='bottom', fontsize=6, rotation=90)
            axes[2].set_xticks(regret_x)
            axes[2].set_xticklabels(regret_names)
            axes[2].set_ylabel('Normalized regret')
            axes[2].set_title('Pareto compromise decision')
            axes[2].grid(True, axis='y', alpha=0.25)
            axes[2].legend(fontsize=8)

            attitude_names = ['Pitch P95', 'Pitch max',
                              'Roll P95', 'Roll max']
            attitude_values = np.asarray([
                [float(item.get('p95_abs_pitch_deg', 0.0)),
                 float(item.get('max_abs_pitch_deg', 0.0)),
                 float(item.get('p95_abs_roll_deg', 0.0)),
                 float(item.get('max_abs_roll_deg', 0.0))]
                for item in candidates], dtype=float)
            attitude_x = np.arange(len(attitude_names), dtype=float)
            for index, (label, color) in enumerate(
                    zip(candidate_labels, candidate_colors)):
                positions = (attitude_x - 0.5 * group_width +
                             (index + 0.5) * bar_width)
                bars = axes[3].bar(
                    positions, attitude_values[index], width=bar_width,
                    color=color, alpha=0.88, label=label)
                for bar, value in zip(bars, attitude_values[index]):
                    axes[3].text(
                        bar.get_x() + 0.5 * bar.get_width(),
                        bar.get_height(), '%.1f' % value,
                        ha='center', va='bottom', fontsize=6, rotation=90)
            axes[3].set_xticks(attitude_x)
            axes[3].set_xticklabels(attitude_names)
            axes[3].set_ylabel('Predicted attitude (deg)')
            axes[3].set_title('Directional terrain attitude')
            axes[3].grid(True, axis='y', alpha=0.25)
            axes[3].legend(fontsize=8)
            fig.suptitle(stage_title +
                         '\nFine Candidate Metric Comparison',
                         fontsize=11, fontweight='bold')
            fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
            metric_file = os.path.join(
                figure_dir, 'candidate_metrics_comparison.png')
            fig.savefig(metric_file, dpi=150, bbox_inches='tight')
            plt.close(fig)
            saved.append(metric_file)

        # 4) Final path selected by the Pareto decision only.
        fig, ax = plt.subplots(figsize=(12, 8))
        self._draw_terrain_background(ax)
        if selected_path is not None:
            ax.plot(selected_path[:, 0], selected_path[:, 1], '--',
                    color='#d81b60', lw=4.0, alpha=0.55,
                    label='Selected fine candidate %d' % selected_index,
                    zorder=6)
        ax.plot(final_plan[:, 0], final_plan[:, 1], color='#0057d9',
                lw=2.5, label='Selected global path', zorder=7)
        ax.plot(final_plan[0, 0], final_plan[0, 1], 'o', color='#009e73',
                ms=9, label='Start', zorder=9)
        ax.plot(final_plan[-1, 0], final_plan[-1, 1], '*', color='#d62728',
                ms=13, label='Goal', zorder=9)
        selection_reason = self.pareto_summary.get('reason', 'unavailable')
        gap_text = ('chain gap %.3fm' % chain_gap
                    if chain_gap is not None else 'chain gap unavailable')
        ax.set_title(stage_title + '\nFinal Selected Global Path | ' +
                     'candidate %d | %s | %s' %
                     (selected_index, selection_reason, gap_text),
                     fontsize=10, fontweight='bold')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        final_file = os.path.join(figure_dir, 'final_planned_path.png')
        fig.savefig(final_file, dpi=150, bbox_inches='tight')
        plt.close(fig)
        saved.append(final_file)

        audit_file = os.path.join(json_dir, 'pareto_decision.json')
        with open(audit_file, 'w') as stream:
            json.dump({
                'map': self.map_identity,
                'localization_mode': self.localization_mode,
                'plan_stamp_ns': self.plan_stamp_ns,
                'summary': self.pareto_summary,
                'candidates': self.pareto_candidates,
                'selected_path_chain_max_gap_m': chain_gap,
                'chain_consistent_5cm': (
                    chain_gap is not None and chain_gap <= 0.05),
            }, stream, indent=2, sort_keys=True)
        saved.append(audit_file)
        if chain_gap is not None and chain_gap > 0.05:
            rospy.logwarn(
                'Planner chain mismatch at stamp %s: selected candidate %d '
                'and published plan differ by %.3fm',
                str(self.plan_stamp_ns), selected_index, chain_gap)
        return saved

    # ---- plot generation ----

    def _generate_plots(self, status, elapsed):
        plan = np.array(self.planned_path)
        traj = np.array(self.actual_traj)
        control = np.array(self.control_traj)

        plan_dist, plan_pitch, plan_roll = self._path_metrics(self.planned_path)
        plan_curvature_s, plan_curvature, plan_mean_abs_curvature, \
            plan_max_abs_curvature = self._path_curvature_metrics(plan[:, :2])
        traj_xyz = [(r[1], r[2], r[3]) for r in self.actual_traj]
        traj_dist, _, _ = self._path_metrics(traj_xyz)
        cte = self._cross_track_error(plan[:, :2], traj)
        traj_time = traj[:, 0] - traj[0, 0]

        control_xyz = control[:, 1:4]
        control_dist, _, _ = self._path_metrics(control_xyz)
        control_cte = self._cross_track_error(plan[:, :2], control)
        control_time = control[:, 0] - control[0, 0]

        # Interpolate the Qianxun reference to the LIO-SAM timestamps so control
        # tracking error can be separated from localization error.
        gt = None
        gt_xy = None
        gt_xy_control = None
        gt_cte = None
        gt_total = None
        gt_dist = None
        gt_roll_control = None
        gt_pitch_control = None
        gt_yaw_control = None
        gt_frame_correction = None
        if len(self.gt_traj) > 10:
            raw_gt = np.array(self.gt_traj)
            gt, gt_frame_correction = self._gt_in_control_frame(raw_gt)
        if gt is not None:
            gt_x_interp = np.interp(traj[:, 0], gt[:, 0], gt[:, 1])
            gt_y_interp = np.interp(traj[:, 0], gt[:, 0], gt[:, 2])
            gt_z_interp = np.interp(traj[:, 0], gt[:, 0], gt[:, 3])
            gt_xy = np.column_stack([gt_x_interp, gt_y_interp])
            gt_x_control = np.interp(control[:, 0], gt[:, 0], gt[:, 1])
            gt_y_control = np.interp(control[:, 0], gt[:, 0], gt[:, 2])
            gt_z_control = np.interp(control[:, 0], gt[:, 0], gt[:, 3])
            gt_roll_control = np.interp(
                control[:, 0], gt[:, 0], np.unwrap(gt[:, 4]))
            gt_pitch_control = np.interp(
                control[:, 0], gt[:, 0], np.unwrap(gt[:, 5]))
            gt_yaw_control = np.interp(
                control[:, 0], gt[:, 0], np.unwrap(gt[:, 6]))
            gt_xy_control = np.column_stack([gt_x_control, gt_y_control])
            gt_for_cte = np.column_stack(
                [control[:, 0], gt_x_control, gt_y_control])
            gt_cte = self._cross_track_error(plan[:, :2], gt_for_cte)
            gt_xyz = np.column_stack([gt_x_control, gt_y_control, gt_z_control])
            gt_dist, _, _ = self._path_metrics(gt_xyz)
            gt_total = gt_dist[-1]

        plan_total = plan_dist[-1]
        traj_total = traj_dist[-1]
        max_pitch_deg = np.degrees(np.max(np.abs(plan_pitch))) if len(plan_pitch) > 0 else 0.0
        max_roll_deg = np.degrees(np.max(np.abs(plan_roll))) if len(plan_roll) > 0 else 0.0
        actual_max_pitch_deg = (
            np.degrees(np.max(np.abs(gt_pitch_control)))
            if gt_pitch_control is not None else
            np.degrees(np.max(np.abs(traj[:, 5]))))
        actual_max_roll_deg = (
            np.degrees(np.max(np.abs(gt_roll_control)))
            if gt_roll_control is not None else
            np.degrees(np.max(np.abs(traj[:, 4]))))
        actual_pitch_samples = (gt_pitch_control if gt_pitch_control is not None
                                else traj[:, 5])
        actual_roll_samples = (gt_roll_control if gt_roll_control is not None
                               else traj[:, 4])
        actual_pitch_abs_deg = np.degrees(np.abs(actual_pitch_samples))
        actual_roll_abs_deg = np.degrees(np.abs(actual_roll_samples))
        actual_mean_pitch_deg = float(np.mean(actual_pitch_abs_deg))
        actual_p95_pitch_deg = float(np.percentile(actual_pitch_abs_deg, 95.0))
        actual_pitch_exposure_fraction = float(
            np.mean(actual_pitch_abs_deg >= 10.0))
        actual_mean_roll_deg = float(np.mean(actual_roll_abs_deg))
        actual_p95_roll_deg = float(np.percentile(actual_roll_abs_deg, 95.0))
        actual_roll_exposure_fraction = float(
            np.mean(actual_roll_abs_deg >= 8.0))
        mean_cte = np.mean(cte)
        max_cte = np.max(cte)
        commanded_goal = (self.goal_pose if self.goal_pose is not None
                          else plan[-1, :2])
        map_goal_err, map_goal_along, map_goal_lat = self._goal_metrics(
            plan[:, :2], traj[-1, 1:3], commanded_goal)
        ctrl_goal_err, ctrl_goal_along, ctrl_goal_lat = self._goal_metrics(
            plan[:, :2], control[-1, 1:3], commanded_goal)
        plan_goal_gap = float(np.linalg.norm(plan[-1, :2] - commanded_goal))
        gt_goal_err = gt_goal_along = gt_goal_lat = None
        if gt_xy_control is not None:
            gt_goal_err, gt_goal_along, gt_goal_lat = self._goal_metrics(
                plan[:, :2], gt_xy_control[-1], commanded_goal)

        import matplotlib.gridspec as gridspec
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(self.output_dir, 'nav_%d' % self.run_count)
        artifact_dirs = self._run_artifact_dirs(run_dir)
        figure_dir = artifact_dirs['figures']
        csv_dir = artifact_dirs['csv']
        json_dir = artifact_dirs['json']
        # Attach evaluator-defined curvature only to the successful planner
        # call that produced the published path.  Failed/recovery attempts in
        # the same run must not inherit metrics from a different path.
        published_planner_record = None
        for planner_record in reversed(self.planner_statistics):
            if planner_record.get('success', False):
                published_planner_record = planner_record
                break
        if published_planner_record is None and self.planner_statistics:
            published_planner_record = self.planner_statistics[-1]
        if published_planner_record is not None:
            published_planner_record['mean_abs_curvature_1pm'] = float(
                plan_mean_abs_curvature)
            published_planner_record['max_abs_curvature_1pm'] = float(
                plan_max_abs_curvature)
        raw_csv_files = self._export_raw_csv(run_dir)
        diagnostic_files = self._generate_mpc_diagnostic_plots(
            run_dir, control[0, 0])
        sensor_files = self._generate_sensor_diagnostic_plots(run_dir)
        if gt_cte is not None:
            title_base = (u'Nav #%d | %s | Plan %.1fm | Map %.1fm | Ctrl %.1fm | GT %.1fm | Time %.1fs\n'
                          u'CTE Map %.2fm | Ctrl %.2fm | GT %.2fm | '
                          u'Goal GT %.2fm' %
                          (self.run_count, status, plan_total, traj_total,
                           control_dist[-1], gt_total, elapsed, mean_cte,
                           np.mean(control_cte), np.mean(gt_cte), gt_goal_err))
        else:
            title_base = (u'Nav #%d | %s | Plan %.1fm | Est %.1fm | '
                          u'Time %.1fs | CTE Est %.2fm (max %.2fm)' %
                          (self.run_count, status, plan_total, traj_total,
                           elapsed, mean_cte, max_cte))

        planner_stage_files = self._generate_planner_stage_plots(
            run_dir, title_base, plan[:, :2])

        # -- 1) controller-view top-down comparison --
        fig1, ax = plt.subplots(figsize=(12, 8))
        self._draw_terrain_background(ax)

        ax.plot(plan[:, 0], plan[:, 1], color='#0057d9', lw=2.2,
                label='Planned', zorder=6)
        ax.plot(control[:, 1], control[:, 2], color='#ff7f0e', lw=1.6,
                label='Controller estimate', zorder=7)
        ax.plot(plan[0, 0], plan[0, 1], 'o', color='#0057d9', ms=8,
                label='Planned start')
        ax.plot(plan[-1, 0], plan[-1, 1], 'x', color='#0057d9', ms=8,
                label='Planned end')
        ax.plot(control[0, 1], control[0, 2], 'o', color='orange', ms=8,
                fillstyle='none', label='Controller start')
        ax.plot(control[-1, 1], control[-1, 2], 'x', color='orange', ms=8,
                label='Controller end')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(title_base + '\nPlanned vs Controller View',
                     fontsize=10, fontweight='bold')
        ax.legend(fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

        path1 = os.path.join(figure_dir, 'top_view.png')
        fig1.savefig(path1, dpi=150, bbox_inches='tight')
        plt.close(fig1)

        # -- 2) planned path versus external Qianxun reference --
        path_gt_view = None
        if gt_xy_control is not None:
            fig_gt, ax = plt.subplots(figsize=(12, 8))
            self._draw_terrain_background(ax)

            ax.plot(plan[:, 0], plan[:, 1], color='#0057d9', lw=2.2,
                    label='Planned', zorder=6)
            ax.plot(gt_xy_control[:, 0], gt_xy_control[:, 1],
                    color='#d81b60', lw=2.0,
                    label='Qianxun reference', zorder=7)
            ax.plot(plan[0, 0], plan[0, 1], 'o', color='#0057d9', ms=8,
                    label='Planned start')
            ax.plot(plan[-1, 0], plan[-1, 1], 'x', color='#0057d9', ms=8,
                    label='Planned end')
            ax.plot(gt_xy_control[0, 0], gt_xy_control[0, 1], 'o',
                    color='#d81b60', ms=8, fillstyle='none',
                    label='Qianxun start')
            ax.plot(gt_xy_control[-1, 0], gt_xy_control[-1, 1], 'x',
                    color='#d81b60', ms=8,
                    label='Qianxun end')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(title_base + '\nPlanned vs Qianxun Reference',
                         fontsize=10, fontweight='bold')
            ax.legend(fontsize=8)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)

            path_gt_view = os.path.join(
                figure_dir, 'planned_vs_qianxun.png')
            fig_gt.savefig(path_gt_view, dpi=150, bbox_inches='tight')
            plt.close(fig_gt)

        # -- 3) pitch & roll --
        fig2, ax = plt.subplots(figsize=(12, 5))
        mid = (plan_dist[:-1] + plan_dist[1:]) / 2.0
        ax.plot(mid, np.degrees(plan_pitch), 'b-', lw=1.5, label='Plan pitch')
        ax.plot(mid, np.degrees(plan_roll), 'g-', lw=1.5, label='Plan roll')
        if gt_pitch_control is not None and gt_roll_control is not None:
            ax.plot(gt_dist, np.degrees(gt_pitch_control), 'r--', lw=1,
                    alpha=0.85, label='Actual pitch (Qianxun reference)')
            ax.plot(gt_dist, np.degrees(gt_roll_control), 'm--', lw=1,
                    alpha=0.85, label='Actual roll (Qianxun reference)')
        else:
            ax.plot(traj_dist, np.degrees(traj[:, 5]), 'r--', lw=1,
                    alpha=0.7, label='Pitch (mapping odometry fallback)')
            ax.plot(traj_dist, np.degrees(traj[:, 4]), 'm--', lw=1,
                    alpha=0.7, label='Roll (mapping odometry fallback)')
        ax.axhline(y=31.5, color='orange', ls=':', alpha=0.5,
                   label='max limit')
        ax.axhline(y=-31.5, color='orange', ls=':', alpha=0.5)
        ax.set_xlabel('Path Distance (m)')
        ax.set_ylabel(u'Angle (°)')
        ax.set_title(title_base + '\nTerrain Pitch & Roll', fontsize=10, fontweight='bold')
        ax.legend(fontsize=8, ncol=3)
        ax.grid(True, alpha=0.3)
        fig2.tight_layout()

        path2 = os.path.join(figure_dir, 'pitch_roll.png')
        fig2.savefig(path2, dpi=150, bbox_inches='tight')
        plt.close(fig2)

        # -- 4) controller-view cross-track error --
        fig3, ax = plt.subplots(figsize=(12, 5))
        ax.plot(control_time, control_cte, color='orange', lw=1.3,
                label='Controller-view CTE')
        controller_mean_cte = np.mean(control_cte)
        ax.axhline(y=controller_mean_cte, color='orange', ls='--',
                   label='Estimate mean %.2fm' % controller_mean_cte)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Cross-Track Error (m)')
        ax.yaxis.set_major_locator(MultipleLocator(0.1))
        # Centre the displayed range on the estimate mean while preserving
        # every measured sample. A minimum +/-0.5 m range keeps repeated runs
        # visually comparable without smoothing or clipping the CTE data.
        max_deviation = np.max(np.abs(control_cte - controller_mean_cte))
        half_range = max(
            self.cte_plot_min_half_range,
            np.ceil(max_deviation * 1.15 / 0.1) * 0.1)
        ax.set_ylim(controller_mean_cte - half_range,
                    controller_mean_cte + half_range)
        ax.set_title(title_base.split('\n')[0] +
                     '\nController-view Tracking Accuracy',
                     fontsize=10, fontweight='bold')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig3.tight_layout()

        path3 = os.path.join(figure_dir, 'cte.png')
        fig3.savefig(path3, dpi=150, bbox_inches='tight')
        plt.close(fig3)

        # -- 5) operational localization error used by the controller --
        # GT already has the frozen pre-navigation world->map transform.
        # A whole-run Umeyama fit would hide the drift that creates GT CTE.
        path4 = None
        localization_metrics_path = None
        raw_mean_loc = raw_rmse_loc = raw_max_loc = None
        relative_mean_loc = relative_rmse_loc = relative_max_loc = None
        target_mean_loc = target_rmse_loc = target_max_loc = None
        target_relative_mean_loc = None
        target_relative_rmse_loc = None
        target_relative_max_loc = None
        if gt_xy_control is not None:
            est_xy = control[:, 1:3]
            est_yaw = np.unwrap(control[:, 6])
            raw_pos_err = np.sqrt(
                np.sum((est_xy - gt_xy_control) ** 2, axis=1))
            absolute_metrics = self._position_error_metrics(raw_pos_err)
            raw_mean_loc = absolute_metrics['mean_m']
            raw_rmse_loc = absolute_metrics['rmse_m']
            raw_max_loc = absolute_metrics['max_m']

            relative_gt_xy, controller_start_alignment = (
                self._align_reference_at_start(
                    gt_xy_control, gt_yaw_control, est_xy, est_yaw))
            relative_pos_err = np.sqrt(
                np.sum((est_xy - relative_gt_xy) ** 2, axis=1))
            relative_metrics = self._position_error_metrics(relative_pos_err)
            relative_mean_loc = relative_metrics['mean_m']
            relative_rmse_loc = relative_metrics['rmse_m']
            relative_max_loc = relative_metrics['max_m']

            target_xy = None
            target_error = None
            target_relative_error = None
            target_start_alignment = None
            if gt is not None and len(self.ndt_target_traj) >= 3:
                target = np.asarray(self.ndt_target_traj, dtype=float)
                target_xy = target[:, 1:3]
                target_gt_x = np.interp(target[:, 0], gt[:, 0], gt[:, 1])
                target_gt_y = np.interp(target[:, 0], gt[:, 0], gt[:, 2])
                target_gt_xy = np.column_stack([target_gt_x, target_gt_y])
                target_error = np.sqrt(
                    np.sum((target_xy - target_gt_xy) ** 2, axis=1))
                target_metrics = self._position_error_metrics(target_error)
                target_mean_loc = target_metrics['mean_m']
                target_rmse_loc = target_metrics['rmse_m']
                target_max_loc = target_metrics['max_m']

                target_gt_yaw = np.interp(
                    target[:, 0], gt[:, 0], np.unwrap(gt[:, 6]))
                target_yaw = np.unwrap(target[:, 6])
                target_relative_gt_xy, target_start_alignment = (
                    self._align_reference_at_start(
                        target_gt_xy, target_gt_yaw, target_xy, target_yaw))
                target_relative_error = np.sqrt(np.sum(
                    (target_xy - target_relative_gt_xy) ** 2, axis=1))
                target_relative_metrics = self._position_error_metrics(
                    target_relative_error)
                target_relative_mean_loc = target_relative_metrics['mean_m']
                target_relative_rmse_loc = target_relative_metrics['rmse_m']
                target_relative_max_loc = target_relative_metrics['max_m']

            fig4, axes = plt.subplots(1, 2, figsize=(16, 7))
            ax = axes[0]
            self._draw_terrain_background(ax)

            ax.plot(gt_xy_control[:, 0], gt_xy_control[:, 1],
                    color='#0057d9', lw=2.2,
                    label='GT (absolute map anchor)', zorder=6)
            ax.plot(relative_gt_xy[:, 0], relative_gt_xy[:, 1],
                    color='#6a3d9a', lw=1.4, ls='--',
                    label='GT (start-pose aligned)', zorder=6)
            ax.plot(est_xy[:, 0], est_xy[:, 1], color='#e31a1c', lw=1.7,
                    label='Controller localization', zorder=7)
            if target_xy is not None:
                ax.plot(target_xy[:, 0], target_xy[:, 1], color='#00a087',
                        lw=1.0, alpha=0.75, label='Raw NDT target', zorder=8)
            ax.plot(gt_xy_control[0, 0], gt_xy_control[0, 1], 'o',
                    color='#0057d9',
                    ms=10, zorder=8)
            ax.plot(gt_xy_control[-1, 0], gt_xy_control[-1, 1], '*',
                    color='#e31a1c',
                    ms=14, zorder=8)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Absolute and Start-aligned Trajectories',
                         fontsize=10, fontweight='bold')
            ax.legend(fontsize=8)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)

            error_ax = axes[1]
            error_ax.plot(control_time, raw_pos_err, color='#e31a1c', lw=1.4,
                          label='Controller absolute error')
            error_ax.plot(control_time, relative_pos_err, color='#6a3d9a',
                          lw=1.4, ls='--',
                          label='Controller relative drift')
            if target_error is not None:
                target_time = target[:, 0] - control[0, 0]
                error_ax.plot(target_time, target_error, color='#00a087',
                              lw=1.0, alpha=0.8,
                              label='Raw NDT absolute error')
                error_ax.plot(target_time, target_relative_error,
                              color='#00755e', lw=1.0, ls='--', alpha=0.8,
                              label='Raw NDT relative drift')
            error_ax.set_xlabel('Time (s)')
            error_ax.set_ylabel('Position Error (m)')
            error_ax.set_title(
                'Absolute ATE %.3fm | Relative-drift ATE %.3fm' %
                (raw_mean_loc, relative_mean_loc),
                fontsize=10, fontweight='bold')
            error_ax.grid(True, alpha=0.3)
            error_ax.legend(fontsize=8)

            fig4.suptitle(
                title_base.split('\n')[0] +
                '\nAbsolute error retains map/NDT bias; relative drift removes '
                'only the first synchronized pose',
                fontsize=10, fontweight='bold')
            fig4.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))

            path4 = os.path.join(figure_dir, 'localization_error.png')
            fig4.savefig(path4, dpi=150, bbox_inches='tight')
            plt.close(fig4)

            localization_metrics_path = os.path.join(
                json_dir, 'localization_metrics.json')
            localization_metrics = {
                'map': self.map_identity,
                'localization_mode': self.localization_mode,
                'definition': {
                    'absolute': ('stationary Qianxun world-to-map anchor; '
                                 'retains map registration and initial '
                                 'localization bias'),
                    'relative_drift': ('GT aligned by first synchronized '
                                       'position and heading only; no '
                                       'whole-trajectory fit'),
                },
                'gt_absolute_anchor': gt_frame_correction,
                'controller_localization': {
                    'absolute_ate': absolute_metrics,
                    'relative_drift_ate': relative_metrics,
                    'relative_start_alignment': controller_start_alignment,
                },
            }
            if target_mean_loc is not None:
                localization_metrics['raw_ndt_target'] = {
                    'absolute_ate': target_metrics,
                    'relative_drift_ate': target_relative_metrics,
                    'relative_start_alignment': target_start_alignment,
                }
            with open(localization_metrics_path, 'w') as stream:
                json.dump(localization_metrics, stream, indent=2,
                          sort_keys=True)

        # -- 6) navigation progress metrics and controller-view motion --
        # Keep these values for CSV/JSON evaluation, but do not generate the
        # former navigation_progress.png.  Its "Measured speed" could be a
        # pose-derivative fallback and was easily confused with physical GT.
        progress = self._path_progress_series(plan[:, :2], control[:, 1:3])
        goal_xy = np.asarray(commanded_goal, dtype=float)
        distance_to_goal = np.linalg.norm(control[:, 1:3] - goal_xy, axis=1)
        # Odometry twist is the speed seen by MPC. Some LIO-SAM odometry
        # publishers leave both twist fields at zero; detect that condition
        # and derive the controller-view response from synchronized poses.
        reported_speed = control[:, 7]
        reported_yaw_rate = control[:, 8]
        pose_motion = float(np.sum(np.linalg.norm(
            np.diff(control[:, 1:3], axis=0), axis=1)))
        twist_is_populated = (
            np.max(np.abs(reported_speed)) > 1e-4 or
            np.max(np.abs(reported_yaw_rate)) > 1e-4 or
            pose_motion <= 0.05)
        if twist_is_populated:
            measured_speed = reported_speed
            measured_yaw_rate = reported_yaw_rate
            controller_twist_source = 'odometry_twist'
        else:
            measured_speed, measured_yaw_rate = (
                self._derive_twist_from_pose(control))
            controller_twist_source = 'pose_derivative_fallback'
            rospy.logwarn(
                'Controller odometry twist is empty; velocity evaluation '
                'uses pose derivatives for this run')
        # -- 7) applied command versus Qianxun-reference response --
        control_response_path = None
        velocity_response_metrics = None
        if len(self.cmd_vel_traj) >= 2:
            command = np.asarray(self.cmd_vel_traj, dtype=float)
            command_time = command[:, 0] - control[0, 0]
            gt_twist = (np.asarray(self.gt_twist_traj, dtype=float)
                        if len(self.gt_twist_traj) >= 2 else None)
            fig_response, response_axes = plt.subplots(
                2, 1, figsize=(12, 7), sharex=True)
            response_axes[0].plot(command_time, command[:, 1],
                                  color='#0057d9', lw=1.2,
                                  label='Applied /cmd_vel')
            if gt_twist is not None:
                gt_twist_time = gt_twist[:, 0] - control[0, 0]
                response_axes[0].plot(
                    gt_twist_time, gt_twist[:, 1], color='#009e73', lw=1.3,
                    label='Qianxun reference')
            response_axes[0].set_ylabel('Linear speed (m/s)')
            response_axes[0].legend(fontsize=8)
            response_axes[0].grid(True, alpha=0.3)
            response_axes[1].plot(command_time, command[:, 2],
                                  color='#6a3d9a', lw=1.2,
                                  label='Applied /cmd_vel')
            if gt_twist is not None:
                response_axes[1].plot(
                    gt_twist_time, gt_twist[:, 2], color='#009e73', lw=1.3,
                    label='Qianxun reference')
            response_axes[1].set_xlabel('Time (s)')
            response_axes[1].set_ylabel('Yaw rate (rad/s)')
            response_axes[1].legend(fontsize=8)
            response_axes[1].grid(True, alpha=0.3)
            fig_response.suptitle(title_base.split('\n')[0] +
                                  '\nApplied Command and Qianxun-Reference Response',
                                  fontsize=10, fontweight='bold')
            fig_response.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
            control_response_path = os.path.join(
                figure_dir, 'control_response.png')
            fig_response.savefig(control_response_path, dpi=150,
                                  bbox_inches='tight')
            plt.close(fig_response)

            if gt_twist is not None:
                gt_speed_at_control = np.interp(
                    control[:, 0], gt_twist[:, 0], gt_twist[:, 1])
                gt_yaw_rate_at_control = np.interp(
                    control[:, 0], gt_twist[:, 0], gt_twist[:, 2])
                speed_error = measured_speed - gt_speed_at_control
                yaw_rate_error = measured_yaw_rate - gt_yaw_rate_at_control
                velocity_response_metrics = {
                    'controller_twist_source': controller_twist_source,
                    'controller_vs_gt_linear_speed_rmse_mps': float(
                        np.sqrt(np.mean(speed_error ** 2))),
                    'controller_vs_gt_linear_speed_max_abs_mps': float(
                        np.max(np.abs(speed_error))),
                    'controller_vs_gt_yaw_rate_rmse_radps': float(
                        np.sqrt(np.mean(yaw_rate_error ** 2))),
                    'controller_vs_gt_yaw_rate_max_abs_radps': float(
                        np.max(np.abs(yaw_rate_error))),
                }

        # -- 8) curvature of the final published global path --
        fig_curvature, curvature_ax = plt.subplots(figsize=(10, 4.5))
        if plan_curvature.size:
            curvature_ax.plot(plan_curvature_s, plan_curvature,
                              color='#6a3d9a', lw=1.6,
                              label='Signed curvature')
        curvature_ax.axhline(0.0, color='black', lw=0.7)
        curvature_ax.set_xlabel('Path arc length (m)')
        curvature_ax.set_ylabel(r'Curvature (m$^{-1}$)')
        curvature_ax.set_title(
            'Global Path Curvature | mean |k|=%.4f 1/m | max |k|=%.4f 1/m' %
            (plan_mean_abs_curvature, plan_max_abs_curvature))
        curvature_ax.grid(True, alpha=0.25)
        if plan_curvature.size:
            curvature_ax.legend(fontsize=8)
        curvature_path = os.path.join(figure_dir, 'path_curvature.png')
        fig_curvature.tight_layout()
        fig_curvature.savefig(curvature_path, dpi=150, bbox_inches='tight')
        plt.close(fig_curvature)

        # -- 9) one-page numerical metric summary --
        fig_summary, summary_axes = plt.subplots(2, 3, figsize=(16, 9))

        length_names = ['Plan', 'Mapping', 'Controller']
        length_values = [plan_total, traj_total, control_dist[-1]]
        if gt_total is not None:
            length_names.append('Ground truth')
            length_values.append(gt_total)
        bars = summary_axes[0, 0].bar(length_names, length_values,
                                      color=['#0057d9', '#7f7f7f',
                                             '#ff7f0e', '#d81b60'][:len(length_names)])
        summary_axes[0, 0].set_title('Trajectory length')
        summary_axes[0, 0].set_ylabel('Length (m)')
        summary_axes[0, 0].tick_params(axis='x', rotation=20)
        for bar, value in zip(bars, length_values):
            summary_axes[0, 0].text(bar.get_x() + bar.get_width() / 2.0,
                                    bar.get_height(), '%.2f' % value,
                                    ha='center', va='bottom', fontsize=8)

        cte_names = ['Map', 'Controller']
        cte_mean_values = [mean_cte, float(np.mean(control_cte))]
        cte_max_values = [max_cte, float(np.max(control_cte))]
        if gt_cte is not None:
            cte_names.append('Ground truth')
            cte_mean_values.append(float(np.mean(gt_cte)))
            cte_max_values.append(float(np.max(gt_cte)))
        x_cte = np.arange(len(cte_names))
        summary_axes[0, 1].bar(x_cte - 0.18, cte_mean_values, 0.36,
                               label='Mean')
        summary_axes[0, 1].bar(x_cte + 0.18, cte_max_values, 0.36,
                               label='Max')
        summary_axes[0, 1].set_xticks(x_cte)
        summary_axes[0, 1].set_xticklabels(cte_names, rotation=15)
        summary_axes[0, 1].set_ylabel('CTE (m)')
        summary_axes[0, 1].set_title('Cross-track error')
        summary_axes[0, 1].legend(fontsize=8)

        goal_names = ['Controller', 'Mapping']
        goal_position = [ctrl_goal_err, map_goal_err]
        goal_along = [ctrl_goal_along, map_goal_along]
        goal_lateral = [ctrl_goal_lat, map_goal_lat]
        if gt_goal_err is not None:
            goal_names.append('Ground truth')
            goal_position.append(gt_goal_err)
            goal_along.append(gt_goal_along)
            goal_lateral.append(gt_goal_lat)
        x_goal = np.arange(len(goal_names))
        summary_axes[0, 2].bar(x_goal - 0.25, goal_position, 0.25,
                               label='Position')
        summary_axes[0, 2].bar(x_goal, goal_along, 0.25, label='Along')
        summary_axes[0, 2].bar(x_goal + 0.25, goal_lateral, 0.25,
                               label='Lateral')
        summary_axes[0, 2].axhline(0.0, color='black', lw=0.6)
        summary_axes[0, 2].set_xticks(x_goal)
        summary_axes[0, 2].set_xticklabels(goal_names, rotation=15)
        summary_axes[0, 2].set_ylabel('Goal error (m)')
        summary_axes[0, 2].set_title('Terminal error components')
        summary_axes[0, 2].legend(fontsize=8)

        attitude_names = ['Plan pitch', 'Plan roll',
                          'Actual pitch', 'Actual roll']
        attitude_values = [max_pitch_deg, max_roll_deg,
                           actual_max_pitch_deg, actual_max_roll_deg]
        attitude_bars = summary_axes[1, 0].bar(
            attitude_names, attitude_values,
            color=['#0057d9', '#009e73', '#e31a1c', '#d81b60'])
        summary_axes[1, 0].axhline(31.5, color='orange', ls='--', lw=1.0,
                                   label='Limit')
        summary_axes[1, 0].set_ylabel('Maximum absolute angle (deg)')
        summary_axes[1, 0].set_title('Terrain attitude')
        summary_axes[1, 0].tick_params(axis='x', rotation=20)
        summary_axes[1, 0].legend(fontsize=8)
        for bar, value in zip(attitude_bars, attitude_values):
            summary_axes[1, 0].text(bar.get_x() + bar.get_width() / 2.0,
                                    bar.get_height(), '%.1f' % value,
                                    ha='center', va='bottom', fontsize=8)

        localization_names = []
        localization_values = []
        if raw_mean_loc is not None:
            localization_names.extend(['Absolute ATE', 'Relative drift'])
            localization_values.extend([raw_mean_loc, relative_mean_loc])
        if target_mean_loc is not None:
            localization_names.extend(['NDT absolute', 'NDT relative'])
            localization_values.extend(
                [target_mean_loc, target_relative_mean_loc])
        if localization_values:
            loc_bars = summary_axes[1, 1].bar(
                localization_names, localization_values,
                color=['#e31a1c', '#6a3d9a', '#00a087', '#00755e'][:len(localization_values)])
            for bar, value in zip(loc_bars, localization_values):
                summary_axes[1, 1].text(
                    bar.get_x() + bar.get_width() / 2.0, bar.get_height(),
                    '%.3f' % value, ha='center', va='bottom', fontsize=8)
            summary_axes[1, 1].tick_params(axis='x', rotation=20)
        else:
            summary_axes[1, 1].text(0.5, 0.5, 'GT localization metrics unavailable',
                                    ha='center', va='center')
        summary_axes[1, 1].set_ylabel('Mean position error (m)')
        summary_axes[1, 1].set_title('Localization decomposition')

        summary_axes[1, 2].axis('off')
        summary_axes[1, 2].text(
            0.03, 0.95,
             ('Status: %s\nElapsed time: %.1f s\nPlan endpoint gap: %.3f m\n'
              'Mean |curvature|: %.4f 1/m\nMax |curvature|: %.4f 1/m\n'
              'Final progress: %.1f %%\nFinal distance to goal: %.3f m\n'
             'Controller samples: %d\nGT samples: %d\nNDT targets: %d\n'
             'Applied commands: %d\nMPC raw commands: %d') %
            (status, elapsed, plan_goal_gap, plan_mean_abs_curvature,
             plan_max_abs_curvature, progress[-1],
             distance_to_goal[-1], len(control), len(self.gt_traj),
             len(self.ndt_target_traj), len(self.cmd_vel_traj),
             len(self.mpc_cmd_vel_traj)),
            va='top', ha='left', family='monospace', fontsize=10)
        summary_axes[1, 2].set_title('Run information')
        for summary_ax in summary_axes.flat:
            if summary_ax.axison:
                summary_ax.grid(True, axis='y', alpha=0.22)
        fig_summary.suptitle(title_base.split('\n')[0] +
                             '\nEvaluation Metric Summary',
                             fontsize=11, fontweight='bold')
        fig_summary.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
        summary_path = os.path.join(figure_dir, 'evaluation_summary.png')
        fig_summary.savefig(summary_path, dpi=150, bbox_inches='tight')
        plt.close(fig_summary)

        evaluation_metrics_path = os.path.join(
            json_dir, 'evaluation_metrics.json')
        evaluation_metrics = {
            'map': self.map_identity,
            'localization_mode': self.localization_mode,
            'run': {'index': int(self.run_count), 'status': status,
                    'elapsed_s': float(elapsed)},
            'length_m': dict(zip(length_names, map(float, length_values))),
            'cte_m': {
                name: {'mean': float(mean), 'max': float(maximum)}
                for name, mean, maximum in zip(
                    cte_names, cte_mean_values, cte_max_values)},
            'goal_error_m': {
                name: {'position': float(position), 'along': float(along),
                       'lateral': float(lateral)}
                for name, position, along, lateral in zip(
                    goal_names, goal_position, goal_along, goal_lateral)},
            'attitude_max_abs_deg': dict(zip(
                attitude_names, map(float, attitude_values))),
            'path_curvature_1pm': {
                'mean_abs': float(plan_mean_abs_curvature),
                'max_abs': float(plan_max_abs_curvature)},
            'plan_endpoint_gap_m': float(plan_goal_gap),
            'final_progress_percent': float(progress[-1]),
            'final_distance_to_goal_m': float(distance_to_goal[-1]),
            'controller_twist_source': controller_twist_source,
            'planner_statistics': self.planner_statistics,
        }
        if raw_mean_loc is not None:
            evaluation_metrics['localization'] = localization_metrics
        if velocity_response_metrics is not None:
            evaluation_metrics['velocity_response'] = velocity_response_metrics
        with open(evaluation_metrics_path, 'w') as stream:
            json.dump(evaluation_metrics, stream, indent=2, sort_keys=True)

        latest_planner = published_planner_record or {}
        comparison_summary = {
            'map_module': self.map_module,
            'map_fingerprint': self.map_identity['fingerprint'],
            'localization_mode': self.localization_mode,
            'run_index': int(self.run_count),
            'status': status,
            'algorithm': latest_planner.get('algorithm', 'unknown'),
            'navigation_time_s': float(elapsed),
            'planning_time_ms': latest_planner.get(
                'planning_time_ms', float('nan')),
            'expanded_nodes': latest_planner.get(
                'expanded_nodes', float('nan')),
            'generated_nodes': latest_planner.get(
                'generated_nodes', float('nan')),
            'search_calls': latest_planner.get('search_calls', 1),
            'coarse_channels': latest_planner.get(
                'coarse_channels', float('nan')),
            'fine_candidates': latest_planner.get(
                'fine_candidates', float('nan')),
            'selected_candidate': latest_planner.get(
                'selected_candidate', float('nan')),
            'plan_length_m': float(plan_total),
            'plan_mean_abs_curvature_1pm': float(plan_mean_abs_curvature),
            'plan_max_abs_curvature_1pm': float(plan_max_abs_curvature),
            'mapping_length_m': float(traj_total),
            'controller_length_m': float(control_dist[-1]),
            'controller_cte_mean_m': float(np.mean(control_cte)),
            'controller_cte_max_m': float(np.max(control_cte)),
            'controller_goal_error_m': float(ctrl_goal_err),
            'plan_endpoint_gap_m': float(plan_goal_gap),
            'mean_terrain_risk': latest_planner.get(
                'mean_terrain_risk', float('nan')),
            'peak_terrain_risk': latest_planner.get(
                'peak_terrain_risk', float('nan')),
            'terrain_cost_integral': latest_planner.get(
                'terrain_cost_integral', float('nan')),
            'planned_pitch_mean_deg': latest_planner.get(
                'mean_abs_pitch_deg', float('nan')),
            'planned_pitch_p95_deg': latest_planner.get(
                'p95_abs_pitch_deg', float('nan')),
            'planned_pitch_max_deg': latest_planner.get(
                'max_abs_pitch_deg', float('nan')),
            'planned_pitch_exposure_fraction': latest_planner.get(
                'pitch_exposure_fraction', float('nan')),
            'planned_roll_mean_deg': latest_planner.get(
                'mean_abs_roll_deg', float('nan')),
            'planned_roll_p95_deg': latest_planner.get(
                'p95_abs_roll_deg', float('nan')),
            'planned_roll_max_deg': latest_planner.get(
                'max_abs_roll_deg', float('nan')),
            'planned_roll_exposure_fraction': latest_planner.get(
                'roll_exposure_fraction', float('nan')),
            'actual_max_pitch_deg': float(actual_max_pitch_deg),
            'actual_max_roll_deg': float(actual_max_roll_deg),
            'actual_pitch_mean_deg': actual_mean_pitch_deg,
            'actual_pitch_p95_deg': actual_p95_pitch_deg,
            'actual_pitch_exposure_fraction':
                actual_pitch_exposure_fraction,
            'actual_roll_mean_deg': actual_mean_roll_deg,
            'actual_roll_p95_deg': actual_p95_roll_deg,
            'actual_roll_exposure_fraction': actual_roll_exposure_fraction,
            'gt_length_m': float('nan'),
            'gt_cte_mean_m': float('nan'),
            'gt_cte_max_m': float('nan'),
            'gt_goal_error_m': float('nan'),
            'localization_ate_mean_m': float('nan'),
            'localization_ate_rmse_m': float('nan'),
            'localization_ate_max_m': float('nan'),
        }
        if gt_total is not None:
            comparison_summary.update({
                'gt_length_m': float(gt_total),
                'gt_cte_mean_m': float(np.mean(gt_cte)),
                'gt_cte_max_m': float(np.max(gt_cte)),
                'gt_goal_error_m': float(gt_goal_err),
            })
        if raw_mean_loc is not None:
            comparison_summary.update({
                'localization_ate_mean_m': float(raw_mean_loc),
                'localization_ate_rmse_m': float(raw_rmse_loc),
                'localization_ate_max_m': float(raw_max_loc),
            })
        run_summary_path = os.path.join(csv_dir, 'run_summary.csv')
        self._write_csv_dicts(run_summary_path, [comparison_summary])
        comparison_table_path = os.path.join(
            csv_dir, 'algorithm_comparison_runs.csv')
        comparison_table_exists = os.path.exists(comparison_table_path)
        with open(comparison_table_path, 'a', newline='') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=list(comparison_summary.keys()))
            if not comparison_table_exists:
                writer.writeheader()
            writer.writerow(comparison_summary)

        rospy.loginfo('Saved: %s', path1)
        for stage_file in planner_stage_files:
            rospy.loginfo('Saved: %s', stage_file)
        if path_gt_view:
            rospy.loginfo('Saved: %s', path_gt_view)
        rospy.loginfo('Saved: %s', path2)
        rospy.loginfo('Saved: %s', path3)
        if control_response_path:
            rospy.loginfo('Saved: %s', control_response_path)
        rospy.loginfo('Saved: %s', summary_path)
        rospy.loginfo('Saved: %s', evaluation_metrics_path)
        rospy.loginfo('Saved: %s', run_summary_path)
        rospy.loginfo('Saved: %s', comparison_table_path)
        for raw_file in raw_csv_files:
            rospy.loginfo('Saved: %s', raw_file)
        for diagnostic_file in diagnostic_files:
            rospy.loginfo('Saved: %s', diagnostic_file)
        for sensor_file in sensor_files:
            rospy.loginfo('Saved: %s', sensor_file)
        if path4:
            rospy.loginfo('Saved: %s', path4)
            rospy.loginfo('Saved: %s', localization_metrics_path)
            rospy.loginfo(
                '  Controller absolute ATE: mean=%.3fm RMSE=%.3fm '
                'max=%.3fm (stationary GT map anchor)',
                raw_mean_loc, raw_rmse_loc, raw_max_loc)
            rospy.loginfo(
                '  Controller relative drift ATE: mean=%.3fm RMSE=%.3fm '
                'max=%.3fm (first synchronized pose only)',
                relative_mean_loc, relative_rmse_loc, relative_max_loc)
            if target_mean_loc is not None:
                rospy.loginfo(
                    '  Raw NDT target absolute ATE: mean=%.3fm RMSE=%.3fm '
                    'max=%.3fm',
                    target_mean_loc, target_rmse_loc, target_max_loc)
                rospy.loginfo(
                    '  Raw NDT target relative drift ATE: mean=%.3fm '
                    'RMSE=%.3fm max=%.3fm',
                    target_relative_mean_loc, target_relative_rmse_loc,
                    target_relative_max_loc)
        rospy.loginfo('  Plan %.1fm | Actual %.1fm | CTE mean=%.2fm max=%.2fm',
                      plan_total, traj_total, mean_cte, max_cte)
        if gt_cte is not None:
            rospy.loginfo('  Ground truth %.1fm | CTE mean=%.2fm max=%.2fm',
                          gt_total, np.mean(gt_cte), np.max(gt_cte))
            if gt_frame_correction is not None:
                rospy.loginfo(
                    '  GT world->map anchor: translation=(%+.3f, %+.3f, '
                    '%+.3f)m yaw=%+.2fdeg samples=%d',
                    gt_frame_correction['x'], gt_frame_correction['y'],
                    gt_frame_correction['z'],
                    np.degrees(gt_frame_correction['yaw']),
                    gt_frame_correction['samples'])
            rospy.loginfo('  Goal GT: position=%.2fm along=%+.2fm lateral=%.2fm',
                          gt_goal_err, gt_goal_along, gt_goal_lat)
        rospy.loginfo('  Controller %.1fm | CTE mean=%.2fm max=%.2fm',
                      control_dist[-1], np.mean(control_cte), np.max(control_cte))
        rospy.loginfo('  Goal controller: position=%.2fm along=%+.2fm lateral=%.2fm',
                      ctrl_goal_err, ctrl_goal_along, ctrl_goal_lat)
        rospy.loginfo('  Goal mapping: position=%.2fm along=%+.2fm lateral=%.2fm',
                      map_goal_err, map_goal_along, map_goal_lat)
        rospy.loginfo('  Plan endpoint gap to commanded goal: %.2fm', plan_goal_gap)
        rospy.loginfo('  Max pitch: %.1f deg | Max roll: %.1f deg',
                      max_pitch_deg, max_roll_deg)
        rospy.loginfo('  Qianxun-reference max pitch: %.1f deg | max roll: %.1f deg',
                      actual_max_pitch_deg, actual_max_roll_deg)

        self._nav_output(
            '[NAV] 导航完成 | 状态=%s | 用时=%.1fs | 规划=%.2fm | '
            '控制轨迹=%.2fm%s',
            status, elapsed, plan_total, control_dist[-1],
            (' | 千寻参考轨迹=%.2fm' % gt_total) if gt_total is not None else '')
        self._nav_output(
            '[NAV] 跟踪误差 | 控制器CTE均值=%.3fm 最大=%.3fm%s',
            np.mean(control_cte), np.max(control_cte),
            (' | 千寻参考CTE均值=%.3fm 最大=%.3fm'
             % (np.mean(gt_cte), np.max(gt_cte)))
            if gt_cte is not None else '')
        self._nav_output(
            '[NAV] 终点误差 | 控制器位置=%.3fm 纵向=%+.3fm 横向=%.3fm%s',
            ctrl_goal_err, ctrl_goal_along, ctrl_goal_lat,
            (' | 千寻参考位置=%.3fm 纵向=%+.3fm 横向=%.3fm'
             % (gt_goal_err, gt_goal_along, gt_goal_lat))
            if gt_goal_err is not None else '')
        if raw_mean_loc is not None:
            self._nav_output(
                '[NAV] 定位误差 | ATE均值=%.3fm RMSE=%.3fm 最大=%.3fm '
                '| 导航前固定锚定，无全轨迹拟合',
                raw_mean_loc, raw_rmse_loc, raw_max_loc)
            self._nav_output(
                '[NAV] Localization decomposition | absolute ATE '
                'mean=%.3fm RMSE=%.3fm max=%.3fm | relative drift '
                'mean=%.3fm RMSE=%.3fm max=%.3fm',
                raw_mean_loc, raw_rmse_loc, raw_max_loc,
                relative_mean_loc, relative_rmse_loc, relative_max_loc)
        self._nav_output(
            '[NAV] 地形姿态 | 最大俯仰=%.1fdeg | 最大横滚=%.1fdeg',
            max_pitch_deg, max_roll_deg)
        self._nav_output(
            '[NAV] 路径曲率 | 平均绝对曲率=%.4f 1/m | '
            '最大绝对曲率=%.4f 1/m',
            plan_mean_abs_curvature, plan_max_abs_curvature)


if __name__ == '__main__':
    try:
        node = NavEvaluator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
