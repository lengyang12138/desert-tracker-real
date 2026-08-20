#include "navigation/mpc_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <fstream>
#include <cstring>

PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlanner, nav_core::BaseLocalPlanner)

namespace mpc_local_planner {

static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

MpcLocalPlanner::MpcLocalPlanner()
    : plan_to_control_x_(0.0), plan_to_control_y_(0.0),
      plan_to_control_yaw_(0.0), terrain_query_transform_valid_(false),
      initialized_(false), goal_reached_(false),
      prev_v_cmd_(0.0), prev_omega_cmd_(0.0),
      current_v_(0.0), current_lateral_v_(0.0), current_omega_(0.0),
      current_state_measurement_stamp_(0),
      current_position_covariance_(std::numeric_limits<double>::infinity()),
      current_yaw_covariance_(std::numeric_limits<double>::infinity()),
      observed_lateral_disturbance_(0.0),
      estimated_lateral_disturbance_(0.0),
      last_lateral_disturbance_update_time_(0),
      identified_linear_gain_(1.0), identified_omega_gain_(0.6),
      identified_tau_v_(0.12), identified_tau_omega_(0.20),
      previous_measured_v_(0.0), previous_measured_omega_(0.0),
      last_response_update_time_(0), last_response_measurement_stamp_(0),
      aligned_response_v_cmd_(0.0), aligned_response_omega_cmd_(0.0),
      response_command_alignment_age_(0.0),
      response_model_initialized_(false),
      linear_gain_probe_cmd_(0.0), linear_gain_stable_cycles_(0),
      gain_probe_cmd_(0.0),
      gain_stable_cycles_(0), previous_signed_cte_(0.0),
      filtered_cte_rate_(0.0), held_rate_heading_correction_(0.0),
      turn_assist_improving_time_(0.0),
      last_cte_update_time_(0), turn_assist_latched_(false),
      one_step_cte_prediction_valid_(false),
      one_step_predicted_cte_(0.0),
      one_step_prediction_residual_(0.0),
      one_step_prediction_created_time_(0),
      projection_segment_cache_(-1),
      last_control_cycle_start_(0),
      last_control_cycle_interval_(0.0),
      last_solver_duration_(0.0),
      last_solver_timed_out_(false),
      last_evaluated_candidates_(0),
      consecutive_solver_timeouts_(0),
      localization_quality_ok_(false),
      last_localization_quality_time_(0) {}

void MpcLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) return;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh("~/" + name);

    nh.param("tau_v", params_.tau_v, 0.12);
    nh.param("tau_omega", params_.tau_omega, 0.20);
    nh.param("linear_tracking_gain", params_.linear_tracking_gain, 1.0);
    nh.param("enable_linear_gain_adaptation",
             params_.enable_linear_gain_adaptation, true);
    nh.param("linear_gain_min", params_.linear_gain_min, 0.40);
    nh.param("linear_gain_max", params_.linear_gain_max, 1.20);
    nh.param("linear_gain_filter_alpha",
             params_.linear_gain_filter_alpha, 0.05);
    nh.param("linear_gain_min_command",
             params_.linear_gain_min_command, 0.08);
    nh.param("linear_gain_stable_delta",
             params_.linear_gain_stable_delta, 0.02);
    nh.param("linear_gain_stable_cycles",
             params_.linear_gain_stable_cycles, 3);
    nh.param("omega_tracking_gain", params_.omega_tracking_gain, 0.6);
    nh.param("enable_omega_gain_adaptation",
             params_.enable_omega_gain_adaptation, true);
    nh.param("omega_gain_min", params_.omega_gain_min, 0.20);
    nh.param("omega_gain_max", params_.omega_gain_max, 0.90);
    nh.param("omega_gain_filter_alpha",
             params_.omega_gain_filter_alpha, 0.10);
    nh.param("omega_gain_rise_alpha",
             params_.omega_gain_rise_alpha, 0.02);
    nh.param("omega_gain_min_command",
             params_.omega_gain_min_command, 0.02);
    nh.param("omega_gain_stable_delta",
             params_.omega_gain_stable_delta, 0.015);
    nh.param("omega_gain_stable_cycles",
             params_.omega_gain_stable_cycles, 3);
    nh.param("enable_time_constant_adaptation",
             params_.enable_time_constant_adaptation, true);
    nh.param("tau_v_min", params_.tau_v_min, 0.05);
    nh.param("tau_v_max", params_.tau_v_max, 0.80);
    nh.param("tau_omega_min", params_.tau_omega_min, 0.05);
    nh.param("tau_omega_max", params_.tau_omega_max, 1.00);
    nh.param("tau_filter_alpha", params_.tau_filter_alpha, 0.02);
    nh.param("tau_min_response_delta",
             params_.tau_min_response_delta, 0.03);
    nh.param("command_alignment_delay",
             params_.command_alignment_delay, 0.0);
    nh.param("command_alignment_max_age",
             params_.command_alignment_max_age, 0.30);
    nh.param("angular_deadband", params_.angular_deadband, 0.05);
    nh.param("state_timeout", params_.state_timeout, 0.30);
    nh.param("tf_timeout", params_.tf_timeout, 0.30);
    nh.param("max_position_covariance",
             params_.max_position_covariance, 1.0);
    nh.param("max_yaw_covariance", params_.max_yaw_covariance, 0.50);
    nh.param("max_control_cycle_interval",
             params_.max_control_cycle_interval, 0.25);
    nh.param("solver_time_limit", params_.solver_time_limit, 0.080);
    nh.param("max_consecutive_solver_timeouts",
             params_.max_consecutive_solver_timeouts, 3);
    nh.param("require_localization_quality",
             params_.require_localization_quality, false);
    nh.param("localization_quality_timeout",
             params_.localization_quality_timeout, 0.30);
    nh.param("projection_search_back_segments",
             params_.projection_search_back_segments, 20);
    nh.param("projection_search_forward_segments",
             params_.projection_search_forward_segments, 80);
    nh.param("projection_global_fallback_distance",
             params_.projection_global_fallback_distance, 2.0);

    params_.linear_gain_min = std::max(0.05, params_.linear_gain_min);
    params_.linear_gain_max =
        std::max(params_.linear_gain_min, params_.linear_gain_max);
    params_.linear_gain_filter_alpha = std::max(
        0.0, std::min(1.0, params_.linear_gain_filter_alpha));
    params_.linear_gain_min_command =
        std::max(0.01, params_.linear_gain_min_command);
    params_.linear_gain_stable_delta =
        std::max(0.0, params_.linear_gain_stable_delta);
    params_.linear_gain_stable_cycles =
        std::max(1, params_.linear_gain_stable_cycles);
    params_.omega_gain_min = std::max(0.05, params_.omega_gain_min);
    params_.omega_gain_max =
        std::max(params_.omega_gain_min, params_.omega_gain_max);
    params_.omega_gain_filter_alpha = std::max(
        0.0, std::min(1.0, params_.omega_gain_filter_alpha));
    params_.omega_gain_rise_alpha = std::max(
        0.0, std::min(params_.omega_gain_filter_alpha,
                      params_.omega_gain_rise_alpha));
    params_.omega_gain_min_command =
        std::max(0.005, params_.omega_gain_min_command);
    params_.omega_gain_stable_delta =
        std::max(0.0, params_.omega_gain_stable_delta);
    params_.omega_gain_stable_cycles =
        std::max(1, params_.omega_gain_stable_cycles);
    params_.tau_v_min = std::max(0.01, params_.tau_v_min);
    params_.tau_v_max = std::max(params_.tau_v_min, params_.tau_v_max);
    params_.tau_omega_min = std::max(0.01, params_.tau_omega_min);
    params_.tau_omega_max =
        std::max(params_.tau_omega_min, params_.tau_omega_max);
    params_.tau_filter_alpha = std::max(
        0.0, std::min(1.0, params_.tau_filter_alpha));
    params_.tau_min_response_delta =
        std::max(0.005, params_.tau_min_response_delta);
    params_.command_alignment_delay = std::max(
        0.0, std::min(1.0, params_.command_alignment_delay));
    params_.command_alignment_max_age = std::max(
        0.05, std::min(2.0, params_.command_alignment_max_age));
    params_.state_timeout = std::max(0.05, params_.state_timeout);
    params_.tf_timeout = std::max(0.05, params_.tf_timeout);
    params_.max_position_covariance = std::max(
        0.0, params_.max_position_covariance);
    params_.max_yaw_covariance = std::max(
        0.0, params_.max_yaw_covariance);
    params_.max_control_cycle_interval = std::max(
        0.10, params_.max_control_cycle_interval);
    params_.solver_time_limit = std::max(
        0.005, params_.solver_time_limit);
    params_.max_consecutive_solver_timeouts = std::max(
        1, params_.max_consecutive_solver_timeouts);
    params_.localization_quality_timeout = std::max(
        0.05, params_.localization_quality_timeout);
    params_.projection_search_back_segments = std::max(
        1, params_.projection_search_back_segments);
    params_.projection_search_forward_segments = std::max(
        1, params_.projection_search_forward_segments);
    params_.projection_global_fallback_distance = std::max(
        0.1, params_.projection_global_fallback_distance);
    identified_linear_gain_ = std::max(
        params_.linear_gain_min,
        std::min(params_.linear_gain_max, params_.linear_tracking_gain));
    identified_omega_gain_ = std::max(
        params_.omega_gain_min,
        std::min(params_.omega_gain_max, params_.omega_tracking_gain));
    identified_tau_v_ = std::max(
        params_.tau_v_min, std::min(params_.tau_v_max, params_.tau_v));
    identified_tau_omega_ = std::max(
        params_.tau_omega_min,
        std::min(params_.tau_omega_max, params_.tau_omega));

    nh.param("max_vel_x", params_.max_vel_x, 1.0);
    nh.param("max_vel_x_backwards", params_.max_vel_x_backwards, 0.3);
    nh.param("max_vel_theta", params_.max_vel_theta, 0.35);
    nh.param("acc_lim_x", params_.acc_lim_x, 0.8);
    nh.param("acc_lim_theta", params_.acc_lim_theta, 0.25);
    nh.param("curve_acc_lim_theta", params_.curve_acc_lim_theta, 0.5);
    nh.param("curve_threshold", params_.curve_threshold, 0.1);

    nh.param("prediction_dt", params_.prediction_dt, 0.1);
    nh.param("prediction_steps", params_.prediction_steps, 20);
    nh.param("num_omega_samples", params_.num_omega_samples, 41);
    nh.param("num_v_samples", params_.num_v_samples, 3);
    nh.param("control_hold_steps", params_.control_hold_steps, 6);
    nh.param("num_second_omega_samples", params_.num_second_omega_samples, 9);

    nh.param("weight_cte", params_.weight_cte, 3.0);
    nh.param("weight_heading", params_.weight_heading, 1.0);
    nh.param("weight_omega", params_.weight_omega, 0.5);
    nh.param("weight_omega_rate", params_.weight_omega_rate, 2.0);
    nh.param("weight_speed", params_.weight_speed, 1.0);
    nh.param("weight_v_rate", params_.weight_v_rate, 0.5);
    nh.param("weight_curvature_feedforward",
             params_.weight_curvature_feedforward, 1.0);
    nh.param("weight_obstacle", params_.weight_obstacle, 10.0);

    nh.param("goal_tolerance_xy", params_.goal_tolerance_xy, 0.5);
    nh.param("goal_tolerance_yaw", params_.goal_tolerance_yaw, 0.3);

    nh.param("curvature_decel", params_.curvature_decel, 0.5);
    nh.param("goal_decel_dist", params_.goal_decel_dist, 3.0);
    nh.param("min_vel_x", params_.min_vel_x, 0.05);
    nh.param("terminal_recovery_speed",
             params_.terminal_recovery_speed, 0.15);
    nh.param("terminal_alignment_distance",
             params_.terminal_alignment_distance, 2.0);
    nh.param("terminal_bearing_tolerance",
             params_.terminal_bearing_tolerance, 0.20);
    nh.param("terminal_bearing_gain",
             params_.terminal_bearing_gain, 0.8);
    params_.terminal_recovery_speed = std::max(
        params_.min_vel_x,
        std::min(params_.max_vel_x, params_.terminal_recovery_speed));
    params_.terminal_alignment_distance = std::max(
        params_.goal_tolerance_xy, params_.terminal_alignment_distance);
    params_.terminal_bearing_tolerance = std::max(
        0.05, std::min(0.5 * M_PI,
                       params_.terminal_bearing_tolerance));
    params_.terminal_bearing_gain =
        std::max(0.1, params_.terminal_bearing_gain);

    nh.param("heading_lookahead", params_.heading_lookahead, 1.5);
    nh.param("cte_heading_gain", params_.cte_heading_gain, 0.40);
    nh.param("cte_heading_soft_speed", params_.cte_heading_soft_speed, 0.20);
    nh.param("max_cte_heading_correction",
             params_.max_cte_heading_correction, 0.7854);
    nh.param("enable_turn_authority_assist",
             params_.enable_turn_authority_assist, true);
    nh.param("turn_assist_cte_threshold",
             params_.turn_assist_cte_threshold, 0.10);
    nh.param("turn_assist_release_cte_threshold",
             params_.turn_assist_release_cte_threshold, 0.08);
    nh.param("turn_assist_heading_threshold",
             params_.turn_assist_heading_threshold, 0.0524);
    nh.param("turn_assist_time_constant",
             params_.turn_assist_time_constant, 1.5);
    nh.param("turn_assist_cte_rate_alpha",
             params_.turn_assist_cte_rate_alpha, 0.20);
    nh.param("turn_assist_cte_rate_threshold",
             params_.turn_assist_cte_rate_threshold, 0.01);
    nh.param("turn_assist_cte_rate_gain",
             params_.turn_assist_cte_rate_gain, 0.80);
    nh.param("turn_assist_rate_heading_alpha",
             params_.turn_assist_rate_heading_alpha, 0.35);
    nh.param("turn_assist_max_rate_heading",
             params_.turn_assist_max_rate_heading, 0.1745);
    nh.param("turn_assist_release_time",
             params_.turn_assist_release_time, 1.0);
    params_.cte_heading_gain = std::max(0.0, params_.cte_heading_gain);
    params_.cte_heading_soft_speed =
        std::max(0.01, params_.cte_heading_soft_speed);
    params_.max_cte_heading_correction = std::max(
        0.0, std::min(0.5 * M_PI, params_.max_cte_heading_correction));
    params_.turn_assist_cte_threshold =
        std::max(0.0, params_.turn_assist_cte_threshold);
    params_.turn_assist_release_cte_threshold = std::max(
        0.0, std::min(params_.turn_assist_cte_threshold,
                      params_.turn_assist_release_cte_threshold));
    params_.turn_assist_heading_threshold = std::max(
        0.0, std::min(0.5 * M_PI,
                      params_.turn_assist_heading_threshold));
    params_.turn_assist_time_constant =
        std::max(0.5, params_.turn_assist_time_constant);
    params_.turn_assist_cte_rate_alpha = std::max(
        0.01, std::min(1.0, params_.turn_assist_cte_rate_alpha));
    params_.turn_assist_cte_rate_threshold =
        std::max(0.0, params_.turn_assist_cte_rate_threshold);
    params_.turn_assist_cte_rate_gain =
        std::max(0.0, params_.turn_assist_cte_rate_gain);
    params_.turn_assist_rate_heading_alpha = std::max(
        0.01, std::min(1.0,
                      params_.turn_assist_rate_heading_alpha));
    params_.turn_assist_max_rate_heading = std::max(
        0.0, std::min(0.5 * M_PI,
                      params_.turn_assist_max_rate_heading));
    params_.turn_assist_release_time =
        std::max(0.0, params_.turn_assist_release_time);
    nh.param("enable_lateral_disturbance_observer",
             params_.enable_lateral_disturbance_observer, true);
    nh.param("lateral_disturbance_time_constant",
             params_.lateral_disturbance_time_constant, 0.8);
    nh.param("lateral_disturbance_release_time_constant",
             params_.lateral_disturbance_release_time_constant, 0.6);
    nh.param("lateral_disturbance_min_speed",
             params_.lateral_disturbance_min_speed, 0.05);
    nh.param("lateral_disturbance_deadband",
             params_.lateral_disturbance_deadband, 0.015);
    nh.param("max_lateral_disturbance",
             params_.max_lateral_disturbance, 0.12);
    nh.param("max_lateral_disturbance_heading",
             params_.max_lateral_disturbance_heading, 0.2094);
    params_.lateral_disturbance_time_constant = std::max(
        0.10, params_.lateral_disturbance_time_constant);
    params_.lateral_disturbance_release_time_constant = std::max(
        0.10, params_.lateral_disturbance_release_time_constant);
    params_.lateral_disturbance_min_speed = std::max(
        0.0, params_.lateral_disturbance_min_speed);
    params_.lateral_disturbance_deadband = std::max(
        0.0, params_.lateral_disturbance_deadband);
    params_.max_lateral_disturbance = std::max(
        params_.lateral_disturbance_deadband,
        params_.max_lateral_disturbance);
    params_.max_lateral_disturbance_heading = std::max(
        0.0, std::min(0.5 * M_PI,
                      params_.max_lateral_disturbance_heading));
    nh.param("terrain_decel", params_.terrain_decel, 0.6);
    nh.param("slip_compensation_gain", params_.slip_compensation_gain, 0.3);
    nh.param<std::string>("terrain_cost_file", terrain_cost_file_, "");

    if (!terrain_cost_file_.empty())
        loadTerrainMap(terrain_cost_file_, terrain_map_);

    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "/vehicle/state");
    odom_sub_ = ros::NodeHandle().subscribe(odom_topic, 1, &MpcLocalPlanner::odomCallback, this);
    std::string localization_quality_topic;
    nh.param<std::string>("localization_quality_topic",
                          localization_quality_topic,
                          "/localization/quality_ok");
    if (params_.require_localization_quality) {
        localization_quality_sub_ = ros::NodeHandle().subscribe(
            localization_quality_topic, 1,
            &MpcLocalPlanner::localizationQualityCallback, this);
    }
    local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "diagnostics", 10);
    prediction_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "prediction_diagnostics", 10);

    initialized_ = true;
    ROS_INFO("MPC Local Planner initialized (tau_v=%.3f, tau_omega=%.3f, "
             "linear_gain=%.2f adaptive=%s, yaw_gain=%.2f adaptive=%s, "
             "tau_adaptive=%s, deadband=%.3f, heading_lookahead=%.1fm, "
             "control_samples=v:%d omega:%d+%d, "
             "terrain=%s, lateral_residual_observer=%s tau=%.2fs max=%.2fm/s, "
             "terrain_decel=%.2f, yaw_slip_gain=%.2f)",
             params_.tau_v, params_.tau_omega, identified_linear_gain_,
             params_.enable_linear_gain_adaptation ? "on" : "off",
             identified_omega_gain_,
             params_.enable_omega_gain_adaptation ? "on" : "off",
             params_.enable_time_constant_adaptation ? "on" : "off",
             params_.angular_deadband,
             params_.heading_lookahead,
             params_.num_v_samples, params_.num_omega_samples,
             params_.num_second_omega_samples,
             terrain_map_.loaded ? "loaded" : "none",
             params_.enable_lateral_disturbance_observer ? "on" : "off",
             params_.lateral_disturbance_time_constant,
             params_.max_lateral_disturbance,
             params_.terrain_decel, params_.slip_compensation_gain);
}

// ==================== Terrain map I/O ====================

bool MpcLocalPlanner::loadTerrainMap(const std::string& path, TerrainMap& map) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { ROS_WARN("MPC: cannot open terrain cost: %s", path.c_str()); return false; }

    char magic[4];
    f.read(magic, 4);
    if (std::strncmp(magic, "TCM1", 4) != 0) {
        ROS_WARN("MPC: invalid magic in %s", path.c_str()); return false;
    }

    f.read(reinterpret_cast<char*>(&map.rows), 4);
    f.read(reinterpret_cast<char*>(&map.cols), 4);
    f.read(reinterpret_cast<char*>(&map.res), 4);
    f.read(reinterpret_cast<char*>(&map.origin_x), 4);
    f.read(reinterpret_cast<char*>(&map.origin_y), 4);
    f.read(reinterpret_cast<char*>(&map.friction_coeff), 4);
    f.read(reinterpret_cast<char*>(&map.track_width), 4);
    f.read(reinterpret_cast<char*>(&map.cg_height), 4);

    int n = map.rows * map.cols;
    map.dem.resize(n);
    map.grad_x.resize(n);
    map.grad_y.resize(n);
    map.cost_total.resize(n);

    f.read(reinterpret_cast<char*>(map.dem.data()), n * sizeof(float));
    f.read(reinterpret_cast<char*>(map.grad_x.data()), n * sizeof(float));
    f.read(reinterpret_cast<char*>(map.grad_y.data()), n * sizeof(float));
    f.read(reinterpret_cast<char*>(map.cost_total.data()), n * sizeof(float));

    if (!f.good()) { ROS_WARN("MPC: terrain cost read incomplete"); return false; }

    map.loaded = true;
    ROS_INFO("MPC terrain cost loaded: %s (%dx%d, %.2fm/cell)",
             path.c_str(), map.cols, map.rows, map.res);
    return true;
}

float MpcLocalPlanner::bilinearQuery(const TerrainMap& map,
                                      const std::vector<float>& layer,
                                      double x, double y) const {
    double cx = (x - map.origin_x) / map.res - 0.5;
    double cy = (y - map.origin_y) / map.res - 0.5;
    int c0 = std::max(0, std::min((int)floor(cx), map.cols - 2));
    int r0 = std::max(0, std::min((int)floor(cy), map.rows - 2));

    float fx = std::max(0.0f, std::min(1.0f, (float)(cx - c0)));
    float fy = std::max(0.0f, std::min(1.0f, (float)(cy - r0)));

    float v00 = layer[r0 * map.cols + c0];
    float v10 = layer[r0 * map.cols + c0 + 1];
    float v01 = layer[(r0 + 1) * map.cols + c0];
    float v11 = layer[(r0 + 1) * map.cols + c0 + 1];

    int nan_cnt = 0;
    float valid_sum = 0;
    for (float v : {v00, v10, v01, v11}) {
        if (std::isnan(v)) nan_cnt++;
        else valid_sum += v;
    }
    if (nan_cnt == 4) return 0.0f;
    if (nan_cnt > 0) {
        float fill = valid_sum / (4 - nan_cnt);
        if (std::isnan(v00)) v00 = fill;
        if (std::isnan(v10)) v10 = fill;
        if (std::isnan(v01)) v01 = fill;
        if (std::isnan(v11)) v11 = fill;
    }

    return v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy)
         + v01 * (1 - fx) * fy        + v11 * fx * fy;
}

bool MpcLocalPlanner::controlToTerrainFrame(
        double x, double y, double theta,
        double& terrain_x, double& terrain_y,
        double& terrain_theta) const {
    if (!terrain_query_transform_valid_) return false;

    // updatePlanInCostmapFrame() stores T_control_plan. Terrain arrays use
    // the plan/map frame, therefore query with its inverse T_plan_control.
    const double dx = x - plan_to_control_x_;
    const double dy = y - plan_to_control_y_;
    const double c = std::cos(plan_to_control_yaw_);
    const double s = std::sin(plan_to_control_yaw_);
    terrain_x = c * dx + s * dy;
    terrain_y = -s * dx + c * dy;
    terrain_theta = normalizeAngle(theta - plan_to_control_yaw_);
    return true;
}

float MpcLocalPlanner::queryTerrainCost(double x, double y) const {
    if (!terrain_map_.loaded) return 0.0f;
    double terrain_x, terrain_y, terrain_theta;
    if (!controlToTerrainFrame(x, y, 0.0,
                               terrain_x, terrain_y, terrain_theta)) {
        return 0.0f;
    }
    return std::max(
        0.0f, bilinearQuery(terrain_map_, terrain_map_.cost_total,
                            terrain_x, terrain_y));
}

// Slope-induced yaw disturbance: lateral slope creates differential traction
// on left/right tracks, producing a turning bias toward the downhill side.
// ω_slip ≈ k * slope_lat * v  (proportional to lateral slope and speed)
double MpcLocalPlanner::querySlipOmega(double x, double y,
                                        double theta, double v) const {
    if (!terrain_map_.loaded || std::fabs(v) < 0.01) return 0.0;

    double terrain_x, terrain_y, terrain_theta;
    if (!controlToTerrainFrame(x, y, theta,
                               terrain_x, terrain_y, terrain_theta)) {
        return 0.0;
    }

    float gx = bilinearQuery(
        terrain_map_, terrain_map_.grad_x, terrain_x, terrain_y);
    float gy = bilinearQuery(
        terrain_map_, terrain_map_.grad_y, terrain_x, terrain_y);

    double slope_lat =
        -gx * sin(terrain_theta) + gy * cos(terrain_theta);

    return -params_.slip_compensation_gain * slope_lat * v;
}

// ==================== Callbacks ====================

void MpcLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // /vehicle/state is normalized by the active localization adapter.  ROS
    // Odometry defines twist in child_frame_id, which is always base_link for
    // this contract: x is forward and y is left.  Never rotate it a second
    // time in the controller.
    const double forward_v = msg->twist.twist.linear.x;
    const double lateral_v = msg->twist.twist.linear.y;
    const double omega = msg->twist.twist.angular.z;
    const double position_covariance = std::max(
        msg->pose.covariance[0], msg->pose.covariance[7]);
    const double yaw_covariance = msg->pose.covariance[35];

    if (!std::isfinite(forward_v) || !std::isfinite(lateral_v) ||
        !std::isfinite(omega) || !std::isfinite(position_covariance) ||
        !std::isfinite(yaw_covariance) || position_covariance < 0.0 ||
        yaw_covariance < 0.0) {
        ROS_WARN_THROTTLE(
            1.0, "MPC: ignoring non-finite/invalid vehicle state");
        return;
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_v_ = forward_v;
    current_lateral_v_ = lateral_v;
    current_omega_ = omega;
    current_state_measurement_stamp_ = msg->header.stamp;
    current_position_covariance_ = position_covariance;
    current_yaw_covariance_ = yaw_covariance;
}

void MpcLocalPlanner::localizationQualityCallback(
        const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(localization_quality_mutex_);
    localization_quality_ok_ = msg->data;
    last_localization_quality_time_ = ros::Time::now();
}

bool MpcLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) return false;
    source_plan_ = plan;
    global_plan_.clear();
    terrain_query_transform_valid_ = false;
    goal_reached_ = false;
    // Start each navigation from the calibrated flat-ground response. The
    // estimator then follows a persistent loss of steering authority on a
    // slope without carrying the previous experiment into the next one.
    identified_linear_gain_ = std::max(
        params_.linear_gain_min,
        std::min(params_.linear_gain_max, params_.linear_tracking_gain));
    identified_omega_gain_ = std::max(
        params_.omega_gain_min,
        std::min(params_.omega_gain_max, params_.omega_tracking_gain));
    identified_tau_v_ = std::max(
        params_.tau_v_min, std::min(params_.tau_v_max, params_.tau_v));
    identified_tau_omega_ = std::max(
        params_.tau_omega_min,
        std::min(params_.tau_omega_max, params_.tau_omega));
    previous_measured_v_ = 0.0;
    previous_measured_omega_ = 0.0;
    last_response_update_time_ = ros::Time(0);
    last_response_measurement_stamp_ = ros::Time(0);
    response_model_initialized_ = false;
    aligned_response_v_cmd_ = 0.0;
    aligned_response_omega_cmd_ = 0.0;
    response_command_alignment_age_ = 0.0;
    linear_gain_probe_cmd_ = 0.0;
    linear_gain_stable_cycles_ = 0;
    gain_probe_cmd_ = 0.0;
    gain_stable_cycles_ = 0;
    previous_signed_cte_ = 0.0;
    filtered_cte_rate_ = 0.0;
    observed_lateral_disturbance_ = 0.0;
    estimated_lateral_disturbance_ = 0.0;
    last_lateral_disturbance_update_time_ = ros::Time(0);
    held_rate_heading_correction_ = 0.0;
    turn_assist_improving_time_ = 0.0;
    last_cte_update_time_ = ros::Time(0);
    turn_assist_latched_ = false;
    one_step_cte_prediction_valid_ = false;
    one_step_predicted_cte_ = 0.0;
    one_step_prediction_residual_ = 0.0;
    one_step_prediction_created_time_ = ros::Time(0);
    projection_segment_cache_ = -1;
    last_control_cycle_start_ = ros::WallTime(0);
    last_control_cycle_interval_ = 0.0;
    last_solver_duration_ = 0.0;
    last_solver_timed_out_ = false;
    last_evaluated_candidates_ = 0;
    consecutive_solver_timeouts_ = 0;
    {
        std::lock_guard<std::mutex> history_lock(command_history_mutex_);
        command_history_.clear();
    }
    return true;
}

double MpcLocalPlanner::effectiveLinearTrackingGain() const {
    return params_.enable_linear_gain_adaptation
               ? identified_linear_gain_
               : params_.linear_tracking_gain;
}

double MpcLocalPlanner::effectiveOmegaTrackingGain() const {
    return params_.enable_omega_gain_adaptation
               ? identified_omega_gain_
               : params_.omega_tracking_gain;
}

double MpcLocalPlanner::effectiveTauV() const {
    return params_.enable_time_constant_adaptation
               ? identified_tau_v_ : params_.tau_v;
}

double MpcLocalPlanner::effectiveTauOmega() const {
    return params_.enable_time_constant_adaptation
               ? identified_tau_omega_ : params_.tau_omega;
}

void MpcLocalPlanner::recordIssuedCommand(double v, double omega) {
    const ros::Time now = ros::Time::now();
    std::lock_guard<std::mutex> lock(command_history_mutex_);
    command_history_.push_back({now, v, omega});
    const ros::Time cutoff = now - ros::Duration(5.0);
    while (!command_history_.empty() &&
           command_history_.front().stamp < cutoff) {
        command_history_.pop_front();
    }
}

bool MpcLocalPlanner::commandAtMeasurement(
        const ros::Time& measurement_stamp, double& v, double& omega,
        double& alignment_age) const {
    if (measurement_stamp.isZero()) return false;
    const ros::Time target = measurement_stamp -
        ros::Duration(params_.command_alignment_delay);
    std::lock_guard<std::mutex> lock(command_history_mutex_);
    for (auto it = command_history_.rbegin();
         it != command_history_.rend(); ++it) {
        if (it->stamp <= target) {
            alignment_age = (target - it->stamp).toSec();
            if (alignment_age < 0.0 ||
                alignment_age > params_.command_alignment_max_age) {
                return false;
            }
            v = it->v;
            omega = it->omega;
            return true;
        }
    }
    return false;
}

void MpcLocalPlanner::updateResponseModel(
        double actual_v, double actual_omega,
        const ros::Time& measurement_stamp) {
    // Match every response to the command that was actually present at its
    // measurement time. Repeated 50 Hz fusion publications of one GNSS sample
    // carry the same stamp and therefore update the estimator exactly once.
    double aligned_v_command = 0.0;
    double aligned_omega_command = 0.0;
    double alignment_age = 0.0;
    if (!commandAtMeasurement(measurement_stamp, aligned_v_command,
                              aligned_omega_command, alignment_age)) {
        return;
    }
    if (!last_response_measurement_stamp_.isZero() &&
        measurement_stamp <= last_response_measurement_stamp_) {
        return;
    }

    updateOmegaTrackingGain(actual_omega, aligned_omega_command);

    if (!response_model_initialized_ ||
        last_response_measurement_stamp_.isZero()) {
        previous_measured_v_ = actual_v;
        previous_measured_omega_ = actual_omega;
        last_response_update_time_ = ros::Time::now();
        last_response_measurement_stamp_ = measurement_stamp;
        aligned_response_v_cmd_ = aligned_v_command;
        aligned_response_omega_cmd_ = aligned_omega_command;
        response_command_alignment_age_ = alignment_age;
        linear_gain_probe_cmd_ = aligned_v_command;
        gain_probe_cmd_ = aligned_omega_command;
        response_model_initialized_ = true;
        return;
    }

    const double dt =
        (measurement_stamp - last_response_measurement_stamp_).toSec();
    last_response_update_time_ = ros::Time::now();
    last_response_measurement_stamp_ = measurement_stamp;
    response_command_alignment_age_ = alignment_age;
    if (dt <= 0.01 || dt > 0.5 || !std::isfinite(actual_v) ||
        !std::isfinite(actual_omega)) {
        previous_measured_v_ = actual_v;
        previous_measured_omega_ = actual_omega;
        aligned_response_v_cmd_ = aligned_v_command;
        aligned_response_omega_cmd_ = aligned_omega_command;
        return;
    }

    const bool linear_command_stable =
        std::fabs(aligned_v_command - linear_gain_probe_cmd_) <=
        params_.linear_gain_stable_delta;
    if (std::fabs(aligned_v_command) >= params_.linear_gain_min_command &&
        linear_command_stable && aligned_v_command * actual_v > 0.0) {
        ++linear_gain_stable_cycles_;
    } else {
        linear_gain_stable_cycles_ = 0;
    }
    linear_gain_probe_cmd_ = aligned_v_command;

    if (params_.enable_linear_gain_adaptation &&
        linear_gain_stable_cycles_ >= params_.linear_gain_stable_cycles) {
        const double observed = std::max(
            params_.linear_gain_min,
            std::min(params_.linear_gain_max,
                     std::fabs(actual_v / aligned_v_command)));
        identified_linear_gain_ += params_.linear_gain_filter_alpha *
            (observed - identified_linear_gain_);
    }

    if (params_.enable_time_constant_adaptation) {
        const auto update_tau = [dt, this](
                double previous, double current, double target,
                double minimum, double maximum, double& estimate) {
            const double previous_error = previous - target;
            const double current_error = current - target;
            if (std::fabs(previous_error) < params_.tau_min_response_delta ||
                previous_error * current_error <= 0.0) {
                return;
            }
            const double ratio = current_error / previous_error;
            if (!(ratio > 0.05 && ratio < 0.98)) return;
            const double observed_tau = -dt / std::log(ratio);
            if (!std::isfinite(observed_tau)) return;
            const double bounded = std::max(
                minimum, std::min(maximum, observed_tau));
            estimate += params_.tau_filter_alpha * (bounded - estimate);
        };

        if (linear_command_stable) {
            const double v_target =
                effectiveLinearTrackingGain() * aligned_v_command;
            update_tau(previous_measured_v_, actual_v, v_target,
                       params_.tau_v_min, params_.tau_v_max,
                       identified_tau_v_);
        }

        const bool omega_command_stable =
            std::fabs(aligned_omega_command - aligned_response_omega_cmd_) <=
            params_.omega_gain_stable_delta;
        if (omega_command_stable) {
            const double omega_target =
                effectiveOmegaTrackingGain() * aligned_omega_command;
            update_tau(previous_measured_omega_, actual_omega, omega_target,
                       params_.tau_omega_min, params_.tau_omega_max,
                       identified_tau_omega_);
        }
    }

    previous_measured_v_ = actual_v;
    previous_measured_omega_ = actual_omega;
    aligned_response_v_cmd_ = aligned_v_command;
    aligned_response_omega_cmd_ = aligned_omega_command;
}

void MpcLocalPlanner::updateLateralDisturbanceEstimate(
        double heading_error, double cte_prediction_residual,
        double prediction_dt, bool measurement_valid) {
    const ros::Time now = ros::Time::now();
    if (last_lateral_disturbance_update_time_.isZero()) {
        last_lateral_disturbance_update_time_ = now;
        observed_lateral_disturbance_ = 0.0;
        return;
    }

    const double dt = (now - last_lateral_disturbance_update_time_).toSec();
    last_lateral_disturbance_update_time_ = now;
    if (dt <= 0.0 || dt > 0.5) return;

    double target = 0.0;
    double tau = params_.lateral_disturbance_release_time_constant;
    const double heading_projection = std::cos(heading_error);
    if (params_.enable_lateral_disturbance_observer &&
        measurement_valid &&
        std::fabs(heading_projection) >= 0.5 &&
        prediction_dt > 0.01 && prediction_dt < 0.5 &&
        std::isfinite(cte_prediction_residual) &&
        std::isfinite(heading_error)) {
        // Compare this cycle's measured CTE with the CTE predicted one control
        // interval ago. The prediction already contains the current augmented
        // disturbance estimate, so the residual represents the missing
        // disturbance increment:
        //   r_cte = e_measured(k) - e_predicted(k|k-1)
        //   delta_v_lat = -r_cte / (dt*cos(e_psi))
        // signed_cte is positive to the path's right while body lateral
        // velocity is positive to the vehicle's left, hence the minus sign.
        observed_lateral_disturbance_ =
            -cte_prediction_residual /
            (prediction_dt * heading_projection);
        if (std::fabs(observed_lateral_disturbance_) <
            params_.lateral_disturbance_deadband) {
            observed_lateral_disturbance_ = 0.0;
        }
        target = std::max(-params_.max_lateral_disturbance,
                          std::min(params_.max_lateral_disturbance,
                                   estimated_lateral_disturbance_ +
                                       observed_lateral_disturbance_));
        tau = params_.lateral_disturbance_time_constant;
    } else {
        observed_lateral_disturbance_ = 0.0;
    }

    const double alpha = 1.0 - std::exp(-dt / tau);
    estimated_lateral_disturbance_ +=
        alpha * (target - estimated_lateral_disturbance_);
}

double MpcLocalPlanner::lateralDisturbanceHeading(
        double forward_v, double lateral_disturbance) const {
    if (!params_.enable_lateral_disturbance_observer) return 0.0;

    // Counter-steer against body-frame lateral motion so that the resulting
    // world-frame velocity follows the path tangent. The soft speed avoids an
    // excessive reference close to standstill.
    const double raw = -std::atan2(
        lateral_disturbance,
        std::fabs(forward_v) + params_.cte_heading_soft_speed);
    return std::max(-params_.max_lateral_disturbance_heading,
                    std::min(params_.max_lateral_disturbance_heading, raw));
}

void MpcLocalPlanner::updateOmegaTrackingGain(
        double actual_omega, double aligned_omega_command) {
    if (!params_.enable_omega_gain_adaptation) return;

    const double command = aligned_omega_command;
    const bool command_large_enough =
        std::fabs(command) >= params_.omega_gain_min_command;
    const bool command_stable =
        std::fabs(command - gain_probe_cmd_) <=
        params_.omega_gain_stable_delta;
    const bool response_same_direction = command * actual_omega > 0.0;

    if (command_large_enough && command_stable &&
        response_same_direction && std::isfinite(actual_omega)) {
        ++gain_stable_cycles_;
    } else {
        gain_stable_cycles_ = 0;
    }
    gain_probe_cmd_ = command;

    if (gain_stable_cycles_ < params_.omega_gain_stable_cycles) return;

    const double observed_gain = std::fabs(actual_omega / command);
    if (!std::isfinite(observed_gain)) return;

    const double bounded_gain = std::max(
        params_.omega_gain_min,
        std::min(params_.omega_gain_max, observed_gain));
    // A side slope can briefly rotate the chassis in the commanded direction
    // even though sustained steering authority is still poor. Treating that
    // transient as an immediate gain recovery made the model jump from about
    // 0.22 back above 0.5 and prematurely reduce the steering command. Follow
    // a loss of authority quickly, but require persistent evidence to raise
    // the model again.
    const double alpha = bounded_gain < identified_omega_gain_
        ? params_.omega_gain_filter_alpha
        : params_.omega_gain_rise_alpha;
    identified_omega_gain_ =
        (1.0 - alpha) * identified_omega_gain_ + alpha * bounded_gain;
}

bool MpcLocalPlanner::isGoalReached() {
    return goal_reached_;
}

void MpcLocalPlanner::issueSafeStop(geometry_msgs::Twist& cmd_vel,
                                    const char* reason) {
    cmd_vel = geometry_msgs::Twist();
    prev_v_cmd_ = 0.0;
    prev_omega_cmd_ = 0.0;
    one_step_cte_prediction_valid_ = false;
    recordIssuedCommand(0.0, 0.0);
    ROS_ERROR_THROTTLE(1.0, "MPC watchdog stop: %s", reason);
}

bool MpcLocalPlanner::controlInputsHealthy(
        const geometry_msgs::PoseStamped& robot_pose,
        const ros::Time& measurement_stamp,
        double position_covariance, double yaw_covariance,
        double& state_age, double& tf_age) const {
    const ros::Time now = ros::Time::now();
    state_age = std::numeric_limits<double>::infinity();
    tf_age = std::numeric_limits<double>::infinity();

    if (!std::isfinite(robot_pose.pose.position.x) ||
        !std::isfinite(robot_pose.pose.position.y) ||
        !std::isfinite(tf2::getYaw(robot_pose.pose.orientation))) {
        ROS_WARN_THROTTLE(1.0, "MPC watchdog: robot pose is non-finite");
        return false;
    }
    if (measurement_stamp.isZero()) {
        ROS_WARN_THROTTLE(1.0,
                          "MPC watchdog: /vehicle/state has zero stamp");
        return false;
    }
    state_age = (now - measurement_stamp).toSec();
    if (!std::isfinite(state_age) || state_age < -0.05 ||
        state_age > params_.state_timeout) {
        ROS_WARN_THROTTLE(
            1.0, "MPC watchdog: state age %.3fs outside [-0.05, %.3f]s",
            state_age, params_.state_timeout);
        return false;
    }
    if (!std::isfinite(position_covariance) ||
        !std::isfinite(yaw_covariance) || position_covariance < 0.0 ||
        yaw_covariance < 0.0 ||
        position_covariance > params_.max_position_covariance ||
        yaw_covariance > params_.max_yaw_covariance) {
        ROS_WARN_THROTTLE(
            1.0,
            "MPC watchdog: pose covariance xy=%.3f yaw=%.3f exceeds "
            "limits %.3f/%.3f",
            position_covariance, yaw_covariance,
            params_.max_position_covariance, params_.max_yaw_covariance);
        return false;
    }

    try {
        const geometry_msgs::TransformStamped latest_tf =
            tf_->lookupTransform(robot_pose.header.frame_id,
                                 costmap_ros_->getBaseFrameID(),
                                 ros::Time(0), ros::Duration(0.01));
        if (latest_tf.header.stamp.isZero()) {
            ROS_WARN_THROTTLE(1.0,
                              "MPC watchdog: latest control TF has zero stamp");
            return false;
        }
        tf_age = (now - latest_tf.header.stamp).toSec();
        if (!std::isfinite(tf_age) || tf_age < -0.05 ||
            tf_age > params_.tf_timeout) {
            ROS_WARN_THROTTLE(
                1.0, "MPC watchdog: TF age %.3fs outside [-0.05, %.3f]s",
                tf_age, params_.tf_timeout);
            return false;
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "MPC TF watchdog lookup failed: %s",
                          ex.what());
        return false;
    }

    if (params_.require_localization_quality) {
        bool quality_ok = false;
        ros::Time quality_stamp;
        {
            std::lock_guard<std::mutex> lock(localization_quality_mutex_);
            quality_ok = localization_quality_ok_;
            quality_stamp = last_localization_quality_time_;
        }
        const double quality_age = quality_stamp.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - quality_stamp).toSec();
        if (!quality_ok || !std::isfinite(quality_age) ||
            quality_age < -0.05 ||
            quality_age > params_.localization_quality_timeout) {
            ROS_WARN_THROTTLE(
                1.0,
                "MPC watchdog: localization quality ok=%s age=%.3fs "
                "limit=%.3fs",
                quality_ok ? "true" : "false", quality_age,
                params_.localization_quality_timeout);
            return false;
        }
    }
    return true;
}

// ==================== Path utilities ====================

bool MpcLocalPlanner::updatePlanInCostmapFrame() {
    if (source_plan_.empty()) {
        global_plan_.clear();
        terrain_query_transform_valid_ = false;
        return false;
    }

    const std::string target_frame = costmap_ros_->getGlobalFrameID();
    const std::string source_frame = source_plan_.front().header.frame_id;
    if (source_frame.empty()) {
        terrain_query_transform_valid_ = false;
        ROS_WARN_THROTTLE(1.0, "MPC: global plan has an empty frame_id");
        return false;
    }

    if (source_frame == target_frame) {
        // setPlan() clears the working copy, so an identical frame needs one
        // copy per plan rather than one full-vector copy per control cycle.
        if (global_plan_.empty()) global_plan_ = source_plan_;
        plan_to_control_x_ = 0.0;
        plan_to_control_y_ = 0.0;
        plan_to_control_yaw_ = 0.0;
        terrain_query_transform_valid_ = true;
        return true;
    }

    try {
        // map->odom is owned by fixed-map localization and changes as scan
        // matching corrects drift. Use its latest value on every controller
        // cycle instead of freezing the transform when setPlan() is called.
        const geometry_msgs::TransformStamped plan_tf =
            tf_->lookupTransform(target_frame, source_frame, ros::Time(0),
                                 ros::Duration(0.05));

        global_plan_.clear();
        global_plan_.reserve(source_plan_.size());
        for (const auto& source_pose : source_plan_) {
            geometry_msgs::PoseStamped target_pose;
            tf2::doTransform(source_pose, target_pose, plan_tf);
            global_plan_.push_back(std::move(target_pose));
        }

        plan_to_control_x_ = plan_tf.transform.translation.x;
        plan_to_control_y_ = plan_tf.transform.translation.y;
        plan_to_control_yaw_ = tf2::getYaw(plan_tf.transform.rotation);
        terrain_query_transform_valid_ = true;

        ROS_INFO_THROTTLE(
            2.0,
            "MPC plan transform: %s -> %s translation=(%+.3f,%+.3f)m "
            "yaw=%+.2fdeg",
            source_frame.c_str(), target_frame.c_str(),
            plan_tf.transform.translation.x,
            plan_tf.transform.translation.y,
            tf2::getYaw(plan_tf.transform.rotation) * 180.0 / M_PI);
        return true;
    } catch (const tf2::TransformException& ex) {
        terrain_query_transform_valid_ = false;
        ROS_WARN_THROTTLE(
            1.0, "MPC: cannot transform global plan %s -> %s: %s",
            source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

MpcLocalPlanner::PathProjection MpcLocalPlanner::projectOntoPath(
        double x, double y, int segment_hint) const {
    PathProjection best = {0, 0.0, 0.0, 0.0, 0.0,
                           std::numeric_limits<double>::infinity(), false};
    if (global_plan_.empty()) return best;
    if (global_plan_.size() == 1) {
        best.x = global_plan_.front().pose.position.x;
        best.y = global_plan_.front().pose.position.y;
        best.distance = std::hypot(x - best.x, y - best.y);
        best.signed_cte = best.distance;
        best.valid = true;
        return best;
    }

    const int segment_count = static_cast<int>(global_plan_.size()) - 1;
    double best_dist2 = std::numeric_limits<double>::infinity();
    const auto scan_range = [&](int begin, int end) {
        begin = std::max(0, begin);
        end = std::min(segment_count, end);
        for (int i = begin; i < end; ++i) {
            const double ax = global_plan_[i].pose.position.x;
            const double ay = global_plan_[i].pose.position.y;
            const double bx = global_plan_[i + 1].pose.position.x;
            const double by = global_plan_[i + 1].pose.position.y;
            const double dx = bx - ax;
            const double dy = by - ay;
            const double len2 = dx * dx + dy * dy;
            if (len2 < 1e-12) continue;

            const double raw_fraction =
                ((x - ax) * dx + (y - ay) * dy) / len2;
            const double fraction =
                std::max(0.0, std::min(1.0, raw_fraction));
            const double projected_x = ax + fraction * dx;
            const double projected_y = ay + fraction * dy;
            const double error_x = x - projected_x;
            const double error_y = y - projected_y;
            const double dist2 = error_x * error_x + error_y * error_y;

            const bool strictly_better = dist2 < best_dist2 - 1e-12;
            const bool tied_and_continuous =
                std::fabs(dist2 - best_dist2) <= 1e-12 &&
                segment_hint >= 0 &&
                (!best.valid || std::abs(i - segment_hint) <
                                    std::abs(best.segment_index - segment_hint));
            if (!strictly_better && !tied_and_continuous) continue;

            const double len = std::sqrt(len2);
            best.segment_index = i;
            best.fraction = fraction;
            best.x = projected_x;
            best.y = projected_y;
            best.signed_cte =
                (dy * (x - ax) - dx * (y - ay)) / len;
            best.distance = std::sqrt(dist2);
            best.valid = true;
            best_dist2 = dist2;
        }
    };

    if (segment_hint >= 0 && segment_hint < segment_count) {
        scan_range(segment_hint - params_.projection_search_back_segments,
                   segment_hint +
                       params_.projection_search_forward_segments + 1);
    }
    // The first projection is global. Later cycles normally use only the
    // cached local window; a large localization jump deliberately triggers a
    // global reacquisition rather than keeping a stale segment forever.
    if (!best.valid ||
        best.distance > params_.projection_global_fallback_distance) {
        scan_range(0, segment_count);
    }

    if (!best.valid) {
        best.x = global_plan_.front().pose.position.x;
        best.y = global_plan_.front().pose.position.y;
        best.distance = std::hypot(x - best.x, y - best.y);
        best.signed_cte = best.distance;
        best.valid = true;
    }
    return best;
}

double MpcLocalPlanner::projectionHeading(
        const PathProjection& projection) const {
    if (global_plan_.empty()) return 0.0;
    if (global_plan_.size() == 1)
        return tf2::getYaw(global_plan_.front().pose.orientation);

    const int segment = std::max(
        0, std::min(projection.segment_index,
                    static_cast<int>(global_plan_.size()) - 2));
    // Anchor the smoothed tangent at the segment containing the finite-path
    // projection. This keeps the heading reference and CTE on the same path
    // branch while retaining the centred-chord noise suppression.
    const int reference_index = std::min(
        static_cast<int>(global_plan_.size()) - 1,
        segment + (projection.fraction >= 0.5 ? 1 : 0));
    return pathHeading(reference_index);
}

double MpcLocalPlanner::projectionCurvature(
        const PathProjection& projection) const {
    const int n = static_cast<int>(global_plan_.size());
    if (n < 3) return 0.0;
    const int segment =
        std::max(0, std::min(projection.segment_index, n - 2));
    const int left = std::max(1, std::min(segment, n - 2));
    const int right = std::max(1, std::min(segment + 1, n - 2));
    const double k0 = pathCurvature(left);
    const double k1 = pathCurvature(right);
    return (1.0 - projection.fraction) * k0 +
           projection.fraction * k1;
}

double MpcLocalPlanner::pathHeading(int idx) const {
    const int n = static_cast<int>(global_plan_.size());
    if (n == 0) return 0.0;
    if (n == 1)
        return tf2::getYaw(global_plan_.back().pose.orientation);
    idx = std::max(0, std::min(idx, n - 1));

    // Estimate the local tangent with a centred chord. The old implementation
    // looked only forward by 1.5m, so it used a future curve heading as the
    // current reference and made the vehicle turn early and cut the corner.
    const double half_window =
        std::max(0.1, 0.5 * params_.heading_lookahead);
    int begin = idx;
    int end = idx;
    double before = 0.0;
    double after = 0.0;
    while (begin > 0 && before < half_window) {
        const double dx = global_plan_[begin].pose.position.x -
                          global_plan_[begin-1].pose.position.x;
        const double dy = global_plan_[begin].pose.position.y -
                          global_plan_[begin-1].pose.position.y;
        before += std::hypot(dx, dy);
        --begin;
    }
    while (end < n - 1 && after < half_window) {
        const double dx = global_plan_[end+1].pose.position.x -
                          global_plan_[end].pose.position.x;
        const double dy = global_plan_[end+1].pose.position.y -
                          global_plan_[end].pose.position.y;
        after += std::hypot(dx, dy);
        ++end;
    }

    const double dx = global_plan_[end].pose.position.x -
                      global_plan_[begin].pose.position.x;
    const double dy = global_plan_[end].pose.position.y -
                      global_plan_[begin].pose.position.y;
    if (dx*dx + dy*dy < 1e-9)
        return tf2::getYaw(global_plan_[idx].pose.orientation);
    return std::atan2(dy, dx);
}

double MpcLocalPlanner::pathCurvature(int idx) const {
    if (idx < 1 || idx >= (int)global_plan_.size() - 1) return 0.0;

    double x0 = global_plan_[idx - 1].pose.position.x;
    double y0 = global_plan_[idx - 1].pose.position.y;
    double x1 = global_plan_[idx].pose.position.x;
    double y1 = global_plan_[idx].pose.position.y;
    double x2 = global_plan_[idx + 1].pose.position.x;
    double y2 = global_plan_[idx + 1].pose.position.y;

    double h1 = std::atan2(y1 - y0, x1 - x0);
    double h2 = std::atan2(y2 - y1, x2 - x1);
    double ds = 0.5 * (std::hypot(x1 - x0, y1 - y0) + std::hypot(x2 - x1, y2 - y1));
    if (ds < 1e-6) return 0.0;
    return normalizeAngle(h2 - h1) / ds;
}

// Terrain-adaptive speed: high-cost areas → automatic deceleration
double MpcLocalPlanner::desiredSpeed(int idx, double dist_to_goal,
                                      double x, double y) const {
    double v = params_.max_vel_x;

    // curvature deceleration
    double max_curv = 0.0;
    for (int i = idx; i < std::min(idx + 20, (int)global_plan_.size()); ++i)
        max_curv = std::max(max_curv, std::fabs(pathCurvature(i)));
    v *= std::max(1.0 - params_.curvature_decel * max_curv * 5.0, 0.3);

    // terrain cost deceleration: v *= (1 - terrain_decel * cost)
    if (terrain_map_.loaded) {
        float tc = queryTerrainCost(x, y);
        v *= std::max(1.0 - params_.terrain_decel * (double)tc, 0.2);
    }

    // goal approach deceleration
    if (dist_to_goal < params_.goal_decel_dist)
        v *= std::max(dist_to_goal / params_.goal_decel_dist, 0.1);

    return std::max(v, params_.min_vel_x);
}

// ==================== Dynamics model ====================

// Prediction with slip compensation: lateral slope induces yaw disturbance
MpcLocalPlanner::State MpcLocalPlanner::predictStep(
        const State& s, double v_cmd, double omega_cmd) const {
    State n = s;
    double dt = params_.prediction_dt;

    // Do not apply the vehicle's near-zero command threshold inside the MPC
    // prediction model. With the hard acceleration window, the first
    // reachable commands are only +/-acc_lim_theta*dt (currently 0.025rad/s),
    // which is smaller than angular_deadband (0.05rad/s). Clipping those
    // candidates to zero makes every initial prediction identical and traps
    // the optimizer at omega=0 forever, so it can never ramp into a turn.
    const double omega_eff = omega_cmd;

    double alpha_v = 1.0 - std::exp(-dt / effectiveTauV());
    double alpha_w = 1.0 - std::exp(-dt / effectiveTauOmega());

    n.v += (effectiveLinearTrackingGain() * v_cmd - n.v) * alpha_v;
    // The tracked vehicle's yaw response changes with terrain load. Use the
    // online identified steady-state gain instead of one fixed flat-ground
    // ratio so the prediction remains conservative on a side slope.
    n.omega +=
        (effectiveOmegaTrackingGain() * omega_eff - n.omega) * alpha_w;

    // Model the observed lateral component explicitly. The optional legacy
    // terrain-gradient term below only represents a yaw bias and remains
    // disabled by default.
    double omega_slip = querySlipOmega(n.x, n.y, n.theta, n.v);
    double omega_actual = n.omega + omega_slip;
    n.x += (n.v * std::cos(n.theta) -
            n.lateral_disturbance * std::sin(n.theta)) * dt;
    n.y += (n.v * std::sin(n.theta) +
            n.lateral_disturbance * std::cos(n.theta)) * dt;
    n.theta = normalizeAngle(n.theta + omega_actual * dt);

    return n;
}

// ==================== MPC core ====================

double MpcLocalPlanner::evaluateOmegaSequence(
        const State& state, double v_cmd, double v_reference,
        double omega_first, double omega_second,
        std::vector<State>* trajectory) const {
    double cost = 0.0;
    State s = state;
    costmap_2d::Costmap2D* cm = costmap_ros_->getCostmap();
    const int first_steps = std::max(
        1, std::min(params_.control_hold_steps, params_.prediction_steps));
    int projection_hint = projection_segment_cache_;

    for (int i = 0; i < params_.prediction_steps; ++i) {
        const double omega_cmd =
            i < first_steps ? omega_first : omega_second;
        s = predictStep(s, v_cmd, omega_cmd);
        if (trajectory) trajectory->push_back(s);

        const PathProjection projection =
            projectOntoPath(s.x, s.y, projection_hint);
        if (!projection.valid) {
            cost += 1e6;
            continue;
        }
        projection_hint = projection.segment_index;
        const double cte = projection.signed_cte;
        // A short horizon covers little distance after terrain deceleration.
        // Penalising only the path-tangent heading then lets the vehicle run
        // almost parallel to the path with a persistent lateral offset.
        // Convert CTE into a bounded recovery-heading reference. It vanishes
        // continuously when the vehicle returns to the path.
        const double raw_heading_correction = std::atan2(
            params_.cte_heading_gain * cte,
            std::fabs(s.v) + params_.cte_heading_soft_speed);
        const double cte_heading_correction = std::max(
            -params_.max_cte_heading_correction,
            std::min(params_.max_cte_heading_correction,
                     raw_heading_correction));
        const double disturbance_heading_correction =
            lateralDisturbanceHeading(s.v, s.lateral_disturbance);
        const double heading_correction = std::max(
            -params_.max_cte_heading_correction,
            std::min(params_.max_cte_heading_correction,
                     cte_heading_correction +
                         disturbance_heading_correction));
        double position_error = cte;
        double recovery_heading =
            normalizeAngle(projectionHeading(projection) +
                           heading_correction);

        // Once the nearest point is the final segment, the tangent and CTE
        // describe an infinite line rather than the finite path endpoint.
        // A vehicle that has passed the goal can therefore be told to keep
        // following that line and accelerate away. In terminal recovery,
        // optimise Euclidean distance and bearing to the actual endpoint.
        const double goal_dx =
            global_plan_.back().pose.position.x - s.x;
        const double goal_dy =
            global_plan_.back().pose.position.y - s.y;
        const double predicted_goal_distance =
            std::hypot(goal_dx, goal_dy);
        if (projection.segment_index >=
                static_cast<int>(global_plan_.size()) - 2 &&
            predicted_goal_distance < params_.terminal_alignment_distance) {
            position_error = predicted_goal_distance;
            recovery_heading = std::atan2(goal_dy, goal_dx);
        }
        double heading_err = normalizeAngle(s.theta - recovery_heading);
        const double curvature = projectionCurvature(projection);
        const double tracking_gain =
            std::max(effectiveOmegaTrackingGain(), 0.1);
        const double omega_reference = std::max(
            -params_.max_vel_theta,
            std::min(params_.max_vel_theta,
                     s.v * curvature / tracking_gain));
        const double omega_error = omega_cmd - omega_reference;
        const double speed_error = s.v - v_reference;

        cost += params_.weight_cte * position_error * position_error;
        cost += params_.weight_heading * heading_err * heading_err;
        cost += params_.weight_speed * speed_error * speed_error;
        cost += params_.weight_curvature_feedforward *
                omega_error * omega_error;

        // obstacle cost
        unsigned int mx, my;
        if (cm->worldToMap(s.x, s.y, mx, my)) {
            unsigned char c = cm->getCost(mx, my);
            if (c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                cost += 1e6;
            else if (c > 0)
                cost += params_.weight_obstacle * (c / 252.0);
        } else {
            cost += 1e6;
        }
    }

    // control cost
    const int second_steps = params_.prediction_steps - first_steps;
    cost += params_.weight_omega *
            (omega_first * omega_first * first_steps +
             omega_second * omega_second * second_steps);
    cost += params_.weight_omega_rate *
            (omega_first - prev_omega_cmd_) *
            (omega_first - prev_omega_cmd_);
    cost += params_.weight_v_rate *
            (v_cmd - prev_v_cmd_) * (v_cmd - prev_v_cmd_);
    if (second_steps > 0) {
        cost += params_.weight_omega_rate *
                (omega_second - omega_first) *
                (omega_second - omega_first);
    }

    return cost;
}

bool MpcLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_ || source_plan_.empty()) return false;

    const ros::WallTime cycle_start = ros::WallTime::now();
    last_control_cycle_interval_ = last_control_cycle_start_.isZero()
        ? 0.0 : (cycle_start - last_control_cycle_start_).toSec();
    last_control_cycle_start_ = cycle_start;
    if (last_control_cycle_interval_ >
        params_.max_control_cycle_interval) {
        ROS_WARN_THROTTLE(
            1.0, "MPC watchdog: control cycle interval %.3fs exceeds %.3fs",
            last_control_cycle_interval_,
            params_.max_control_cycle_interval);
        issueSafeStop(cmd_vel, "control cycle interval exceeded");
        return true;
    }

    if (!updatePlanInCostmapFrame()) return false;

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose)) {
        ROS_WARN_THROTTLE(1.0, "MPC: cannot get robot pose");
        return false;
    }
    double rx = robot_pose.pose.position.x;
    double ry = robot_pose.pose.position.y;
    double ryaw = tf2::getYaw(robot_pose.pose.orientation);

    double act_v, act_lateral_v, act_omega;
    double position_covariance, yaw_covariance;
    ros::Time act_measurement_stamp;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        act_v = current_v_;
        act_lateral_v = current_lateral_v_;
        act_omega = current_omega_;
        act_measurement_stamp = current_state_measurement_stamp_;
        position_covariance = current_position_covariance_;
        yaw_covariance = current_yaw_covariance_;
    }
    double state_age = 0.0;
    double tf_age = 0.0;
    if (!controlInputsHealthy(robot_pose, act_measurement_stamp,
                              position_covariance, yaw_covariance,
                              state_age, tf_age)) {
        issueSafeStop(cmd_vel, "state/TF/localization quality invalid");
        return true;
    }

    double gx = global_plan_.back().pose.position.x;
    double gy = global_plan_.back().pose.position.y;
    double dist_goal = std::hypot(gx - rx, gy - ry);
    if (dist_goal < params_.goal_tolerance_xy) {
        double gyaw = tf2::getYaw(global_plan_.back().pose.orientation);
        if (std::fabs(normalizeAngle(ryaw - gyaw)) < params_.goal_tolerance_yaw) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            prev_v_cmd_ = 0.0;
            prev_omega_cmd_ = 0.0;
            recordIssuedCommand(0.0, 0.0);
            one_step_cte_prediction_valid_ = false;
            goal_reached_ = true;
            return true;
        }
    }

    // Finish position tracking before switching to final-yaw alignment.
    // Starting the in-place rotation at a fixed 1.0 m distance made a relaxed
    // Dubins arrival oscillate between two incompatible objectives: point the
    // vehicle at the commanded final yaw, then turn back toward the still
    // unreached goal position. Enter rotation only after XY is accepted.
    if (dist_goal < params_.goal_tolerance_xy) {
        double gyaw = tf2::getYaw(global_plan_.back().pose.orientation);
        double gerr = normalizeAngle(ryaw - gyaw);
        if (std::fabs(gerr) > params_.goal_tolerance_yaw) {
            const double target_omega = std::min(
                std::max(-0.5 * gerr, -params_.max_vel_theta),
                params_.max_vel_theta);
            const double domega_max =
                params_.acc_lim_theta * params_.prediction_dt;
            const double omega = std::min(
                std::max(target_omega, prev_omega_cmd_ - domega_max),
                prev_omega_cmd_ + domega_max);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = omega;
            prev_v_cmd_ = 0.0;
            prev_omega_cmd_ = omega;
            recordIssuedCommand(0.0, omega);
            one_step_cte_prediction_valid_ = false;
            return true;
        }
    }

    // The odometry response belongs to commands issued before this cycle.
    // Update the model before evaluating the next MPC command sequence.
    updateResponseModel(act_v, act_omega, act_measurement_stamp);

    const PathProjection current_projection = projectOntoPath(
        rx, ry, projection_segment_cache_);
    if (!current_projection.valid) {
        ROS_WARN_THROTTLE(1.0, "MPC: cannot project robot pose onto plan");
        return false;
    }
    projection_segment_cache_ = current_projection.segment_index;
    const int closest_segment = current_projection.segment_index;
    const int closest = std::min(
        static_cast<int>(global_plan_.size()) - 1,
        closest_segment + (current_projection.fraction >= 0.5 ? 1 : 0));

    // Near the finite path endpoint, a tracked vehicle cannot recover from
    // an overshoot using forward motion alone: its minimum turning radius can
    // carry it farther away while the path projection already reports 100%.
    // Stop and align with the actual goal bearing first, then let MPC perform
    // the final low-speed approach. This state is deliberately entered only
    // near the goal and outside the accepted XY tolerance.
    if (closest_segment >= static_cast<int>(global_plan_.size()) - 2 &&
        dist_goal < params_.terminal_alignment_distance &&
        dist_goal >= params_.goal_tolerance_xy) {
        const double goal_bearing = std::atan2(gy - ry, gx - rx);
        const double bearing_error =
            normalizeAngle(ryaw - goal_bearing);
        if (std::fabs(bearing_error) >
            params_.terminal_bearing_tolerance) {
            const double target_omega = std::max(
                -params_.max_vel_theta,
                std::min(params_.max_vel_theta,
                         -params_.terminal_bearing_gain * bearing_error));
            const double domega_max =
                params_.curve_acc_lim_theta * params_.prediction_dt;
            const double omega = std::max(
                prev_omega_cmd_ - domega_max,
                std::min(prev_omega_cmd_ + domega_max, target_omega));
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = omega;
            prev_v_cmd_ = 0.0;
            prev_omega_cmd_ = omega;
            recordIssuedCommand(0.0, omega);
            one_step_cte_prediction_valid_ = false;
            return true;
        }
    }

    double v_reference = desiredSpeed(closest, dist_goal, rx, ry);
    if (closest_segment >= static_cast<int>(global_plan_.size()) - 2 &&
        dist_goal < params_.terminal_alignment_distance &&
        dist_goal >= params_.goal_tolerance_xy) {
        v_reference = std::min(v_reference, params_.terminal_recovery_speed);
    }

    // Slow down when heading deviates from path
    const double signed_heading_err =
        normalizeAngle(ryaw - projectionHeading(current_projection));
    const double abs_heading_err = std::fabs(signed_heading_err);
    const double signed_cte = current_projection.signed_cte;

    // Estimate whether the lateral error is actually recovering. Body yaw
    // alone is not sufficient on a side slope: the chassis can point at the
    // recovery heading while its velocity still drifts laterally downhill.
    // Filter the signed CTE derivative, then convert it to d|CTE|/dt. A
    // positive growth rate always means that tracking is getting worse,
    // independent of which side of the path the vehicle is on.
    const ros::Time cte_update_time = ros::Time::now();
    double cte_update_dt = 0.0;
    if (!last_cte_update_time_.isZero()) {
        cte_update_dt = (cte_update_time - last_cte_update_time_).toSec();
        if (cte_update_dt > 0.01 && cte_update_dt < 0.5) {
            const double raw_cte_rate =
                (signed_cte - previous_signed_cte_) / cte_update_dt;
            filtered_cte_rate_ += params_.turn_assist_cte_rate_alpha *
                (raw_cte_rate - filtered_cte_rate_);
        } else {
            filtered_cte_rate_ = 0.0;
            cte_update_dt = 0.0;
        }
    }
    previous_signed_cte_ = signed_cte;
    last_cte_update_time_ = cte_update_time;

    // Close the disturbance-observer loop with the prediction made one
    // controller cycle ago. This is a true model-prediction innovation, not a
    // derivative reconstructed from the current CTE and heading alone.
    const ros::Time observer_now = ros::Time::now();
    const double one_step_prediction_age =
        one_step_prediction_created_time_.isZero()
            ? 0.0
            : (observer_now - one_step_prediction_created_time_).toSec();
    const bool one_step_residual_valid =
        one_step_cte_prediction_valid_ &&
        one_step_prediction_age >= 0.04 &&
        one_step_prediction_age <= 0.30 &&
        std::fabs(act_v) >= params_.lateral_disturbance_min_speed;
    const double compared_predicted_cte = one_step_predicted_cte_;
    one_step_prediction_residual_ = one_step_residual_valid
        ? signed_cte - compared_predicted_cte : 0.0;
    updateLateralDisturbanceEstimate(
        signed_heading_err, one_step_prediction_residual_,
        one_step_prediction_age, one_step_residual_valid);
    State state = {rx, ry, ryaw, act_v, act_omega,
                   estimated_lateral_disturbance_};

    const double cte_growth_rate =
        std::fabs(signed_cte) > 1e-6
            ? std::copysign(1.0, signed_cte) * filtered_cte_rate_
            : 0.0;

    const double current_cte_heading_raw = std::atan2(
        params_.cte_heading_gain * signed_cte,
        std::fabs(act_v) + params_.cte_heading_soft_speed);
    const double current_cte_heading_correction = std::max(
        -params_.max_cte_heading_correction,
        std::min(params_.max_cte_heading_correction,
                 current_cte_heading_raw));
    const double current_disturbance_heading_correction =
        lateralDisturbanceHeading(act_v,
                                  estimated_lateral_disturbance_);
    if (abs_heading_err > M_PI / 6.0) {
        v_reference *= std::max(
            1.0 - (abs_heading_err - M_PI / 6.0) / (M_PI / 3.0), 0.0);
    }
    // Linear-speed candidates share the same hard acceleration-reachable
    // window as the yaw candidates. The desired-speed rule now supplies the
    // reference; MPC is free to choose a nearby feasible speed when slowing
    // down improves CTE, obstacle clearance, or terminal behaviour.
    double dv_max = params_.acc_lim_x * params_.prediction_dt;
    double v_min_reachable = std::max(
        -params_.max_vel_x_backwards, prev_v_cmd_ - dv_max);
    const double v_max_reachable = std::min(
        params_.max_vel_x, prev_v_cmd_ + dv_max);
    // Normal navigation is forward-only. Retain the configured reverse limit
    // for a future explicit reversing mode, but do not let a symmetric start
    // window make MPC select a reverse command for a forward reference.
    if (v_reference >= 0.0) {
        v_min_reachable = std::max(0.0, v_min_reachable);
    }
    v_reference = std::max(
        v_min_reachable, std::min(v_max_reachable, v_reference));

    double upcoming_max_curv = 0.0;
    for (int i = closest;
         i < std::min(closest + 20, static_cast<int>(global_plan_.size()));
         ++i) {
        upcoming_max_curv =
            std::max(upcoming_max_curv, std::fabs(pathCurvature(i)));
    }

    // Keep conservative angular acceleration on straight segments to reject
    // localization jitter, but allow a faster build-up when a real curve is
    // present in the look-ahead path.
    const double active_acc_lim_theta =
        upcoming_max_curv >= params_.curve_threshold
            ? params_.curve_acc_lim_theta
            : params_.acc_lim_theta;

    // Scan only dynamically reachable angular-velocity candidates.  The
    // previous implementation sampled the complete [-max, +max] range every
    // control cycle, so acc_lim_theta was loaded but never enforced.  At
    // 10 Hz that allowed a +0.35 -> -0.35 rad/s command reversal in one frame,
    // which amplified localization noise into repeated steering corrections.
    const double domega_max =
        active_acc_lim_theta * params_.prediction_dt;
    const double omega_min = std::max(
        -params_.max_vel_theta, prev_omega_cmd_ - domega_max);
    const double omega_max = std::min(
        params_.max_vel_theta, prev_omega_cmd_ + domega_max);

    const double current_curvature =
        projectionCurvature(current_projection);
    double best_omega = 0.0;
    double best_second_omega = 0.0;
    double best_v = v_reference;
    double best_omega_feedforward = 0.0;
    double best_cost = std::numeric_limits<double>::max();
    std::vector<State> best_traj;
    const ros::WallTime solver_start = ros::WallTime::now();
    last_solver_timed_out_ = false;
    last_evaluated_candidates_ = 0;

    const int Nv = std::max(1, params_.num_v_samples);
    const int N = std::max(1, params_.num_omega_samples);
    for (int iv = 0; iv < Nv && !last_solver_timed_out_; ++iv) {
        const double v_ratio =
            Nv > 1 ? static_cast<double>(iv) / (Nv - 1) : 0.0;
        const double v_candidate = Nv > 1
            ? v_min_reachable +
                (v_max_reachable - v_min_reachable) * v_ratio
            : v_reference;
        const double omega_feedforward_candidate = std::max(
            -params_.max_vel_theta,
            std::min(params_.max_vel_theta,
                     v_candidate * current_curvature /
                     std::max(effectiveOmegaTrackingGain(), 0.1)));

        // Optimise a correction around curvature feedforward. Sampling the
        // final reachable command preserves the old hard constraints, while
        // the variable itself is explicitly Δω = ω_cmd - ω_ff.
        const double delta_omega_min =
            omega_min - omega_feedforward_candidate;
        const double delta_omega_max =
            omega_max - omega_feedforward_candidate;
        for (int i = 0; i < N && !last_solver_timed_out_; ++i) {
            const double ratio =
                N > 1 ? static_cast<double>(i) / (N - 1) : 0.0;
            const double delta_omega =
                delta_omega_min +
                (delta_omega_max - delta_omega_min) * ratio;
            const double omega_first =
                omega_feedforward_candidate + delta_omega;

            // A constant angular command over the full prediction horizon
            // cannot represent "steer toward the path, then straighten".
            const double second_delta =
                active_acc_lim_theta * params_.prediction_dt;
            const double second_min = std::max(
                -params_.max_vel_theta, omega_first - second_delta);
            const double second_max = std::min(
                params_.max_vel_theta, omega_first + second_delta);
            const int N2 = std::max(1, params_.num_second_omega_samples);

            for (int j = 0; j < N2; ++j) {
                if ((ros::WallTime::now() - solver_start).toSec() >=
                    params_.solver_time_limit) {
                    last_solver_timed_out_ = true;
                    break;
                }
                const double ratio2 =
                    N2 > 1 ? static_cast<double>(j) / (N2 - 1) : 0.0;
                const double omega_second = N2 > 1
                    ? second_min + (second_max - second_min) * ratio2
                    : omega_first;

                const double cost = evaluateOmegaSequence(
                    state, v_candidate, v_reference,
                    omega_first, omega_second, nullptr);
                ++last_evaluated_candidates_;

                if (cost < best_cost) {
                    best_cost = cost;
                    best_v = v_candidate;
                    best_omega = omega_first;
                    best_second_omega = omega_second;
                    best_omega_feedforward =
                        omega_feedforward_candidate;
                }
            }
        }
    }
    last_solver_duration_ =
        (ros::WallTime::now() - solver_start).toSec();
    if (last_solver_duration_ > params_.solver_time_limit) {
        last_solver_timed_out_ = true;
    }

    if (last_solver_timed_out_) {
        ++consecutive_solver_timeouts_;
        // Candidate enumeration is ordered, so a partially explored optimum
        // can be directionally biased. Reuse the complete command accepted on
        // the previous cycle instead of applying a partial search result.
        best_v = std::max(-params_.max_vel_x_backwards,
                          std::min(params_.max_vel_x, prev_v_cmd_));
        best_omega = std::max(-params_.max_vel_theta,
                              std::min(params_.max_vel_theta,
                                       prev_omega_cmd_));
        best_second_omega = best_omega;
        best_omega_feedforward = std::max(
            -params_.max_vel_theta,
            std::min(params_.max_vel_theta,
                     best_v * current_curvature /
                         std::max(effectiveOmegaTrackingGain(), 0.1)));
        best_cost = std::numeric_limits<double>::quiet_NaN();
        best_traj.clear();
        ROS_WARN_THROTTLE(
            1.0,
            "MPC solver timeout %.1fms candidates=%d; reusing previous "
            "command (count=%d)",
            last_solver_duration_ * 1000.0, last_evaluated_candidates_,
            consecutive_solver_timeouts_);
        if (consecutive_solver_timeouts_ >=
            params_.max_consecutive_solver_timeouts) {
            issueSafeStop(cmd_vel, "repeated solver timeouts");
            return true;
        }
    } else {
        consecutive_solver_timeouts_ = 0;
        (void)evaluateOmegaSequence(
            state, best_v, v_reference, best_omega,
            best_second_omega, &best_traj);
    }
    double v_cmd = best_v;

    // On a side slope the measured yaw response can fall below 40% of the
    // command. The finite-horizon optimum may then remain too conservative
    // even though it points in the correct direction. Convert the remaining
    // recovery-heading error into the minimum command required to close it
    // within a configurable time, compensated by the identified yaw gain.
    // The result is still clipped to this cycle's acceleration-reachable
    // window, so the assistance cannot introduce an angular command step.
    const double base_recovery_heading = normalizeAngle(
        projectionHeading(current_projection) +
        current_cte_heading_correction +
        current_disturbance_heading_correction);
    const double base_recovery_heading_error =
        normalizeAngle(ryaw - base_recovery_heading);

    const bool cte_above_entry =
        std::fabs(signed_cte) >= params_.turn_assist_cte_threshold;
    const bool cte_below_release =
        std::fabs(signed_cte) <=
            params_.turn_assist_release_cte_threshold;
    const bool cte_worsening =
        cte_growth_rate >= params_.turn_assist_cte_rate_threshold;
    const bool recovery_heading_unmet =
        std::fabs(base_recovery_heading_error) >=
            params_.turn_assist_heading_threshold;

    if (!params_.enable_turn_authority_assist || cte_below_release) {
        turn_assist_latched_ = false;
        turn_assist_improving_time_ = 0.0;
        held_rate_heading_correction_ = 0.0;
    } else {
        if (!turn_assist_latched_ && cte_above_entry &&
            (recovery_heading_unmet || cte_worsening)) {
            turn_assist_latched_ = true;
            turn_assist_improving_time_ = 0.0;
        }

        if (turn_assist_latched_) {
            if (cte_growth_rate <=
                -params_.turn_assist_cte_rate_threshold) {
                turn_assist_improving_time_ += cte_update_dt;
                if (turn_assist_improving_time_ >=
                    params_.turn_assist_release_time) {
                    turn_assist_latched_ = false;
                    turn_assist_improving_time_ = 0.0;
                    held_rate_heading_correction_ = 0.0;
                }
            } else {
                turn_assist_improving_time_ = 0.0;
            }
        }
    }

    // While |CTE| is still growing, demand a little more recovery heading
    // in the same direction as the geometric CTE correction. This term is
    // bounded separately and disappears smoothly as the error stops growing,
    // so it compensates side-slope drift without permanently biasing turns.
    double instantaneous_rate_heading = 0.0;
    if (turn_assist_latched_ && cte_worsening) {
        const double rate_heading_magnitude = std::min(
            params_.turn_assist_max_rate_heading,
            std::atan2(params_.turn_assist_cte_rate_gain * cte_growth_rate,
                       std::fabs(act_v) +
                           params_.cte_heading_soft_speed));
        instantaneous_rate_heading =
            std::copysign(rate_heading_magnitude, signed_cte);

        // Build the disturbance-heading bias quickly, but do not remove it
        // merely because one noisy derivative sample falls below threshold.
        // A sign change starts a fresh correction; otherwise the held bias
        // only grows toward a stronger observation. It is released by the
        // same 0.08 m / one-second-improvement hysteresis as the assist latch.
        if (held_rate_heading_correction_ *
                instantaneous_rate_heading < 0.0) {
            held_rate_heading_correction_ = 0.0;
        }
        if (std::fabs(instantaneous_rate_heading) >
            std::fabs(held_rate_heading_correction_)) {
            held_rate_heading_correction_ +=
                params_.turn_assist_rate_heading_alpha *
                (instantaneous_rate_heading -
                 held_rate_heading_correction_);
        }
    }
    const double rate_heading_correction =
        turn_assist_latched_ ? held_rate_heading_correction_ : 0.0;
    const double assisted_cte_heading_correction = std::max(
        -params_.max_cte_heading_correction,
        std::min(params_.max_cte_heading_correction,
                 current_cte_heading_correction +
                     rate_heading_correction +
                     current_disturbance_heading_correction));
    const double recovery_heading = normalizeAngle(
        projectionHeading(current_projection) +
        assisted_cte_heading_correction);
    const double recovery_heading_error =
        normalizeAngle(ryaw - recovery_heading);
    double turn_assist_omega = 0.0;
    bool turn_assist_active = false;
    if (turn_assist_latched_ && !last_solver_timed_out_) {
        const double required_omega =
            -recovery_heading_error /
            (params_.turn_assist_time_constant *
             std::max(effectiveOmegaTrackingGain(), 0.1));
        turn_assist_omega = std::max(
            omega_min, std::min(omega_max,
                std::max(-params_.max_vel_theta,
                    std::min(params_.max_vel_theta, required_omega))));
        if (std::fabs(turn_assist_omega) > std::fabs(best_omega) &&
            turn_assist_omega * recovery_heading_error < 0.0) {
            best_omega = turn_assist_omega;
            turn_assist_active = true;
        }
    }

    cmd_vel.linear.x = v_cmd;
    cmd_vel.angular.z = best_omega;
    // current_omega_ is the response to commands issued before this cycle.
    // Compare it with the previous command, not the newly selected command,
    // so repeated turn logs can identify the real steering gain.
    const double previous_omega_cmd = prev_omega_cmd_;
    const double observed_omega_gain =
        std::fabs(aligned_response_omega_cmd_) >=
                params_.omega_gain_min_command &&
        aligned_response_omega_cmd_ * act_omega > 0.0
            ? std::fabs(act_omega / aligned_response_omega_cmd_)
            : std::numeric_limits<double>::quiet_NaN();
    const double omega_feedforward = best_omega_feedforward;
    const double delta_v_mpc = v_cmd - v_reference;
    const double delta_omega_mpc = best_omega - omega_feedforward;

    // Publish a read-only snapshot of the quantities that explain the MPC
    // decision.  The one-step prediction is evaluated only for diagnostics;
    // it is never used to alter the command selected above.
    const State one_step_prediction =
        predictStep(state, v_cmd, best_omega);
    const PathProjection one_step_projection = projectOntoPath(
        one_step_prediction.x, one_step_prediction.y,
        current_projection.segment_index);
    one_step_cte_prediction_valid_ = one_step_projection.valid;
    one_step_predicted_cte_ = one_step_projection.valid
        ? one_step_projection.signed_cte : signed_cte;
    one_step_prediction_created_time_ = ros::Time::now();
    if (diagnostics_pub_.getNumSubscribers() > 0) {
        std_msgs::Float64MultiArray diagnostic;
        diagnostic.data = {
            ros::Time::now().toSec(),                 // 0 stamp_s
            signed_cte,                               // 1 signed_cte_m
            projectionHeading(current_projection),   // 2 path_heading_rad
            signed_heading_err,                       // 3 heading_error_rad
            current_curvature,                       // 4 curvature_1pm
            omega_feedforward,                       // 5 omega_ff_radps
            v_cmd,                                    // 6 linear_cmd_mps
            best_omega,                               // 7 omega_cmd_radps
            previous_omega_cmd,                       // 8 previous_omega_cmd_radps
            act_v,                                    // 9 measured_forward_mps
            act_lateral_v,                            // 10 odom_lateral_mps
            act_omega,                                // 11 measured_omega_radps
            observed_omega_gain,                      // 12 observed_omega_gain
            effectiveOmegaTrackingGain(),             // 13 model_omega_gain
            filtered_cte_rate_,                       // 14 filtered_cte_rate_mps
            cte_growth_rate,                          // 15 abs_cte_growth_rate_mps
            observed_lateral_disturbance_,            // 16 lateral_residual_mps
            estimated_lateral_disturbance_,           // 17 lateral_estimate_mps
            current_cte_heading_correction,           // 18 cte_heading_ref_rad
            current_disturbance_heading_correction,   // 19 disturbance_heading_ref_rad
            rate_heading_correction,                  // 20 rate_heading_ref_rad
            turn_assist_omega,                        // 21 assist_omega_radps
            turn_assist_latched_ ? 1.0 : 0.0,         // 22 assist_latched
            turn_assist_active ? 1.0 : 0.0,           // 23 assist_active
            best_cost,                                // 24 selected_cost
            best_second_omega,                        // 25 future_omega_radps
            one_step_prediction.v,                    // 26 predicted_v_next_mps
            one_step_prediction.omega,                // 27 predicted_omega_next_radps
            one_step_prediction.x,                    // 28 predicted_x_next_m
            one_step_prediction.y,                    // 29 predicted_y_next_m
            one_step_prediction.theta,                // 30 predicted_yaw_next_rad
            omega_min,                                // 31 reachable_omega_min_radps
            omega_max,                                // 32 reachable_omega_max_radps
            v_reference,                              // 33 linear_reference_mps
            delta_v_mpc,                              // 34 delta_v_mpc_mps
            delta_omega_mpc,                          // 35 delta_omega_mpc_radps
            effectiveLinearTrackingGain(),            // 36 model_linear_gain
            effectiveTauV(),                          // 37 model_tau_v_s
            effectiveTauOmega(),                      // 38 model_tau_omega_s
            v_min_reachable,                          // 39 reachable_v_min_mps
            v_max_reachable,                          // 40 reachable_v_max_mps
            compared_predicted_cte,                   // 41 predicted_cte_at_measurement_m
            one_step_prediction_residual_,            // 42 cte_prediction_residual_m
            act_measurement_stamp.toSec(),             // 43 response_measurement_stamp_s
            aligned_response_v_cmd_,                  // 44 aligned_linear_cmd_mps
            aligned_response_omega_cmd_,              // 45 aligned_omega_cmd_radps
            response_command_alignment_age_,          // 46 command_alignment_age_s
            one_step_residual_valid ? 1.0 : 0.0,       // 47 cte_prediction_residual_valid
            state_age,                                 // 48 state_age_s
            tf_age,                                    // 49 tf_age_s
            last_control_cycle_interval_,              // 50 control_cycle_interval_s
            last_solver_duration_ * 1000.0,            // 51 solver_duration_ms
            last_solver_timed_out_ ? 1.0 : 0.0,        // 52 solver_timed_out
            static_cast<double>(last_evaluated_candidates_), // 53 evaluated_candidates
            static_cast<double>(projection_segment_cache_)   // 54 projection_cache_segment
        };
        diagnostics_pub_.publish(diagnostic);
    }

    // Publish the complete horizon produced by the command that will actually
    // be sent. This remains read-only: it is intended for checking whether
    // the adaptive model predicts the measured response, not for closing a
    // second feedback loop through the evaluator.
    if (prediction_diagnostics_pub_.getNumSubscribers() > 0) {
        std_msgs::Float64MultiArray prediction;
        prediction.layout.dim.resize(2);
        prediction.layout.dim[0].label = "points";
        prediction.layout.dim[0].size = params_.prediction_steps;
        prediction.layout.dim[0].stride = params_.prediction_steps * 8;
        prediction.layout.dim[1].label =
            "t,x,y,yaw,v,omega,cte,heading_error";
        prediction.layout.dim[1].size = 8;
        prediction.layout.dim[1].stride = 8;
        prediction.data.reserve(params_.prediction_steps * 8);

        State predicted = state;
        int projection_hint = closest_segment;
        for (int step = 0; step < params_.prediction_steps; ++step) {
            const double predicted_omega_command =
                step < params_.control_hold_steps
                    ? best_omega : best_second_omega;
            predicted = predictStep(predicted, v_cmd,
                                    predicted_omega_command);
            const PathProjection predicted_projection = projectOntoPath(
                predicted.x, predicted.y, projection_hint);
            if (predicted_projection.valid) {
                projection_hint = predicted_projection.segment_index;
            }
            const double predicted_cte = predicted_projection.valid
                ? predicted_projection.signed_cte : 0.0;
            const double predicted_heading_error = predicted_projection.valid
                ? normalizeAngle(
                      predicted.theta -
                      projectionHeading(predicted_projection))
                : 0.0;
            prediction.data.insert(prediction.data.end(), {
                (step + 1) * params_.prediction_dt,
                predicted.x, predicted.y, predicted.theta,
                predicted.v, predicted.omega,
                predicted_cte, predicted_heading_error});
        }
        prediction_diagnostics_pub_.publish(prediction);
    }

    ROS_INFO_THROTTLE(
        0.5,
        "MPC debug: closest=%d/%zu segment=%d t=%.2f cte=%+.3f "
        "path_distance=%.3f heading_err=%+.1fdeg "
        "cte_heading_ref=%+.1fdeg lateral_odom=%+.3f "
        "lateral_residual=%+.3f lateral_est=%+.3f "
        "lateral_ref=%+.1fdeg "
        "curvature=%+.3f omega_ff=%+.3f "
        "acc_theta=%.2f v_ref=%.3f v_cmd=%.3f dv=%+.3f "
        "omega_cmd=%+.3f omega_future=%+.3f domega=%+.3f "
        "omega_prev=%+.3f omega_actual=%+.3f gain_obs=%+.2f "
        "gain_model=(v:%.2f,w:%.2f) tau=(v:%.2f,w:%.2f) "
        "gain_stable=%d cte_rate=%+.3f growth=%+.3f "
        "rate_ref=%+.1fdeg assist_latched=%s assist=%s "
        "assist_omega=%+.3f solve=%.1fms/%dcand timeout=%s "
        "window=[%+.3f,%+.3f]",
        closest, global_plan_.size(), closest_segment,
        current_projection.fraction, signed_cte,
        current_projection.distance,
        signed_heading_err * 180.0 / M_PI,
        current_cte_heading_correction * 180.0 / M_PI,
        act_lateral_v, observed_lateral_disturbance_,
        estimated_lateral_disturbance_,
        current_disturbance_heading_correction * 180.0 / M_PI,
        current_curvature, omega_feedforward,
        active_acc_lim_theta, v_reference, v_cmd, delta_v_mpc,
        best_omega, best_second_omega, delta_omega_mpc,
        previous_omega_cmd, act_omega, observed_omega_gain,
        effectiveLinearTrackingGain(), effectiveOmegaTrackingGain(),
        effectiveTauV(), effectiveTauOmega(), gain_stable_cycles_,
        filtered_cte_rate_, cte_growth_rate,
        rate_heading_correction * 180.0 / M_PI,
        turn_assist_latched_ ? "on" : "off",
        turn_assist_active ? "on" : "off", turn_assist_omega,
        last_solver_duration_ * 1000.0, last_evaluated_candidates_,
        last_solver_timed_out_ ? "yes" : "no",
        omega_min, omega_max);

    prev_v_cmd_ = v_cmd;
    prev_omega_cmd_ = best_omega;
    recordIssuedCommand(v_cmd, best_omega);

    if (local_plan_pub_.getNumSubscribers() > 0 && !best_traj.empty()) {
        nav_msgs::Path path_msg;
        path_msg.header = robot_pose.header;
        for (const auto& s : best_traj) {
            geometry_msgs::PoseStamped p;
            p.header = robot_pose.header;
            p.pose.position.x = s.x;
            p.pose.position.y = s.y;
            p.pose.orientation.z = std::sin(s.theta / 2.0);
            p.pose.orientation.w = std::cos(s.theta / 2.0);
            path_msg.poses.push_back(p);
        }
        local_plan_pub_.publish(path_msg);
    }

    return true;
}

}  // namespace mpc_local_planner
