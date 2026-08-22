#include <navigation/my_Global_path_planning.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <numeric>
#include <sstream>
#include <utility>

PLUGINLIB_EXPORT_CLASS(my_global_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)
PLUGINLIB_EXPORT_CLASS(my_global_planner::BasicHybridAStarPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner {

namespace {

inline double mod2pi(double a) {
  a = fmod(a, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a;
}

inline double angleDiff(double a, double b) {
  double d = a - b;
  while (d > M_PI) d -= 2.0 * M_PI;
  while (d < -M_PI) d += 2.0 * M_PI;
  return d;
}

// ==================== Dubins curves ====================
struct DubinsWord { double t, p, q; bool valid; };

DubinsWord dubinsLSL(double a, double b, double d) {
  DubinsWord w;
  double p2 = 2 + d*d - 2*cos(a-b) + 2*d*(sin(a)-sin(b));
  if (p2 < 0) { w.valid = false; return w; }
  double tmp = atan2(cos(b)-cos(a), d+sin(a)-sin(b));
  w.t = mod2pi(-a + tmp);
  w.p = sqrt(p2);
  w.q = mod2pi(b - tmp);
  w.valid = true;
  return w;
}

DubinsWord dubinsRSR(double a, double b, double d) {
  DubinsWord w;
  double p2 = 2 + d*d - 2*cos(a-b) - 2*d*(sin(a)-sin(b));
  if (p2 < 0) { w.valid = false; return w; }
  double tmp = atan2(cos(a)-cos(b), d-sin(a)+sin(b));
  w.t = mod2pi(a - tmp);
  w.p = sqrt(p2);
  w.q = mod2pi(-b + tmp);
  w.valid = true;
  return w;
}

DubinsWord dubinsLSR(double a, double b, double d) {
  DubinsWord w;
  double p2 = -2 + d*d + 2*cos(a-b) + 2*d*(sin(a)+sin(b));
  if (p2 < 0) { w.valid = false; return w; }
  w.p = sqrt(p2);
  double tmp = atan2(-cos(a)-cos(b), d+sin(a)+sin(b)) - atan2(-2.0, w.p);
  w.t = mod2pi(-a + tmp);
  w.q = mod2pi(-mod2pi(b) + tmp);
  w.valid = true;
  return w;
}

DubinsWord dubinsRSL(double a, double b, double d) {
  DubinsWord w;
  double p2 = -2 + d*d + 2*cos(a-b) - 2*d*(sin(a)+sin(b));
  if (p2 < 0) { w.valid = false; return w; }
  w.p = sqrt(p2);
  double tmp = atan2(cos(a)+cos(b), d-sin(a)-sin(b)) - atan2(2.0, w.p);
  w.t = mod2pi(a - tmp);
  w.q = mod2pi(mod2pi(b) - tmp);
  w.valid = true;
  return w;
}

DubinsWord dubinsRLR(double a, double b, double d) {
  DubinsWord w;
  double tmp = (6.0 - d*d + 2*cos(a-b) + 2*d*(sin(a)-sin(b))) / 8.0;
  if (fabs(tmp) > 1.0) { w.valid = false; return w; }
  w.p = mod2pi(2*M_PI - acos(tmp));
  w.t = mod2pi(a - atan2(cos(a)-cos(b), d-sin(a)+sin(b)) + w.p/2.0);
  w.q = mod2pi(a - b - w.t + w.p);
  w.valid = true;
  return w;
}

DubinsWord dubinsLRL(double a, double b, double d) {
  DubinsWord w;
  double tmp = (6.0 - d*d + 2*cos(a-b) + 2*d*(-sin(a)+sin(b))) / 8.0;
  if (fabs(tmp) > 1.0) { w.valid = false; return w; }
  w.p = mod2pi(2*M_PI - acos(tmp));
  w.t = mod2pi(-a + atan2(-cos(a)+cos(b), d+sin(a)-sin(b)) + w.p/2.0);
  w.q = mod2pi(mod2pi(b) - a - w.t + w.p);
  w.valid = true;
  return w;
}

enum SegType { L_SEG = 0, S_SEG = 1, R_SEG = 2 };
static const int DUBINS_SEGS[6][3] = {
  {L_SEG, S_SEG, L_SEG}, {R_SEG, S_SEG, R_SEG},
  {L_SEG, S_SEG, R_SEG}, {R_SEG, S_SEG, L_SEG},
  {R_SEG, L_SEG, R_SEG}, {L_SEG, R_SEG, L_SEG},
};

struct DubinsPath {
  double segs[3]; int types[3]; double total_length, radius; bool valid;
};

std::vector<DubinsPath> dubinsCandidates(double x0, double y0, double t0,
                                         double x1, double y1, double t1,
                                         double r) {
  double dx = x1 - x0, dy = y1 - y0;
  double D = sqrt(dx*dx + dy*dy);
  double d = D / r;
  double theta = atan2(dy, dx);
  double alpha = mod2pi(t0 - theta);
  double beta  = mod2pi(t1 - theta);

  DubinsWord words[6];
  words[0] = dubinsLSL(alpha, beta, d);
  words[1] = dubinsRSR(alpha, beta, d);
  words[2] = dubinsLSR(alpha, beta, d);
  words[3] = dubinsRSL(alpha, beta, d);
  words[4] = dubinsRLR(alpha, beta, d);
  words[5] = dubinsLRL(alpha, beta, d);

  std::vector<DubinsPath> candidates;
  candidates.reserve(6);
  for (int i = 0; i < 6; ++i) {
    if (!words[i].valid) continue;
    DubinsPath candidate;
    candidate.segs[0] = words[i].t;
    candidate.segs[1] = words[i].p;
    candidate.segs[2] = words[i].q;
    candidate.types[0] = DUBINS_SEGS[i][0];
    candidate.types[1] = DUBINS_SEGS[i][1];
    candidate.types[2] = DUBINS_SEGS[i][2];
    candidate.total_length =
        (words[i].t + words[i].p + words[i].q) * r;
    candidate.radius = r;
    candidate.valid = true;
    candidates.push_back(candidate);
  }
  return candidates;
}

void sampleDubinsSegment(double& x, double& y, double& theta,
                          int seg_type, double seg_len, double r, double step,
                          std::vector<double>& xs, std::vector<double>& ys,
                          std::vector<double>& ts) {
  double world_len = (seg_type == S_SEG) ? seg_len * r : fabs(seg_len) * r;
  int n = std::max(1, (int)ceil(world_len / step));
  double dt = seg_len / n;

  for (int i = 0; i < n; ++i) {
    if (seg_type == L_SEG) {
      double nt = theta + dt;
      x += r * (sin(nt) - sin(theta));
      y -= r * (cos(nt) - cos(theta));
      theta = nt;
    } else if (seg_type == R_SEG) {
      double nt = theta - dt;
      x += r * (sin(theta) - sin(nt));
      y += r * (cos(nt) - cos(theta));
      theta = nt;
    } else {
      x += dt * r * cos(theta);
      y += dt * r * sin(theta);
    }
    xs.push_back(x);
    ys.push_back(y);
    ts.push_back(mod2pi(theta));
  }
  theta = mod2pi(theta);
}

}  // anonymous namespace

// ==================== HybridAStarPlanner ====================

HybridAStarPlanner::HybridAStarPlanner()
    : HybridAStarPlanner(false) {}

HybridAStarPlanner::HybridAStarPlanner(bool basic_mode)
    : initialized_(false), basic_mode_(basic_mode) {}

HybridAStarPlanner::HybridAStarPlanner(std::string name,
                                         costmap_2d::Costmap2DROS* costmap_ros)
    : HybridAStarPlanner(false) {
  initialize(name, costmap_ros);
}

BasicHybridAStarPlanner::BasicHybridAStarPlanner()
    : HybridAStarPlanner(true) {}

void BasicHybridAStarPlanner::initialize(
    std::string /* name */,
    costmap_2d::Costmap2DROS* costmap_ros) {
  // Load the same namespace as the terrain-aware planner. This keeps every
  // kinematic, obstacle and goal parameter identical in the A/B experiment.
  HybridAStarPlanner::initialize("HybridAStarPlanner", costmap_ros);
}

void HybridAStarPlanner::initialize(std::string name,
                                     costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) return;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle nh("~/" + name);
  nh.param("min_turning_radius",       min_turning_radius_,       0.5);
  nh.param("step_size",                step_size_,                0.5);
  nh.param("heading_bins",             heading_bins_,             72);
  nh.param("max_iterations",           max_iterations_,           200000);
  nh.param("goal_xy_tolerance",        goal_xy_tol_,              0.3);
  nh.param("goal_yaw_tolerance",       goal_yaw_tol_,             0.10);
  nh.param("enable_dubins_shot",        enable_dubins_shot_,       false);
  nh.param("dubins_shot_max_distance", dubins_shot_max_distance_, 3.0);
  nh.param("allow_goal_yaw_relaxation",
           allow_goal_yaw_relaxation_, false);
  nh.param("goal_yaw_relaxation_penalty",
           goal_yaw_relaxation_penalty_, 0.2);
  nh.param("enforce_terminal_goal_yaw",
           enforce_terminal_goal_yaw_, true);
  nh.param("terminal_approach_distance",
           terminal_approach_distance_, 2.5);
  nh.param("terminal_heading_blend_distance",
           terminal_heading_blend_distance_, 3.0);
  nh.param("terminal_heading_soft_weight",
           terminal_heading_soft_weight_, 4.0);
  nh.param("terminal_candidate_settle_iterations",
           terminal_candidate_settle_iterations_, 1500);
  nh.param("terminal_transition_tangent_scale",
           terminal_transition_tangent_scale_, 1.2);
  nh.param("terminal_transition_max_backtrack",
           terminal_transition_max_backtrack_, 2.0);
  nh.param("analytic_expansion_skip",  analytic_expansion_skip_,  5);
  nh.param("analytic_min_prefix_distance",
           analytic_min_prefix_distance_, 6.0);
  nh.param("analytic_max_mean_terrain_cost",
           analytic_max_mean_terrain_cost_, 0.30);
  nh.param("analytic_max_peak_terrain_cost",
           analytic_max_peak_terrain_cost_, 0.55);
  nh.param("allow_reverse",            allow_reverse_,            false);
  nh.param("allow_start_in_place_rotation",
           allow_start_in_place_rotation_, false);
  nh.param("start_straight_distance", start_straight_distance_, 1.0);
  start_straight_distance_ = std::max(0.0, start_straight_distance_);
  terminal_approach_distance_ =
      std::max(0.0, terminal_approach_distance_);
  terminal_heading_blend_distance_ =
      std::max(goal_xy_tol_, terminal_heading_blend_distance_);
  terminal_heading_soft_weight_ =
      std::max(0.0, terminal_heading_soft_weight_);
  terminal_candidate_settle_iterations_ =
      std::max(0, terminal_candidate_settle_iterations_);
  terminal_transition_tangent_scale_ =
      std::max(0.5, terminal_transition_tangent_scale_);
  terminal_transition_max_backtrack_ =
      std::max(0.0, terminal_transition_max_backtrack_);
  nh.param("reverse_penalty",          reverse_penalty_,          1.5);
  nh.param("direction_change_penalty", direction_change_penalty_,  0.0);
  nh.param("num_steering_angles",      num_steering_angles_,      5);
  nh.param("obstacle_penalty",         obstacle_penalty_,         0.5);
  nh.param("curvature_penalty",       curvature_penalty_,        1.5);
  nh.param("fine_curvature_change_weight",
           fine_curvature_change_weight_, curvature_penalty_);
  nh.param("min_clearance",            min_clearance_,            1.67);

  nh.param<std::string>("terrain_cost_file",        terrain_cost_file_,        "");
  nh.param<std::string>("terrain_cost_coarse_file", terrain_cost_coarse_file_, "");
  nh.param("terrain_cost_weight",  terrain_cost_weight_,  12.0);
  nh.param("pareto_terrain_mode",  pareto_terrain_mode_,  true);
  nh.param("pareto_high_risk_quantile",
           pareto_high_risk_quantile_, 0.8);
  nh.param("pareto_anchor_strength", pareto_anchor_strength_, 3.0);
  nh.param("fine_risk_strength", fine_risk_strength_, 2.8);
  nh.param("fine_heuristic_weight", fine_heuristic_weight_, 1.05);
  nh.param("fine_stagnation_min_iterations",
           fine_stagnation_min_iterations_, 150000);
  nh.param("fine_stagnation_window_iterations",
           fine_stagnation_window_iterations_, 75000);
  nh.param("fine_stagnation_progress_m",
           fine_stagnation_progress_m_, 0.10);
  nh.param("primitive_collision_check_fraction",
           primitive_collision_check_fraction_, 0.75);
  nh.param("pointwise_risk_exponent", pointwise_risk_exponent_, 4.0);
  nh.param("pareto_tail_fraction", pareto_tail_fraction_, 0.10);
  nh.param("attitude_risk_enabled", attitude_risk_enabled_, true);
  double pitch_soft_start_deg = 10.0;
  double roll_soft_start_deg = 8.0;
  double pitch_hard_limit_deg = 40.0;
  double roll_hard_limit_deg = 30.0;
  nh.param("pitch_soft_start_deg", pitch_soft_start_deg, 10.0);
  nh.param("roll_soft_start_deg", roll_soft_start_deg, 8.0);
  nh.param("pitch_hard_limit_deg", pitch_hard_limit_deg, 40.0);
  nh.param("roll_hard_limit_deg", roll_hard_limit_deg, 30.0);
  nh.param("pitch_risk_weight", pitch_risk_weight_, 1.0);
  nh.param("roll_risk_weight", roll_risk_weight_, 1.8);
  nh.param("attitude_penalty_exponent", attitude_penalty_exponent_, 2.0);
  nh.param("adaptive_safe_candidate_enabled",
           adaptive_safe_candidate_enabled_, true);
  nh.param("adaptive_safe_peak_risk_trigger",
           adaptive_safe_peak_risk_trigger_, 0.90);
  nh.param("adaptive_safe_peak_quantile_margin",
           adaptive_safe_peak_quantile_margin_, 0.20);
  double adaptive_safe_pitch_trigger_deg = 20.0;
  double adaptive_safe_roll_trigger_deg = 16.0;
  nh.param("adaptive_safe_pitch_trigger_deg",
           adaptive_safe_pitch_trigger_deg, 20.0);
  nh.param("adaptive_safe_roll_trigger_deg",
           adaptive_safe_roll_trigger_deg, 16.0);
  nh.param("adaptive_safe_risk_scale",
           adaptive_safe_risk_scale_, 1.5);
  nh.param("adaptive_safe_attitude_scale",
           adaptive_safe_attitude_scale_, 2.0);
  pitch_soft_start_deg = std::max(0.0, pitch_soft_start_deg);
  roll_soft_start_deg = std::max(0.0, roll_soft_start_deg);
  pitch_hard_limit_deg = std::max(
      pitch_soft_start_deg + 1.0, pitch_hard_limit_deg);
  roll_hard_limit_deg = std::max(
      roll_soft_start_deg + 1.0, roll_hard_limit_deg);
  pitch_soft_start_rad_ = pitch_soft_start_deg * M_PI / 180.0;
  roll_soft_start_rad_ = roll_soft_start_deg * M_PI / 180.0;
  pitch_hard_limit_rad_ = pitch_hard_limit_deg * M_PI / 180.0;
  roll_hard_limit_rad_ = roll_hard_limit_deg * M_PI / 180.0;
  pitch_risk_weight_ = std::max(0.0, pitch_risk_weight_);
  roll_risk_weight_ = std::max(0.0, roll_risk_weight_);
  attitude_penalty_exponent_ = std::max(1.0, attitude_penalty_exponent_);
  adaptive_safe_peak_risk_trigger_ = std::max(
      0.0, std::min(1.0, adaptive_safe_peak_risk_trigger_));
  adaptive_safe_peak_quantile_margin_ = std::max(
      0.0, std::min(1.0, adaptive_safe_peak_quantile_margin_));
  adaptive_safe_pitch_trigger_deg = std::max(
      pitch_soft_start_deg, std::min(pitch_hard_limit_deg,
                                     adaptive_safe_pitch_trigger_deg));
  adaptive_safe_roll_trigger_deg = std::max(
      roll_soft_start_deg, std::min(roll_hard_limit_deg,
                                    adaptive_safe_roll_trigger_deg));
  adaptive_safe_pitch_trigger_rad_ =
      adaptive_safe_pitch_trigger_deg * M_PI / 180.0;
  adaptive_safe_roll_trigger_rad_ =
      adaptive_safe_roll_trigger_deg * M_PI / 180.0;
  adaptive_safe_risk_scale_ = std::max(1.0, adaptive_safe_risk_scale_);
  adaptive_safe_attitude_scale_ =
      std::max(1.0, adaptive_safe_attitude_scale_);
  nh.param("pareto_candidate_count", pareto_candidate_count_, 4);
  nh.param("pareto_safety_filter_enabled",
           pareto_safety_filter_enabled_, false);
  if (pareto_safety_filter_enabled_) {
    ROS_WARN("pareto_safety_filter_enabled is deprecated and ignored: "
             "terrain risk is compared as a Pareto objective");
    pareto_safety_filter_enabled_ = false;
  }
  nh.param("pareto_max_high_risk_fraction",
           pareto_max_high_risk_fraction_, 0.02);
  nh.param("pareto_peak_risk_margin", pareto_peak_risk_margin_, 0.05);
  nh.param("pareto_tail_risk_margin", pareto_tail_risk_margin_, 0.0);
  pareto_candidate_count_ = std::max(2, pareto_candidate_count_);
  nh.param("corridor_width",       corridor_width_,       5.0);
  nh.param("corridor_min_width",   corridor_min_width_,   1.0);
  nh.param("corridor_risk_margin", corridor_risk_margin_, 0.15);
  nh.param("corridor_center_weight", corridor_center_weight_, 0.0);
  nh.param("fine_topology_preservation_weight",
           fine_topology_preservation_weight_, 0.08);
  nh.param("adaptive_safe_topology_preservation_weight",
           adaptive_safe_topology_preservation_weight_, 0.30);
  nh.param("fine_min_hausdorff", fine_min_hausdorff_, 0.75);
  nh.param("fine_max_overlap", fine_max_overlap_, 0.92);
  nh.param("use_multi_heuristic_search", use_multi_heuristic_search_, true);
  nh.param("mha_anchor_weight",     mha_anchor_weight_,     1.0);
  nh.param("mha_risk_weight",       mha_risk_weight_,       1.0);
  nh.param("mha_heading_weight",    mha_heading_weight_,    1.0);
  nh.param("mha_heading_gain",      mha_heading_gain_,      1.0);
  nh.param("mha_queue_bound",       mha_queue_bound_,       1.5);
  nh.param("max_coarse_channels",    max_coarse_channels_,
           pareto_candidate_count_);
  nh.param("max_fine_channels",      max_fine_channels_, 3);
  nh.param("coarse_pool_extra_candidates",
           coarse_pool_extra_candidates_, 2);
  nh.param("max_duplicate_attempts", max_duplicate_attempts_, 2);
  nh.param("coarse_search_time_budget",
           coarse_search_time_budget_, 0.5);
  nh.param("coarse_search_time_per_meter",
           coarse_search_time_per_meter_, 0.04);
  nh.param("coarse_search_time_max",
           coarse_search_time_max_, 2.0);
  nh.param("coarse_max_detour_ratio",
           coarse_max_detour_ratio_, 0.30);
  nh.param("pareto_decision_detour_ratio",
           pareto_decision_detour_ratio_, 0.20);
  nh.param("pareto_score_near_tie_margin",
           pareto_score_near_tie_margin_, 0.02);
  nh.param("coarse_diversity_radius",
           coarse_diversity_radius_, 3.0);
  nh.param("coarse_diversity_weight",
           coarse_diversity_weight_, 2.0);
  nh.param("coarse_min_hausdorff",
           coarse_min_hausdorff_, 2.0);
  nh.param("coarse_min_separated_length",
           coarse_min_separated_length_, 3.0);
  nh.param("coarse_multi_label_enabled",
           coarse_multi_label_enabled_, true);
  nh.param("coarse_labels_per_state",
           coarse_labels_per_state_, 4);
  nh.param("coarse_label_epsilon_length",
           coarse_label_epsilon_length_, 0.5);
  nh.param("coarse_label_epsilon_risk",
           coarse_label_epsilon_risk_, 0.02);
  nh.param("coarse_topology_signature_enabled",
           coarse_topology_signature_enabled_, true);
  nh.param("coarse_topology_gate_first_ratio",
           coarse_topology_gate_first_ratio_, 0.30);
  nh.param("coarse_topology_gate_second_ratio",
           coarse_topology_gate_second_ratio_, 0.65);
  nh.param("coarse_topology_lateral_bin_width",
           coarse_topology_lateral_bin_width_, 3.0);
  nh.param("coarse_goal_label_limit",
           coarse_goal_label_limit_, 12);
  nh.param("coarse_goal_settle_iterations",
           coarse_goal_settle_iterations_, 8000);
  nh.param("coarse_min_stable_channels",
           coarse_min_stable_channels_, 2);
  nh.param("coarse_min_goal_labels_for_settle",
           coarse_min_goal_labels_for_settle_, 6);
  nh.param("coarse_single_topology_settle_iterations",
           coarse_single_topology_settle_iterations_, 16000);
  nh.param("coarse_single_topology_anchor_slack",
           coarse_single_topology_anchor_slack_, 0.03);
  nh.param("coarse_multi_label_time_limit",
           coarse_multi_label_time_limit_, 3.0);
  max_coarse_channels_ = std::max(1, max_coarse_channels_);
  max_fine_channels_ =
      std::max(1, std::min(max_coarse_channels_, max_fine_channels_));
  coarse_pool_extra_candidates_ =
      std::max(0, std::min(4, coarse_pool_extra_candidates_));
  max_duplicate_attempts_ = std::max(1, max_duplicate_attempts_);
  coarse_search_time_budget_ = std::max(0.0, coarse_search_time_budget_);
  coarse_search_time_per_meter_ =
      std::max(0.0, coarse_search_time_per_meter_);
  coarse_search_time_max_ = std::max(0.0, coarse_search_time_max_);
  coarse_max_detour_ratio_ =
      std::max(0.0, std::min(1.0, coarse_max_detour_ratio_));
  pareto_decision_detour_ratio_ = std::max(
      1e-3, std::min(coarse_max_detour_ratio_,
                     pareto_decision_detour_ratio_));
  pareto_score_near_tie_margin_ = std::max(
      0.0, std::min(0.10, pareto_score_near_tie_margin_));
  coarse_diversity_radius_ = std::max(0.1, coarse_diversity_radius_);
  coarse_diversity_weight_ = std::max(0.0, coarse_diversity_weight_);
  coarse_min_hausdorff_ = std::max(0.0, coarse_min_hausdorff_);
  coarse_min_separated_length_ =
      std::max(0.5, coarse_min_separated_length_);
  coarse_labels_per_state_ =
      std::max(2, std::min(5, coarse_labels_per_state_));
  coarse_label_epsilon_length_ =
      std::max(0.0, coarse_label_epsilon_length_);
  coarse_label_epsilon_risk_ =
      std::max(0.0, std::min(0.25, coarse_label_epsilon_risk_));
  coarse_topology_gate_first_ratio_ = std::max(
      0.10, std::min(0.80, coarse_topology_gate_first_ratio_));
  coarse_topology_gate_second_ratio_ = std::max(
      coarse_topology_gate_first_ratio_ + 0.10,
      std::min(0.90, coarse_topology_gate_second_ratio_));
  coarse_topology_lateral_bin_width_ = std::max(
      coarse_map_.res > 0.0 ? coarse_map_.res : 0.5,
      coarse_topology_lateral_bin_width_);
  coarse_goal_label_limit_ =
      std::max(max_coarse_channels_, coarse_goal_label_limit_);
  coarse_goal_settle_iterations_ =
      std::max(0, coarse_goal_settle_iterations_);
  coarse_min_stable_channels_ = std::max(
      1, std::min(max_coarse_channels_, coarse_min_stable_channels_));
  coarse_min_goal_labels_for_settle_ = std::max(
      coarse_min_stable_channels_,
      std::min(coarse_goal_label_limit_,
               coarse_min_goal_labels_for_settle_));
  coarse_single_topology_settle_iterations_ = std::max(
      coarse_goal_settle_iterations_,
      coarse_single_topology_settle_iterations_);
  coarse_single_topology_anchor_slack_ = std::max(
      0.0, std::min(0.25, coarse_single_topology_anchor_slack_));
  coarse_multi_label_time_limit_ =
      std::max(0.0, coarse_multi_label_time_limit_);
  pareto_high_risk_quantile_ =
      std::max(0.0, std::min(1.0, pareto_high_risk_quantile_));
  pareto_anchor_strength_ = std::max(1.0, pareto_anchor_strength_);
  fine_risk_strength_ = std::max(0.0, fine_risk_strength_);
  fine_curvature_change_weight_ =
      std::max(0.0, fine_curvature_change_weight_);
  fine_heuristic_weight_ = std::max(
      1.0, std::min(1.5, fine_heuristic_weight_));
  fine_stagnation_min_iterations_ = std::max(
      10000, std::min(max_iterations_, fine_stagnation_min_iterations_));
  fine_stagnation_window_iterations_ = std::max(
      10000, std::min(max_iterations_, fine_stagnation_window_iterations_));
  fine_stagnation_progress_m_ = std::max(
      0.01, std::min(1.0, fine_stagnation_progress_m_));
  primitive_collision_check_fraction_ = std::max(
      0.5, std::min(1.0, primitive_collision_check_fraction_));
  pointwise_risk_exponent_ = std::max(1.0, pointwise_risk_exponent_);
  pareto_tail_fraction_ =
      std::max(0.01, std::min(0.5, pareto_tail_fraction_));
  pareto_max_high_risk_fraction_ = std::max(
      0.0, std::min(1.0, pareto_max_high_risk_fraction_));
  pareto_peak_risk_margin_ =
      std::max(0.0, std::min(1.0, pareto_peak_risk_margin_));
  pareto_tail_risk_margin_ =
      std::max(0.0, std::min(1.0, pareto_tail_risk_margin_));
  pareto_high_risk_threshold_ = 1.0;
  mha_anchor_weight_ = std::max(1.0, mha_anchor_weight_);
  mha_risk_weight_ = std::max(0.0, mha_risk_weight_);
  mha_heading_weight_ = std::max(0.0, mha_heading_weight_);
  mha_heading_gain_ = std::max(0.0, mha_heading_gain_);
  mha_queue_bound_ = std::max(1.0, mha_queue_bound_);
  corridor_width_ = std::max(0.1, corridor_width_);
  corridor_min_width_ =
      std::max(0.1, std::min(corridor_width_, corridor_min_width_));
  corridor_risk_margin_ = std::max(0.0, corridor_risk_margin_);
  corridor_center_weight_ = std::max(0.0, corridor_center_weight_);
  fine_topology_preservation_weight_ =
      std::max(0.0, fine_topology_preservation_weight_);
  adaptive_safe_topology_preservation_weight_ = std::max(
      fine_topology_preservation_weight_,
      adaptive_safe_topology_preservation_weight_);
  fine_min_hausdorff_ = std::max(0.0, fine_min_hausdorff_);
  fine_max_overlap_ =
      std::max(0.0, std::min(1.0, fine_max_overlap_));

  if (basic_mode_) {
    terrain_cost_weight_ = 0.0;
    pareto_terrain_mode_ = false;
    use_multi_heuristic_search_ = false;
    attitude_risk_enabled_ = false;
    adaptive_safe_candidate_enabled_ = false;
  }

  cm_width_ = 0;
  cm_height_ = 0;
  use_corridor_ = false;
  active_corridor_center_weight_ = 0.0;
  active_diversity_weight_ = 0.0;
  active_high_risk_weight_ = 0.0;
  active_pointwise_risk_weight_ = 0.0;
  active_attitude_risk_scale_ = 1.0;
  active_curvature_change_weight_ = 0.0;
  active_search_max_path_length_ =
      std::numeric_limits<double>::infinity();
  active_map_ = nullptr;
  theta_res_ = 2.0 * M_PI / heading_bins_;

  if (!terrain_cost_file_.empty())
    loadTerrainMap(terrain_cost_file_, fine_map_);
  if (!terrain_cost_coarse_file_.empty())
    loadTerrainMap(terrain_cost_coarse_file_, coarse_map_);

  if (fine_map_.loaded) {
    pareto_high_risk_threshold_ = computeHighRiskThreshold(fine_map_);
  } else if (coarse_map_.loaded) {
    pareto_high_risk_threshold_ = computeHighRiskThreshold(coarse_map_);
  }
  ROS_INFO("Pareto high-risk exposure: map quantile=%.2f threshold=%.3f",
           pareto_high_risk_quantile_, pareto_high_risk_threshold_);

  active_map_ = fine_map_.loaded ? &fine_map_ : nullptr;

  ros::NodeHandle global_nh;
  plan_pub_ = global_nh.advertise<nav_msgs::Path>("/hybrid_astar/plan", 1, true);
  coarse_paths_pub_ = global_nh.advertise<visualization_msgs::MarkerArray>(
      "/hybrid_astar/coarse_paths", 1, true);
  corridor_paths_pub_ = global_nh.advertise<visualization_msgs::MarkerArray>(
      "/hybrid_astar/corridors", 1, true);
  fine_candidates_pub_ = global_nh.advertise<visualization_msgs::MarkerArray>(
      "/hybrid_astar/fine_candidates", 1, true);
  pareto_decision_pub_ = global_nh.advertise<visualization_msgs::MarkerArray>(
      "/hybrid_astar/pareto_decision", 1, true);
  planner_statistics_pub_ = global_nh.advertise<std_msgs::String>(
      "/hybrid_astar/planner_statistics", 10, true);

  initialized_ = true;
  ROS_INFO("%s initialized: min_r=%.2f step=%.2f bins=%d "
           "terrain_cost=%s(%.2fm) coarse=%s(%.2fm) corridor=%.1fm "
           "dubins_shot=%s(near<=%.1fm, prefix>=%.1fm, "
           "terrain_mean<=%.2f peak<=%.2f, goal-yaw-relax=%s, "
           "terminal-approach=%s/%.1fm blend=%.1fm soft-weight=%.1f settle=%d) "
           "terrain_selection=%s(%d candidates, anchor_strength=%.1f) "
           "start_rotation=%s "
           "start_straight=%.2fm "
           "coarse_search=%s(channels<=%d, fine<=%d, "
           "budget=%.1f+%.2f/m, cap=%.1fs) "
           "corridor=max%.1fm/min%.1fm risk_margin=%.2f center=%.2f",
           basic_mode_ ? "Basic Hybrid A* (terrain risk disabled)"
                       : "Terrain-aware Hybrid A*",
           min_turning_radius_, step_size_, heading_bins_,
           fine_map_.loaded ? "loaded" : "none", fine_map_.res,
           coarse_map_.loaded ? "loaded" : "none", coarse_map_.res,
           corridor_width_, enable_dubins_shot_ ? "on" : "off",
           dubins_shot_max_distance_,
           analytic_min_prefix_distance_,
           analytic_max_mean_terrain_cost_,
           analytic_max_peak_terrain_cost_,
           allow_goal_yaw_relaxation_ ? "on" : "off",
           enforce_terminal_goal_yaw_ ? "soft-tangent" : "position-only",
           terminal_approach_distance_, terminal_heading_blend_distance_,
           terminal_heading_soft_weight_,
           terminal_candidate_settle_iterations_,
           pareto_terrain_mode_ ? "pareto" : "weighted",
           max_coarse_channels_,
           pareto_anchor_strength_,
           allow_start_in_place_rotation_ ? "on" : "off",
           start_straight_distance_,
           use_multi_heuristic_search_ ? "shared-state MHA*" : "single A*",
           max_coarse_channels_, max_fine_channels_,
           coarse_search_time_budget_, coarse_search_time_per_meter_,
           coarse_search_time_max_, corridor_width_, corridor_min_width_,
           corridor_risk_margin_, corridor_center_weight_);
}

// ==================== Terrain map I/O ====================

bool HybridAStarPlanner::loadTerrainMap(const std::string& path, TerrainMap& map) {
  std::ifstream f(path, std::ios::binary);
  if (!f) { ROS_WARN("Cannot open terrain cost: %s", path.c_str()); return false; }

  char magic[4];
  f.read(magic, 4);
  if (std::strncmp(magic, "TCM1", 4) != 0) {
    ROS_WARN("Invalid magic in %s (expected TCM1)", path.c_str()); return false;
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

  if (!f.good()) { ROS_WARN("Terrain cost read incomplete: %s", path.c_str()); return false; }

  map.loaded = true;
  ROS_INFO("Terrain cost loaded: %s (%dx%d, %.2fm/cell, mu=%.2f, B=%.3fm)",
           path.c_str(), map.cols, map.rows, map.res, map.friction_coeff, map.track_width);
  return true;
}

float HybridAStarPlanner::bilinearQuery(const TerrainMap& map,
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

float HybridAStarPlanner::getDEMZ(double x, double y) const {
  if (!active_map_ || !active_map_->loaded) return 0.0f;
  return bilinearQuery(*active_map_, active_map_->dem, x, y);
}

double HybridAStarPlanner::attitudeSoftRisk(
    double absolute_angle, double soft_start, double hard_limit) const {
  if (!attitude_risk_enabled_ || absolute_angle <= soft_start) return 0.0;
  const double normalized = std::max(
      0.0, std::min(1.0, (absolute_angle - soft_start) /
                             std::max(1e-6, hard_limit - soft_start)));
  return std::pow(normalized, attitude_penalty_exponent_);
}

HybridAStarPlanner::TerrainSample
HybridAStarPlanner::queryTerrainSample(
    double x, double y, double theta) const {
  TerrainSample sample;
  if (!active_map_ || !active_map_->loaded) return sample;

  const double gx = bilinearQuery(
      *active_map_, active_map_->grad_x, x, y);
  const double gy = bilinearQuery(
      *active_map_, active_map_->grad_y, x, y);
  if (!std::isfinite(gx) || !std::isfinite(gy)) {
    sample.risk = basic_mode_ ? 0.0 : 1.0;
    sample.attitude.feasible = false;
    sample.attitude.pitch_risk = 1.0;
    sample.attitude.roll_risk = 1.0;
    return sample;
  }

  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double longitudinal_gradient = gx * ct + gy * st;
  const double lateral_gradient = -gx * st + gy * ct;
  sample.attitude.pitch_rad = std::atan(longitudinal_gradient);
  sample.attitude.roll_rad = std::atan(lateral_gradient);
  sample.attitude.pitch_risk = attitudeSoftRisk(
      std::fabs(sample.attitude.pitch_rad), pitch_soft_start_rad_,
      pitch_hard_limit_rad_);
  sample.attitude.roll_risk = attitudeSoftRisk(
      std::fabs(sample.attitude.roll_rad), roll_soft_start_rad_,
      roll_hard_limit_rad_);
  sample.attitude.feasible = !attitude_risk_enabled_ ||
      (std::fabs(sample.attitude.pitch_rad) <=
           pitch_hard_limit_rad_ + 1e-9 &&
       std::fabs(sample.attitude.roll_rad) <=
           roll_hard_limit_rad_ + 1e-9);

  if (basic_mode_) return sample;

  const double slope_total = std::hypot(gx, gy);
  const double mu = std::max(
      1e-6, static_cast<double>(active_map_->friction_coeff));
  const double tan_crit = std::max(
      1e-6, static_cast<double>(active_map_->track_width) /
                (2.0 * std::max(
                    1e-6, static_cast<double>(active_map_->cg_height))));
  const double c_climb = std::min(
      1.0, std::fabs(longitudinal_gradient) / mu);
  const double c_roll = std::min(
      1.0, std::fabs(lateral_gradient) / tan_crit);
  const double c_side_slip = std::min(
      1.0, std::fabs(lateral_gradient) / mu);
  const double c_worst_case_slip = std::min(1.0, slope_total / mu);

  // Resolve every online constraint in the vehicle frame. The former
  // slope_total/mu term was heading independent and always no smaller than
  // c_climb; with the current vehicle parameters it therefore erased all
  // directional information. The maximum below is a weight-free separable
  // traction/roll envelope: 1.0 means at least one physical limit is reached.
  if (pareto_terrain_mode_) {
    sample.risk = std::max(
        c_climb, std::max(c_roll, c_side_slip));
  } else {
    // Legacy scalar mode keeps the offline worst-case slope term intact for
    // strict A/B tests and immediate rollback.
    sample.risk = 0.4 * c_climb + 0.3 * c_roll +
                  0.3 * c_worst_case_slip;
  }
  return sample;
}

// Heading-dependent terrain cost query using grad_x/grad_y.
double HybridAStarPlanner::queryTerrainCost(
    double x, double y, double theta) const {
  if (basic_mode_) return 0.0;
  return queryTerrainSample(x, y, theta).risk;
}

HybridAStarPlanner::TerrainAttitude
HybridAStarPlanner::queryTerrainAttitude(
    double x, double y, double theta) const {
  return queryTerrainSample(x, y, theta).attitude;
}

double HybridAStarPlanner::computeHighRiskThreshold(
    const TerrainMap& map) const {
  std::vector<double> risks;
  risks.reserve(map.grad_x.size());
  const double mu = std::max(
      1e-6, static_cast<double>(map.friction_coeff));
  const size_t count = std::min(
      map.grad_x.size(),
      std::min(map.grad_y.size(), map.cost_total.size()));
  for (size_t index = 0; index < count; ++index) {
    const float gx = map.grad_x[index];
    const float gy = map.grad_y[index];
    const float stored_cost = map.cost_total[index];
    if (!std::isfinite(gx) || !std::isfinite(gy) ||
        !std::isfinite(stored_cost) || stored_cost >= 0.999f) {
      continue;
    }
    const double risk = std::min(1.0, std::hypot(gx, gy) / mu);
    // Excluding exact flat cells prevents a mostly flat map from producing a
    // zero threshold and classifying the whole route as high risk.
    if (risk > 1e-4) risks.push_back(risk);
  }
  if (risks.empty()) return 1.0;

  const size_t quantile_index = static_cast<size_t>(std::floor(
      pareto_high_risk_quantile_ * static_cast<double>(risks.size() - 1)));
  std::nth_element(risks.begin(), risks.begin() + quantile_index, risks.end());
  return std::max(1e-4, std::min(1.0, risks[quantile_index]));
}

HybridAStarPlanner::PathMetrics HybridAStarPlanner::evaluatePathMetrics(
    const std::vector<geometry_msgs::PoseStamped>& path) const {
  PathMetrics metrics;
  if (path.size() < 2) return metrics;

  double risk_integral = 0.0;
  double high_risk_length = 0.0;
  double pitch_angle_integral = 0.0;
  double roll_angle_integral = 0.0;
  double pitch_risk_integral = 0.0;
  double roll_risk_integral = 0.0;
  double pitch_exposure_length = 0.0;
  double roll_exposure_length = 0.0;
  double max_pitch_risk = 0.0;
  double max_roll_risk = 0.0;
  std::vector<std::pair<double, double>> risk_lengths;
  std::vector<std::pair<double, double>> pitch_lengths;
  std::vector<std::pair<double, double>> roll_lengths;
  risk_lengths.reserve(path.size() - 1);
  pitch_lengths.reserve(path.size() - 1);
  roll_lengths.reserve(path.size() - 1);
  for (size_t i = 1; i < path.size(); ++i) {
    const auto& previous = path[i - 1];
    const auto& current = path[i];
    const double dx =
        current.pose.position.x - previous.pose.position.x;
    const double dy =
        current.pose.position.y - previous.pose.position.y;
    const double ds = std::hypot(dx, dy);
    if (ds < 1e-6) continue;

    metrics.length += ds;
    const double sample_step =
        active_map_ && active_map_->loaded
            ? std::max(0.02, 0.5 * active_map_->res)
            : ds;
    const int sample_count =
        std::max(1, static_cast<int>(std::ceil(ds / sample_step)));
    const double sample_length = ds / sample_count;
    // Evaluate attitude in the actual path-tangent frame.  This keeps the
    // result independent of stale/interpolated Pose orientations after
    // spline smoothing and makes the search and evaluation conventions agree.
    const double theta = std::atan2(dy, dx);
    for (int sample = 1; sample <= sample_count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / sample_count;
      const double x = previous.pose.position.x + ratio * dx;
      const double y = previous.pose.position.y + ratio * dy;
      const TerrainSample terrain = queryTerrainSample(x, y, theta);
      const double risk = terrain.risk;
      const TerrainAttitude& attitude = terrain.attitude;
      if (!attitude.feasible) metrics.attitude_feasible = false;
      const double absolute_pitch = std::fabs(attitude.pitch_rad);
      const double absolute_roll = std::fabs(attitude.roll_rad);
      risk_integral += risk * sample_length;
      metrics.peak_risk = std::max(metrics.peak_risk, risk);
      risk_lengths.emplace_back(risk, sample_length);
      pitch_lengths.emplace_back(absolute_pitch, sample_length);
      roll_lengths.emplace_back(absolute_roll, sample_length);
      pitch_angle_integral += absolute_pitch * sample_length;
      roll_angle_integral += absolute_roll * sample_length;
      pitch_risk_integral += attitude.pitch_risk * sample_length;
      roll_risk_integral += attitude.roll_risk * sample_length;
      max_pitch_risk = std::max(max_pitch_risk, attitude.pitch_risk);
      max_roll_risk = std::max(max_roll_risk, attitude.roll_risk);
      metrics.max_abs_pitch_deg = std::max(
          metrics.max_abs_pitch_deg, absolute_pitch * 180.0 / M_PI);
      metrics.max_abs_roll_deg = std::max(
          metrics.max_abs_roll_deg, absolute_roll * 180.0 / M_PI);
      if (absolute_pitch >= pitch_soft_start_rad_)
        pitch_exposure_length += sample_length;
      if (absolute_roll >= roll_soft_start_rad_)
        roll_exposure_length += sample_length;
      if (risk >= pareto_high_risk_threshold_)
        high_risk_length += sample_length;
    }
  }

  if (metrics.length > 1e-6) {
    metrics.mean_risk = risk_integral / metrics.length;
    metrics.high_risk_fraction = high_risk_length / metrics.length;

    // Arc-length weighted CVaR of the riskiest part of the route. Unlike a
    // single peak sample, this remains stable under map noise and path
    // resampling, while still exposing short dangerous sections that the
    // whole-path mean can hide.
    std::sort(risk_lengths.begin(), risk_lengths.end(),
              [](const std::pair<double, double>& lhs,
                 const std::pair<double, double>& rhs) {
                return lhs.first > rhs.first;
              });
    const double tail_length = pareto_tail_fraction_ * metrics.length;
    double accumulated_length = 0.0;
    double tail_integral = 0.0;
    for (const auto& sample : risk_lengths) {
      if (accumulated_length >= tail_length) break;
      const double used =
          std::min(sample.second, tail_length - accumulated_length);
      tail_integral += sample.first * used;
      accumulated_length += used;
    }
    if (accumulated_length > 1e-9)
      metrics.tail_risk = tail_integral / accumulated_length;

    auto weighted_percentile = [](std::vector<std::pair<double, double>> data,
                                  double quantile, double total_length) {
      if (data.empty() || total_length <= 1e-9) return 0.0;
      std::sort(data.begin(), data.end(),
                [](const std::pair<double, double>& lhs,
                   const std::pair<double, double>& rhs) {
                  return lhs.first < rhs.first;
                });
      const double target = quantile * total_length;
      double accumulated = 0.0;
      for (const auto& sample : data) {
        accumulated += sample.second;
        if (accumulated + 1e-12 >= target) return sample.first;
      }
      return data.back().first;
    };

    metrics.mean_abs_pitch_deg =
        pitch_angle_integral / metrics.length * 180.0 / M_PI;
    metrics.mean_abs_roll_deg =
        roll_angle_integral / metrics.length * 180.0 / M_PI;
    const double p95_pitch = weighted_percentile(
        pitch_lengths, 0.95, metrics.length);
    const double p95_roll = weighted_percentile(
        roll_lengths, 0.95, metrics.length);
    metrics.p95_abs_pitch_deg = p95_pitch * 180.0 / M_PI;
    metrics.p95_abs_roll_deg = p95_roll * 180.0 / M_PI;
    metrics.pitch_exposure_fraction =
        pitch_exposure_length / metrics.length;
    metrics.roll_exposure_fraction = roll_exposure_length / metrics.length;

    const double mean_pitch_risk = pitch_risk_integral / metrics.length;
    const double mean_roll_risk = roll_risk_integral / metrics.length;
    const double p95_pitch_risk = attitudeSoftRisk(
        p95_pitch, pitch_soft_start_rad_, pitch_hard_limit_rad_);
    const double p95_roll_risk = attitudeSoftRisk(
        p95_roll, roll_soft_start_rad_, roll_hard_limit_rad_);
    metrics.pitch_objective = std::sqrt(
        0.15 * mean_pitch_risk * mean_pitch_risk +
        0.35 * p95_pitch_risk * p95_pitch_risk +
        0.30 * max_pitch_risk * max_pitch_risk +
        0.20 * metrics.pitch_exposure_fraction *
            metrics.pitch_exposure_fraction);
    // nav_23/nav_24 showed that the aggregate roll objective could look good
    // while its single worst side-slope sample was still about 0.5 deg above
    // the risk-weighted baseline.  Put more emphasis on the maximum, but keep
    // mean/P95/exposure terms so one noisy cell cannot dictate the route.
    metrics.roll_objective = std::sqrt(
        0.10 * mean_roll_risk * mean_roll_risk +
        0.25 * p95_roll_risk * p95_roll_risk +
        0.45 * max_roll_risk * max_roll_risk +
        0.20 * metrics.roll_exposure_fraction *
            metrics.roll_exposure_fraction);
    metrics.combined_risk_objective = std::sqrt(
        0.25 * (metrics.mean_risk * metrics.mean_risk +
                metrics.peak_risk * metrics.peak_risk +
                metrics.tail_risk * metrics.tail_risk +
                metrics.high_risk_fraction *
                    metrics.high_risk_fraction));
  }
  return metrics;
}

bool HybridAStarPlanner::isParetoSafetyFeasible(
    const PathMetrics& metrics) const {
  if (!pareto_safety_filter_enabled_) return true;

  // The terrain map is normalized independently for every scene.  Use its
  // measured high-risk quantile as an adaptive gate instead of imposing one
  // slope/risk number on every map.  A small peak margin tolerates a single
  // interpolated boundary sample, while CVaR and exposure reject a sustained
  // dangerous section such as the one observed in nav_63.
  const double peak_limit =
      std::min(1.0, pareto_high_risk_threshold_ + pareto_peak_risk_margin_);
  const double tail_limit =
      std::min(1.0, pareto_high_risk_threshold_ + pareto_tail_risk_margin_);
  return metrics.peak_risk <= peak_limit + 1e-9 &&
         metrics.tail_risk <= tail_limit + 1e-9 &&
         metrics.high_risk_fraction <=
             pareto_max_high_risk_fraction_ + 1e-9;
}

double HybridAStarPlanner::computeParetoSafetyViolation(
    const PathMetrics& metrics) const {
  if (!pareto_safety_filter_enabled_) return 0.0;

  const double peak_limit =
      std::min(1.0, pareto_high_risk_threshold_ + pareto_peak_risk_margin_);
  const double tail_limit =
      std::min(1.0, pareto_high_risk_threshold_ + pareto_tail_risk_margin_);
  auto normalized_excess = [](double value, double limit) {
    if (value <= limit + 1e-9) return 0.0;
    // Normalize by the remaining range to the worst representable risk. This
    // keeps peak, tail and exposure comparable without making a small 2%
    // exposure limit dominate every other violation merely through division
    // by a small threshold.
    return (value - limit) / std::max(1e-6, 1.0 - limit);
  };

  const double peak_excess =
      normalized_excess(metrics.peak_risk, peak_limit);
  const double tail_excess =
      normalized_excess(metrics.tail_risk, tail_limit);
  const double exposure_excess = normalized_excess(
      metrics.high_risk_fraction, pareto_max_high_risk_fraction_);

  // A max-only violation saturates as soon as peak risk reaches 1.0.  That
  // made every unsafe candidate in nav_8 receive exactly the same score even
  // though their sustained exposure and CVaR were different.  Keep peak as
  // the largest safety component, but retain the continuous tail/exposure
  // information so the degraded fallback can still choose the less unsafe
  // route deterministically.
  return std::sqrt(0.45 * peak_excess * peak_excess +
                   0.35 * tail_excess * tail_excess +
                   0.20 * exposure_excess * exposure_excess);
}

int HybridAStarPlanner::selectParetoCandidate(
    const std::vector<PathMetrics>& metrics,
    std::vector<bool>* on_pareto_front,
    std::vector<double>* ideal_distances) const {
  if (metrics.empty()) return -1;

  if (on_pareto_front)
    on_pareto_front->assign(metrics.size(), false);
  if (ideal_distances)
    ideal_distances->assign(
        metrics.size(), std::numeric_limits<double>::infinity());

  // Physical obstacles, vehicle curvature and corridor clearance have already
  // been enforced during search. Terrain risk remains a soft objective here:
  // do not discard a route using an arbitrary risk threshold before Pareto
  // comparison, otherwise a difficult map can lose exactly the alternatives
  // that Pareto decision making is intended to compare.
  std::vector<int> candidate_pool;
  candidate_pool.reserve(metrics.size());
  for (size_t index = 0; index < metrics.size(); ++index)
    candidate_pool.push_back(static_cast<int>(index));

  // Keep only non-dominated paths in four interpretable objective groups:
  // distance, general terrain/traction risk, longitudinal attitude risk and
  // lateral attitude risk.  Grouping the related mean/peak/P95/exposure terms
  // prevents a high-dimensional front where almost every route appears
  // non-dominated, while still preventing sustained side slope from being
  // hidden by one scalar mean-risk value.
  const double eps = 1e-9;
  std::vector<int> front;
  for (int i : candidate_pool) {
    bool dominated = false;
    for (int j : candidate_pool) {
      if (dominated) break;
      if (i == j) continue;
      const bool no_worse =
          metrics[j].length <= metrics[i].length + eps &&
          metrics[j].combined_risk_objective <=
              metrics[i].combined_risk_objective + eps &&
          metrics[j].pitch_objective <= metrics[i].pitch_objective + eps &&
          metrics[j].roll_objective <= metrics[i].roll_objective + eps;
      const bool strictly_better =
          metrics[j].length < metrics[i].length - eps ||
          metrics[j].combined_risk_objective <
              metrics[i].combined_risk_objective - eps ||
          metrics[j].pitch_objective < metrics[i].pitch_objective - eps ||
          metrics[j].roll_objective < metrics[i].roll_objective - eps;
      dominated = no_worse && strictly_better;
    }
    if (!dominated) front.push_back(i);
  }
  if (on_pareto_front) {
    for (int index : front) (*on_pareto_front)[index] = true;
  }
  if (front.empty()) return 0;

  double min_length = std::numeric_limits<double>::infinity();
  for (int index : front) {
    min_length = std::min(min_length, metrics[index].length);
  }

  auto unit_risk = [](double value) {
    return std::max(0.0, std::min(1.0, value));
  };
  // Candidate generation may retain a wider 30% envelope, while the final
  // compromise is bounded by the configured decision detour.  Within that
  // bound compare deviation from the attainable four-dimensional utopia
  // point: zero extra distance plus the lowest general, pitch and roll risk.
  // Keep those risk groups separate all the way through the decision.  The
  // former RMS collapse allowed a large peak/tail or pitch improvement to be
  // diluted by an already-small unrelated group (the medium_03 regression).
  std::vector<int> decision_front;
  decision_front.reserve(front.size());
  for (int index : front) {
    const double relative_detour =
        min_length > 1e-9
            ? std::max(0.0, metrics[index].length / min_length - 1.0)
            : 0.0;
    if (relative_detour <= pareto_decision_detour_ratio_ + eps) {
      decision_front.push_back(index);
    }
  }
  // The shortest member is always inside the bound, but keep this defensive
  // fallback so a malformed zero-length metric cannot leave selection empty.
  if (decision_front.empty()) {
    decision_front = front;
  }

  double min_general = std::numeric_limits<double>::infinity();
  double min_pitch = std::numeric_limits<double>::infinity();
  double min_roll = std::numeric_limits<double>::infinity();
  for (int index : decision_front) {
    min_general = std::min(
        min_general, unit_risk(metrics[index].combined_risk_objective));
    min_pitch = std::min(
        min_pitch, unit_risk(metrics[index].pitch_objective));
    min_roll = std::min(
        min_roll, unit_risk(metrics[index].roll_objective));
  }

  auto regrets = [&](int index) {
    const double relative_detour =
        min_length > 1e-9
            ? std::max(0.0, metrics[index].length / min_length - 1.0)
            : 0.0;
    return std::array<double, 4>{{
        relative_detour,
        std::max(0.0, unit_risk(
            metrics[index].combined_risk_objective) - min_general),
        std::max(0.0, unit_risk(
            metrics[index].pitch_objective) - min_pitch),
        std::max(0.0, unit_risk(
            metrics[index].roll_objective) - min_roll)}};
  };

  // Select the eligible non-dominated route by group-wise Chebyshev regret.
  // No fixed distance/risk weighting is introduced: each coordinate is a
  // dimensionless distance from the best value actually attainable by the
  // current candidate set. Scores inside the configured near-tie margin are
  // indistinguishable for practical planning; prefer the member with the
  // smaller worst terrain/pitch/roll regret instead of allowing a 1% score
  // difference to buy a large safety loss (the nav_8 long_03 regression).
  struct RegretScore {
    int index;
    double score;
    double terrain_regret;
    double max_roll_deg;
  };
  std::vector<RegretScore> scored;
  scored.reserve(decision_front.size());
  double minimum_score = std::numeric_limits<double>::infinity();
  for (int index : decision_front) {
    const auto regret = regrets(index);
    const double terrain_regret = std::max(
        regret[1], std::max(regret[2], regret[3]));
    const double worst_group_regret = *std::max_element(
        regret.begin(), regret.end());
    const double mean_group_regret =
        std::accumulate(regret.begin(), regret.end(), 0.0) /
        static_cast<double>(regret.size());
    const double score =
        worst_group_regret + 1e-3 * mean_group_regret;
    if (ideal_distances) (*ideal_distances)[index] = score;
    scored.push_back(
        {index, score, terrain_regret,
         metrics[index].max_abs_roll_deg});
    minimum_score = std::min(minimum_score, score);
  }

  int best = decision_front.front();
  double best_score = std::numeric_limits<double>::infinity();
  double best_terrain_regret = std::numeric_limits<double>::infinity();
  double best_max_roll_deg = std::numeric_limits<double>::infinity();
  // Differences below a quarter degree are smaller than the practical
  // terrain-grid discrimination in this benchmark.  Only a meaningful roll
  // reduction is allowed to decide an otherwise near-tied Pareto compromise.
  constexpr double kPracticalRollTieDeg = 0.25;
  for (const RegretScore& item : scored) {
    if (item.score >
        minimum_score + pareto_score_near_tie_margin_ + eps) {
      continue;
    }
    const bool meaningfully_lower_roll =
        item.max_roll_deg <
        best_max_roll_deg - kPracticalRollTieDeg;
    const bool practically_equal_roll =
        std::fabs(item.max_roll_deg - best_max_roll_deg) <=
        kPracticalRollTieDeg;
    if (meaningfully_lower_roll ||
        (practically_equal_roll &&
         (item.terrain_regret < best_terrain_regret - eps ||
          (std::fabs(item.terrain_regret - best_terrain_regret) <= eps &&
           (item.score < best_score - eps ||
            (std::fabs(item.score - best_score) <= eps &&
             metrics[item.index].length < metrics[best].length)))))) {
      best_max_roll_deg = item.max_roll_deg;
      best_terrain_regret = item.terrain_regret;
      best_score = item.score;
      best = item.index;
    }
  }
  return best;
}

void HybridAStarPlanner::publishParetoDecision(
    const std::vector<std::vector<geometry_msgs::PoseStamped>>& candidates,
    const std::vector<PathMetrics>& metrics,
    const std::vector<int>& source_channels,
    const std::vector<std::string>& candidate_roles,
    const std::vector<bool>& on_pareto_front,
    const std::vector<double>& ideal_distances,
    const CoarseSearchAudit& coarse_audit,
    int selected,
    const std::string& mode,
    const std::string& reason,
    const std::string& frame,
    const ros::Time& stamp) const {
  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  double min_candidate_length = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < metrics.size(); ++index) {
    if (index < on_pareto_front.size() && on_pareto_front[index])
      min_candidate_length = std::min(
          min_candidate_length, metrics[index].length);
  }
  if (!std::isfinite(min_candidate_length)) {
    for (const PathMetrics& item : metrics)
      min_candidate_length = std::min(min_candidate_length, item.length);
  }
  auto unit_risk = [](double value) {
    return std::max(0.0, std::min(1.0, value));
  };
  double decision_min_general = std::numeric_limits<double>::infinity();
  double decision_min_pitch = std::numeric_limits<double>::infinity();
  double decision_min_roll = std::numeric_limits<double>::infinity();
  double decision_min_score = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < metrics.size(); ++index) {
    const bool is_front =
        index < on_pareto_front.size() && on_pareto_front[index];
    const double relative_detour =
        min_candidate_length > 1e-9
            ? std::max(0.0,
                       metrics[index].length / min_candidate_length - 1.0)
            : 0.0;
    if (is_front &&
        relative_detour <= pareto_decision_detour_ratio_ + 1e-9) {
      decision_min_general = std::min(
          decision_min_general,
          unit_risk(metrics[index].combined_risk_objective));
      decision_min_pitch = std::min(
          decision_min_pitch, unit_risk(metrics[index].pitch_objective));
      decision_min_roll = std::min(
          decision_min_roll, unit_risk(metrics[index].roll_objective));
      if (index < ideal_distances.size() &&
          std::isfinite(ideal_distances[index])) {
        decision_min_score = std::min(
            decision_min_score, ideal_distances[index]);
      }
    }
  }
  if (!std::isfinite(decision_min_general)) {
    for (const PathMetrics& item : metrics) {
      decision_min_general = std::min(
          decision_min_general, unit_risk(item.combined_risk_objective));
      decision_min_pitch = std::min(
          decision_min_pitch, unit_risk(item.pitch_objective));
      decision_min_roll = std::min(
          decision_min_roll, unit_risk(item.roll_objective));
    }
  }
  if (!std::isfinite(decision_min_general)) {
    decision_min_general = 0.0;
    decision_min_pitch = 0.0;
    decision_min_roll = 0.0;
  }

  for (size_t index = 0; index < metrics.size(); ++index) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = "pareto_candidates";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    if (index < candidates.size() && !candidates[index].empty()) {
      marker.pose.position =
          candidates[index][candidates[index].size() / 2].pose.position;
    }
    marker.pose.position.z += 0.8;
    marker.scale.z = 0.35;
    const bool is_selected = static_cast<int>(index) == selected;
    const bool is_front =
        index < on_pareto_front.size() && on_pareto_front[index];
    const bool is_zero_high_risk_exposure =
        metrics[index].high_risk_fraction <= 1e-9;
    const bool is_safety_feasible = isParetoSafetyFeasible(metrics[index]);
    const double safety_violation =
        computeParetoSafetyViolation(metrics[index]);
    marker.color.r = is_selected ? 0.85f : (is_front ? 0.05f : 0.55f);
    marker.color.g = is_selected ? 0.10f : (is_front ? 0.60f : 0.55f);
    marker.color.b = is_selected ? 0.10f : (is_front ? 0.25f : 0.55f);
    marker.color.a = 0.95f;

    const int channel = index < source_channels.size()
                            ? source_channels[index] : -1;
    const std::string role = index < candidate_roles.size()
                                 ? candidate_roles[index] : "unknown";
    const double ideal_distance =
        index < ideal_distances.size() &&
                std::isfinite(ideal_distances[index])
            ? ideal_distances[index] : -1.0;
    const double relative_detour =
        min_candidate_length > 1e-9
            ? std::max(0.0,
                       metrics[index].length / min_candidate_length - 1.0)
            : 0.0;
    const double distance_regret =
        relative_detour;
    const double general_risk_regret = std::max(
        0.0, unit_risk(metrics[index].combined_risk_objective) -
                 decision_min_general);
    const double pitch_regret = std::max(
        0.0, unit_risk(metrics[index].pitch_objective) -
                 decision_min_pitch);
    const double roll_regret = std::max(
        0.0, unit_risk(metrics[index].roll_objective) -
                 decision_min_roll);
    const double terrain_regret = std::max(
        general_risk_regret, std::max(pitch_regret, roll_regret));
    const bool decision_eligible =
        is_front &&
        relative_detour <= pareto_decision_detour_ratio_ + 1e-9;
    const bool near_tie =
        decision_eligible && ideal_distance >= 0.0 &&
        std::isfinite(decision_min_score) &&
        ideal_distance <=
            decision_min_score + pareto_score_near_tie_margin_ + 1e-9;
    std::ostringstream text;
    text << std::fixed << std::setprecision(6)
         << "{\"kind\":\"candidate\",\"index\":" << (index + 1)
         << ",\"channel\":" << channel
         << ",\"role\":\"" << role << "\""
         << ",\"length\":" << metrics[index].length
         << ",\"mean_risk\":" << metrics[index].mean_risk
         << ",\"peak_risk\":" << metrics[index].peak_risk
         << ",\"tail_risk\":" << metrics[index].tail_risk
         << ",\"high_risk_fraction\":"
         << metrics[index].high_risk_fraction
         << ",\"combined_risk_objective\":"
         << metrics[index].combined_risk_objective
         << ",\"mean_abs_pitch_deg\":"
         << metrics[index].mean_abs_pitch_deg
         << ",\"p95_abs_pitch_deg\":"
         << metrics[index].p95_abs_pitch_deg
         << ",\"max_abs_pitch_deg\":"
         << metrics[index].max_abs_pitch_deg
         << ",\"pitch_exposure_fraction\":"
         << metrics[index].pitch_exposure_fraction
         << ",\"pitch_objective\":"
         << metrics[index].pitch_objective
         << ",\"mean_abs_roll_deg\":"
         << metrics[index].mean_abs_roll_deg
         << ",\"p95_abs_roll_deg\":"
         << metrics[index].p95_abs_roll_deg
         << ",\"max_abs_roll_deg\":"
         << metrics[index].max_abs_roll_deg
         << ",\"roll_exposure_fraction\":"
         << metrics[index].roll_exposure_fraction
         << ",\"roll_objective\":"
         << metrics[index].roll_objective
         << ",\"attitude_feasible\":"
         << (metrics[index].attitude_feasible ? "true" : "false")
         << ",\"zero_high_risk_exposure\":"
         << (is_zero_high_risk_exposure ? "true" : "false")
         << ",\"safety_feasible\":"
         << (is_safety_feasible ? "true" : "false")
         << ",\"safety_violation\":" << safety_violation
         << ",\"pareto\":" << (is_front ? "true" : "false")
         << ",\"decision_eligible\":"
         << (decision_eligible ? "true" : "false")
         << ",\"near_tie\":" << (near_tie ? "true" : "false")
         << ",\"distance_regret\":" << distance_regret
         << ",\"general_risk_regret\":" << general_risk_regret
         << ",\"pitch_regret\":" << pitch_regret
         << ",\"roll_regret\":" << roll_regret
         << ",\"terrain_regret\":" << terrain_regret
         << ",\"compromise_score\":" << ideal_distance
         << ",\"ideal_distance\":" << ideal_distance
         << ",\"selected\":" << (is_selected ? "true" : "false")
         << "}";
    marker.text = text.str();
    array.markers.push_back(std::move(marker));
  }

  visualization_msgs::Marker summary;
  summary.header.frame_id = frame;
  summary.header.stamp = stamp;
  summary.ns = "pareto_summary";
  summary.id = 10000;
  summary.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  summary.action = visualization_msgs::Marker::ADD;
  summary.pose.orientation.w = 1.0;
  summary.pose.position.z = 1.0;
  summary.scale.z = 0.4;
  summary.color.r = 0.1f;
  summary.color.g = 0.1f;
  summary.color.b = 0.1f;
  summary.color.a = 0.95f;
  std::ostringstream summary_text;
  const size_t zero_exposure_candidate_count = static_cast<size_t>(
      std::count_if(
      metrics.begin(), metrics.end(), [&](const PathMetrics& item) {
        return item.high_risk_fraction <= 1e-9;
      }));
  const size_t safety_feasible_candidate_count = static_cast<size_t>(
      std::count_if(
      metrics.begin(), metrics.end(), [&](const PathMetrics& item) {
        return isParetoSafetyFeasible(item);
      }));
  size_t decision_eligible_candidate_count = 0;
  size_t near_tie_candidate_count = 0;
  for (size_t index = 0; index < metrics.size(); ++index) {
    const bool is_front =
        index < on_pareto_front.size() && on_pareto_front[index];
    const double relative_detour =
        min_candidate_length > 1e-9
            ? std::max(0.0,
                       metrics[index].length / min_candidate_length - 1.0)
            : 0.0;
    if (is_front &&
        relative_detour <= pareto_decision_detour_ratio_ + 1e-9) {
      ++decision_eligible_candidate_count;
      if (index < ideal_distances.size() &&
          std::isfinite(ideal_distances[index]) &&
          std::isfinite(decision_min_score) &&
          ideal_distances[index] <=
              decision_min_score + pareto_score_near_tie_margin_ + 1e-9) {
        ++near_tie_candidate_count;
      }
    }
  }
  const bool safety_filter_applied =
      pareto_safety_filter_enabled_ &&
      safety_feasible_candidate_count > 0 &&
      safety_feasible_candidate_count < metrics.size();
  const double selected_safety_violation =
      selected >= 0 && selected < static_cast<int>(metrics.size())
          ? computeParetoSafetyViolation(metrics[selected]) : -1.0;
  bool safety_focused_fine_used = false;
  for (size_t channel = 0;
       channel < coarse_audit.fine_channel_roles.size(); ++channel) {
    const std::string& role = coarse_audit.fine_channel_roles[channel];
    const std::string status =
        channel < coarse_audit.fine_channel_status.size()
            ? coarse_audit.fine_channel_status[channel] : "not_attempted";
    if ((role == "adaptive_attitude_safe_anchor" ||
         role == "pareto_safest_label") &&
        status != "fine_budget_pruned" &&
        status != "coarse_objective_dominated" &&
        status != "not_attempted") {
      safety_focused_fine_used = true;
      break;
    }
  }
  summary_text << "{\"kind\":\"summary\",\"mode\":\"" << mode
                << "\",\"selected\":" << (selected + 1)
                << ",\"candidate_count\":" << metrics.size()
                << ",\"terrain_risk_soft_objective\":true"
                << ",\"pareto_selector\":"
                   "\"bounded_groupwise_chebyshev_near_tie_risk_priority\""
                << ",\"decision_detour_bound\":"
                << pareto_decision_detour_ratio_
                << ",\"score_near_tie_margin\":"
                << pareto_score_near_tie_margin_
                << ",\"decision_eligible_candidate_count\":"
                << decision_eligible_candidate_count
                << ",\"near_tie_candidate_count\":"
                << near_tie_candidate_count
                << ",\"zero_exposure_candidate_count\":"
                 << zero_exposure_candidate_count
                << ",\"safety_filter_enabled\":"
                << (pareto_safety_filter_enabled_ ? "true" : "false")
                << ",\"safety_filter_applied\":"
                << (safety_filter_applied ? "true" : "false")
                << ",\"safety_feasible_candidate_count\":"
                << safety_feasible_candidate_count
                << ",\"selected_safety_violation\":"
                << selected_safety_violation
                << ",\"max_high_risk_fraction\":"
                << pareto_max_high_risk_fraction_
                << ",\"peak_risk_limit\":"
                << std::min(1.0, pareto_high_risk_threshold_ +
                                     pareto_peak_risk_margin_)
                << ",\"tail_risk_limit\":"
                << std::min(1.0, pareto_high_risk_threshold_ +
                                     pareto_tail_risk_margin_)
                << ",\"coarse_channel_count\":"
                << coarse_audit.accepted_channels
                << ",\"coarse_max_detour_ratio\":"
                << coarse_max_detour_ratio_
                << ",\"coarse_min_separation_m\":"
                << coarse_min_hausdorff_
                << ",\"coarse_min_separated_length_m\":"
                << coarse_min_separated_length_
                << ",\"coarse_pool_extra_candidates\":"
                << coarse_pool_extra_candidates_
                << ",\"risk_anchor_status\":\""
                << coarse_audit.risk_anchor_status << "\""
                << ",\"risk_anchor_attempts\":"
                << coarse_audit.risk_anchor_attempts
                << ",\"exposure_anchor_status\":\""
                << coarse_audit.exposure_anchor_status << "\""
                << ",\"exposure_anchor_attempts\":"
                << coarse_audit.exposure_anchor_attempts
                << ",\"additional_attempts\":"
                << coarse_audit.additional_attempts
                << ",\"failed_attempts\":"
                << coarse_audit.failed_attempts
                << ",\"duplicate_rejections\":"
                << coarse_audit.duplicate_rejections
                << ",\"detour_rejections\":"
                << coarse_audit.detour_rejections
                << ",\"generated_pool_count\":"
                << coarse_audit.generated_pool_count
                 << ",\"pruned_pool_count\":"
                 << coarse_audit.pruned_pool_count
                 << ",\"coarse_multi_label_enabled\":"
                 << (coarse_multi_label_enabled_ ? "true" : "false")
                 << ",\"label_generated\":"
                 << coarse_audit.label_generated
                 << ",\"label_dominance_pruned\":"
                 << coarse_audit.label_dominance_pruned
                 << ",\"label_cap_pruned\":"
                 << coarse_audit.label_cap_pruned
                 << ",\"goal_labels\":"
                 << coarse_audit.goal_labels
                 << ",\"goal_topology_count\":"
                 << coarse_audit.goal_topology_count
                << ",\"goal_signature_count\":"
                 << coarse_audit.goal_signature_count
                << ",\"fine_refinement_limit\":"
                 << coarse_audit.fine_refinement_limit
                << ",\"fine_refinement_selected\":"
                 << coarse_audit.fine_refinement_selected
                << ",\"fine_budget_pruned\":"
                 << coarse_audit.fine_budget_pruned
                << ",\"topology_signature_enabled\":"
                 << (coarse_topology_signature_enabled_ ? "true" : "false")
                << ",\"topology_gate_first_ratio\":"
                 << coarse_topology_gate_first_ratio_
                << ",\"topology_gate_second_ratio\":"
                 << coarse_topology_gate_second_ratio_
                << ",\"topology_lateral_bin_width\":"
                 << coarse_topology_lateral_bin_width_
                << ",\"topology_signature_labels_admitted\":"
                 << coarse_audit.topology_signature_labels_admitted
                << ",\"topology_signature_cap_protected\":"
                 << coarse_audit.topology_signature_cap_protected
                << ",\"coarse_stop_reason\":\""
                << coarse_audit.coarse_stop_reason << "\""
                << ",\"coarse_stop_iteration\":"
                << coarse_audit.coarse_stop_iteration
                 << ",\"dominated_fine_rejections\":"
                 << coarse_audit.dominated_fine_rejections
                 << ",\"dominated_fine_candidates\":"
                 << coarse_audit.dominated_fine_rejections
                 << ",\"fine_geometric_similarity_matches\":"
                 << coarse_audit.fine_geometric_similarity_matches
                 << ",\"fine_objective_equivalent_rejections\":"
                 << coarse_audit.fine_objective_equivalent_rejections
                 << ",\"fine_same_topology_dominated_rejections\":"
                 << coarse_audit.fine_same_topology_dominated_rejections
                 << ",\"fine_same_topology_tradeoffs\":"
                 << coarse_audit.fine_same_topology_tradeoffs
                 << ",\"fine_same_topology_replacements\":"
                 << coarse_audit.fine_same_topology_replacements
                 << ",\"smoothed_fine_candidates\":"
                 << coarse_audit.smoothed_fine_candidates
                << ",\"raw_fine_fallbacks\":"
                << coarse_audit.raw_fine_fallbacks
                << ",\"attitude_fine_fallbacks\":"
                << coarse_audit.attitude_fine_fallbacks
                << ",\"adaptive_safe_status\":\""
                << coarse_audit.adaptive_safe_status << "\""
                << ",\"adaptive_safe_trigger\":\""
                << coarse_audit.adaptive_safe_trigger << "\""
                << ",\"adaptive_safe_attempts\":"
                << coarse_audit.adaptive_safe_attempts
                << ",\"adaptive_safe_candidates\":"
                << coarse_audit.adaptive_safe_candidates
                << ",\"adaptive_safe_generation_detour_limit\":"
                << coarse_audit.adaptive_safe_generation_detour_limit
                << ",\"adaptive_safe_reference_length_m\":"
                << coarse_audit.adaptive_safe_reference_length_m
                << ",\"adaptive_safe_reference_mean_risk\":"
                << coarse_audit.adaptive_safe_reference_mean_risk
                << ",\"adaptive_safe_reference_combined_risk\":"
                << coarse_audit.adaptive_safe_reference_combined_risk
                << ",\"adaptive_safe_reference_peak_risk\":"
                << coarse_audit.adaptive_safe_reference_peak_risk
                << ",\"adaptive_safe_reference_pitch_objective\":"
                << coarse_audit.adaptive_safe_reference_pitch_objective
                << ",\"adaptive_safe_reference_roll_objective\":"
                << coarse_audit.adaptive_safe_reference_roll_objective
                << ",\"adaptive_safe_reference_max_pitch_deg\":"
                << coarse_audit.adaptive_safe_reference_max_pitch_deg
                << ",\"adaptive_safe_reference_max_roll_deg\":"
                << coarse_audit.adaptive_safe_reference_max_roll_deg
                << ",\"adaptive_safe_candidate_length_m\":"
                << coarse_audit.adaptive_safe_candidate_length_m
                << ",\"adaptive_safe_candidate_detour_ratio\":"
                << coarse_audit.adaptive_safe_candidate_detour_ratio
                << ",\"adaptive_safe_candidate_mean_risk\":"
                << coarse_audit.adaptive_safe_candidate_mean_risk
                << ",\"adaptive_safe_candidate_combined_risk\":"
                << coarse_audit.adaptive_safe_candidate_combined_risk
                << ",\"adaptive_safe_candidate_peak_risk\":"
                << coarse_audit.adaptive_safe_candidate_peak_risk
                << ",\"adaptive_safe_candidate_pitch_objective\":"
                << coarse_audit.adaptive_safe_candidate_pitch_objective
                << ",\"adaptive_safe_candidate_roll_objective\":"
                << coarse_audit.adaptive_safe_candidate_roll_objective
                << ",\"adaptive_safe_candidate_max_pitch_deg\":"
                << coarse_audit.adaptive_safe_candidate_max_pitch_deg
                << ",\"adaptive_safe_candidate_max_roll_deg\":"
                << coarse_audit.adaptive_safe_candidate_max_roll_deg
                << ",\"adaptive_safe_coarse_length_bound_m\":"
                << coarse_audit.adaptive_safe_coarse_length_bound_m
                << ",\"adaptive_safe_fine_length_bound_m\":"
                << coarse_audit.adaptive_safe_fine_length_bound_m
                << ",\"adaptive_safe_coarse_length_pruned\":"
                << coarse_audit.adaptive_safe_coarse_length_pruned
                << ",\"adaptive_safe_fine_length_pruned\":"
                << coarse_audit.adaptive_safe_fine_length_pruned
                << ",\"fine_topology_effective_weight\":"
                << ((pareto_terrain_mode_ &&
                     coarse_audit.accepted_channels > 1)
                        ? (safety_focused_fine_used
                               ? adaptive_safe_topology_preservation_weight_
                               : fine_topology_preservation_weight_)
                        : 0.0)
                << ",\"fine_channel_audit\":[";
  for (size_t channel = 0;
       channel < coarse_audit.fine_channel_status.size(); ++channel) {
    if (channel > 0) summary_text << ",";
    const std::string role =
        channel < coarse_audit.fine_channel_roles.size()
            ? coarse_audit.fine_channel_roles[channel] : "unknown";
    const double channel_topology_weight =
        pareto_terrain_mode_ && coarse_audit.accepted_channels > 1
            ? (role == "adaptive_attitude_safe_anchor" ||
                       role == "pareto_safest_label"
                   ? adaptive_safe_topology_preservation_weight_
                   : fine_topology_preservation_weight_)
            : 0.0;
    summary_text << "{\"channel\":" << (channel + 1)
                 << ",\"role\":\"" << role << "\""
                 << ",\"topology_weight\":"
                 << channel_topology_weight
                 << ",\"status\":\""
                 << coarse_audit.fine_channel_status[channel] << "\"}";
  }
  summary_text << "]"
                << ",\"coarse_time_budget_s\":"
                << coarse_audit.time_budget_s
                << ",\"coarse_elapsed_s\":"
                << coarse_audit.elapsed_s
                << ",\"pareto_decision_detour_ratio\":"
                << pareto_decision_detour_ratio_
                << ",\"pareto_score_near_tie_margin\":"
                << pareto_score_near_tie_margin_
                << ",\"high_risk_threshold\":"
                << pareto_high_risk_threshold_
                << ",\"tail_fraction\":" << pareto_tail_fraction_
                << ",\"pointwise_risk_exponent\":"
                << pointwise_risk_exponent_
                << ",\"fine_risk_strength\":" << fine_risk_strength_
                << ",\"fine_heuristic_weight\":"
                << fine_heuristic_weight_
                << ",\"fine_stagnation_min_iterations\":"
                << fine_stagnation_min_iterations_
                << ",\"fine_stagnation_window_iterations\":"
                << fine_stagnation_window_iterations_
                << ",\"fine_stagnation_progress_m\":"
                << fine_stagnation_progress_m_
                << ",\"primitive_collision_check_fraction\":"
                << primitive_collision_check_fraction_
                << ",\"attitude_risk_enabled\":"
                << (attitude_risk_enabled_ ? "true" : "false")
                << ",\"pitch_soft_start_deg\":"
                << pitch_soft_start_rad_ * 180.0 / M_PI
                << ",\"roll_soft_start_deg\":"
                << roll_soft_start_rad_ * 180.0 / M_PI
                << ",\"pitch_hard_limit_deg\":"
                << pitch_hard_limit_rad_ * 180.0 / M_PI
                << ",\"roll_hard_limit_deg\":"
                << roll_hard_limit_rad_ * 180.0 / M_PI
                << ",\"pitch_risk_weight\":" << pitch_risk_weight_
                << ",\"roll_risk_weight\":" << roll_risk_weight_
                << ",\"adaptive_safe_candidate_enabled\":"
                << (adaptive_safe_candidate_enabled_ ? "true" : "false")
                << ",\"adaptive_safe_peak_risk_trigger\":"
                << adaptive_safe_peak_risk_trigger_
                << ",\"adaptive_safe_effective_peak_trigger\":"
                << coarse_audit.adaptive_safe_effective_peak_trigger
                << ",\"adaptive_safe_peak_quantile_margin\":"
                << adaptive_safe_peak_quantile_margin_
                << ",\"adaptive_safe_pitch_trigger_deg\":"
                << adaptive_safe_pitch_trigger_rad_ * 180.0 / M_PI
                << ",\"adaptive_safe_roll_trigger_deg\":"
                << adaptive_safe_roll_trigger_rad_ * 180.0 / M_PI
                << ",\"adaptive_safe_risk_scale\":"
                << adaptive_safe_risk_scale_
                << ",\"adaptive_safe_attitude_scale\":"
                << adaptive_safe_attitude_scale_
                << ",\"fine_mean_risk_weight\":"
                << coarse_audit.fine_mean_risk_weight
                << ",\"fine_high_risk_weight\":"
                << coarse_audit.fine_high_risk_weight
                << ",\"fine_pointwise_risk_weight\":"
                << coarse_audit.fine_pointwise_risk_weight
                << ",\"high_risk_quantile\":"
                << pareto_high_risk_quantile_
                << ",\"corridor_max_width_m\":" << corridor_width_
                << ",\"corridor_min_width_m\":" << corridor_min_width_
                << ",\"corridor_risk_margin\":" << corridor_risk_margin_
                << ",\"corridor_center_weight\":"
                << corridor_center_weight_
                << ",\"fine_topology_preservation_weight\":"
                << fine_topology_preservation_weight_
                << ",\"adaptive_safe_topology_preservation_weight\":"
                << adaptive_safe_topology_preservation_weight_
                << ",\"fine_min_hausdorff\":"
                << fine_min_hausdorff_
                << ",\"fine_max_overlap\":"
                << fine_max_overlap_
                << ",\"reason\":\"" << reason << "\"}";
  summary.text = summary_text.str();
  array.markers.push_back(std::move(summary));

  pareto_decision_pub_.publish(array);
}

// ==================== Corridor for two-level search ====================

void HybridAStarPlanner::buildCorridorMask(
    const std::vector<geometry_msgs::PoseStamped>& path) {
  unsigned int w = costmap_->getSizeInCellsX();
  unsigned int h = costmap_->getSizeInCellsY();
  corridor_mask_.assign(w * h, false);
  corridor_center_distance_field_.assign(
      w * h, std::numeric_limits<float>::infinity());

  double res = costmap_->getResolution();
  int r = (int)ceil(corridor_width_ / res);
  double sum_left_width = 0.0;
  double sum_right_width = 0.0;
  int width_samples = 0;

  for (const auto& ps : path) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my))
      continue;
    const double theta = tf2::getYaw(ps.pose.orientation);
    const double normal_x = -std::sin(theta);
    const double normal_y =  std::cos(theta);
    const double center_risk =
        active_map_ && active_map_->loaded
            ? queryTerrainCost(ps.pose.position.x,
                               ps.pose.position.y, theta)
            : 0.0;

    auto available_side_width = [&](double side) {
      double available = corridor_min_width_;
      for (double distance = res;
           distance <= corridor_width_ + 0.5 * res;
           distance += res) {
        const double wx =
            ps.pose.position.x + side * normal_x * distance;
        const double wy =
            ps.pose.position.y + side * normal_y * distance;
        unsigned int side_mx, side_my;
        if (!costmap_->worldToMap(wx, wy, side_mx, side_my)) break;
        const size_t map_index =
            static_cast<size_t>(side_my * w + side_mx);
        if (!clearance_map_.empty() &&
            (map_index >= clearance_map_.size() ||
             !clearance_map_[map_index])) {
          break;
        }

        if (active_map_ && active_map_->loaded &&
            distance > corridor_min_width_) {
          const double side_risk = queryTerrainCost(wx, wy, theta);
          const double allowed_risk =
              std::min(1.0, center_risk + corridor_risk_margin_);
          if (side_risk > allowed_risk) break;
        }
        available = distance;
      }
      return std::max(corridor_min_width_,
                      std::min(corridor_width_, available));
    };

    const double left_width = available_side_width(1.0);
    const double right_width = available_side_width(-1.0);
    sum_left_width += left_width;
    sum_right_width += right_width;
    ++width_samples;

    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const double offset_x = dx * res;
        const double offset_y = dy * res;
        const double distance = std::hypot(offset_x, offset_y);
        const double lateral =
            normal_x * offset_x + normal_y * offset_y;
        const double local_width =
            lateral >= 0.0 ? left_width : right_width;
        if (distance > local_width) continue;
        int nx = (int)mx + dx, ny = (int)my + dy;
        if (nx >= 0 && ny >= 0 && nx < (int)w && ny < (int)h) {
          const size_t index = static_cast<size_t>(ny * w + nx);
          corridor_mask_[index] = true;
          corridor_center_distance_field_[index] = std::min(
              corridor_center_distance_field_[index],
              static_cast<float>(distance));
        }
      }
    }
  }

  // Risk-adaptive side widths are desirable along the route, but near the
  // goal they can shrink the corridor to corridor_min_width_ and remove the
  // lateral room needed to rotate the arrival tangent.  Preserve a bounded,
  // collision-checked terminal manoeuvre region over the last approach.  Its
  // radius follows the vehicle kinematics (turning diameter), while never
  // exceeding the configured corridor width; terrain remains a soft cost in
  // the subsequent fine search rather than silently becoming a hard wall.
  const double terminal_manoeuvre_radius = std::min(
      corridor_width_,
      std::max(corridor_min_width_, 2.0 * min_turning_radius_));
  const double terminal_widen_length =
      terminal_approach_distance_ + 2.0 * min_turning_radius_;
  const int terminal_radius_cells = static_cast<int>(
      std::ceil(terminal_manoeuvre_radius / res));
  double distance_from_end = 0.0;
  int widened_terminal_samples = 0;
  for (size_t reverse_index = path.size(); reverse_index-- > 0;) {
    if (reverse_index + 1 < path.size()) {
      distance_from_end += std::hypot(
          path[reverse_index + 1].pose.position.x -
              path[reverse_index].pose.position.x,
          path[reverse_index + 1].pose.position.y -
              path[reverse_index].pose.position.y);
    }
    if (distance_from_end > terminal_widen_length) break;

    unsigned int center_x, center_y;
    if (!costmap_->worldToMap(path[reverse_index].pose.position.x,
                              path[reverse_index].pose.position.y,
                              center_x, center_y)) {
      continue;
    }
    ++widened_terminal_samples;
    for (int dy = -terminal_radius_cells;
         dy <= terminal_radius_cells; ++dy) {
      for (int dx = -terminal_radius_cells;
           dx <= terminal_radius_cells; ++dx) {
        const double distance = std::hypot(dx * res, dy * res);
        if (distance > terminal_manoeuvre_radius) continue;
        const int nx = static_cast<int>(center_x) + dx;
        const int ny = static_cast<int>(center_y) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(w) ||
            ny >= static_cast<int>(h)) {
          continue;
        }
        const size_t index = static_cast<size_t>(ny * w + nx);
        if (!clearance_map_.empty() && !clearance_map_[index]) continue;
        corridor_mask_[index] = true;
        corridor_center_distance_field_[index] = std::min(
            corridor_center_distance_field_[index],
            static_cast<float>(distance));
      }
    }
  }

  if (width_samples > 0) {
    ROS_INFO("Adaptive corridor: max=%.2fm min=%.2fm "
             "mean_left=%.2fm mean_right=%.2fm risk_margin=%.2f "
             "terminal_radius=%.2fm terminal_samples=%d",
             corridor_width_, corridor_min_width_,
             sum_left_width / width_samples,
             sum_right_width / width_samples,
             corridor_risk_margin_, terminal_manoeuvre_radius,
             widened_terminal_samples);
  }
}

bool HybridAStarPlanner::isInCorridor(double x, double y) const {
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return false;
  unsigned int w = costmap_->getSizeInCellsX();
  return corridor_mask_[my * w + mx];
}

double HybridAStarPlanner::queryCorridorCenterDistance(
    double x, double y) const {
  if (corridor_center_distance_field_.empty()) return 0.0;
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return corridor_width_;
  const size_t index = static_cast<size_t>(
      my * costmap_->getSizeInCellsX() + mx);
  if (index >= corridor_center_distance_field_.size() ||
      !std::isfinite(corridor_center_distance_field_[index])) {
    return corridor_width_;
  }
  return corridor_center_distance_field_[index];
}

void HybridAStarPlanner::clearDiversityPenalty() {
  diversity_penalty_field_.clear();
  if (!coarse_map_.loaded) return;
  diversity_penalty_field_.assign(
      coarse_map_.rows * coarse_map_.cols, 0.0f);
}

void HybridAStarPlanner::addDiversityPenalty(
    const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!coarse_map_.loaded || path.empty()) return;
  if (diversity_penalty_field_.size() !=
      static_cast<size_t>(coarse_map_.rows * coarse_map_.cols)) {
    clearDiversityPenalty();
  }

  const int radius_cells = std::max(
      1, static_cast<int>(
             std::ceil(coarse_diversity_radius_ / coarse_map_.res)));
  const double radius_sq =
      static_cast<double>(radius_cells * radius_cells);

  for (const auto& pose : path) {
    const int center_col = static_cast<int>(std::floor(
        (pose.pose.position.x - coarse_map_.origin_x) /
        coarse_map_.res));
    const int center_row = static_cast<int>(std::floor(
        (pose.pose.position.y - coarse_map_.origin_y) /
        coarse_map_.res));
    if (center_col < 0 || center_row < 0 ||
        center_col >= coarse_map_.cols ||
        center_row >= coarse_map_.rows) {
      continue;
    }

    for (int row_offset = -radius_cells;
         row_offset <= radius_cells; ++row_offset) {
      for (int col_offset = -radius_cells;
           col_offset <= radius_cells; ++col_offset) {
        const double distance_sq =
            col_offset * col_offset + row_offset * row_offset;
        if (distance_sq > radius_sq) continue;
        const int col = center_col + col_offset;
        const int row = center_row + row_offset;
        if (col < 0 || row < 0 ||
            col >= coarse_map_.cols ||
            row >= coarse_map_.rows) {
          continue;
        }

        // Smooth compact support: 1.0 on the accepted path and 0.0 at the
        // diversity radius. Accumulation lets several accepted channels
        // discourage their shared start/goal prefix without creating a hard
        // obstacle that could destroy reachability.
        const float contribution = static_cast<float>(
            std::max(0.0, 1.0 - std::sqrt(distance_sq / radius_sq)));
        const size_t index =
            static_cast<size_t>(row * coarse_map_.cols + col);
        diversity_penalty_field_[index] =
            std::min(1.0f,
                     diversity_penalty_field_[index] + contribution);
      }
    }
  }
}

double HybridAStarPlanner::queryDiversityPenalty(
    double x, double y) const {
  if (!active_map_ || active_map_ != &coarse_map_ ||
      diversity_penalty_field_.size() !=
          static_cast<size_t>(coarse_map_.rows * coarse_map_.cols)) {
    return 0.0;
  }

  const int col = static_cast<int>(
      std::floor((x - coarse_map_.origin_x) / coarse_map_.res));
  const int row = static_cast<int>(
      std::floor((y - coarse_map_.origin_y) / coarse_map_.res));
  if (col < 0 || row < 0 ||
      col >= coarse_map_.cols || row >= coarse_map_.rows) {
    return 0.0;
  }
  return diversity_penalty_field_[
      static_cast<size_t>(row * coarse_map_.cols + col)];
}

double HybridAStarPlanner::pathHausdorffDistance(
    const std::vector<geometry_msgs::PoseStamped>& first,
    const std::vector<geometry_msgs::PoseStamped>& second) const {
  if (first.empty() || second.empty())
    return std::numeric_limits<double>::infinity();

  auto directed_distance =
      [](const std::vector<geometry_msgs::PoseStamped>& source,
         const std::vector<geometry_msgs::PoseStamped>& target) {
        const size_t source_stride =
            std::max<size_t>(1, source.size() / 100);
        const size_t target_stride =
            std::max<size_t>(1, target.size() / 100);
        double maximum = 0.0;
        for (size_t i = 0; i < source.size(); i += source_stride) {
          double nearest = std::numeric_limits<double>::infinity();
          for (size_t j = 0; j < target.size(); j += target_stride) {
            const double dx =
                source[i].pose.position.x - target[j].pose.position.x;
            const double dy =
                source[i].pose.position.y - target[j].pose.position.y;
            nearest = std::min(nearest, std::hypot(dx, dy));
          }
          maximum = std::max(maximum, nearest);
        }
        return maximum;
      };

  return std::max(directed_distance(first, second),
                  directed_distance(second, first));
}

double HybridAStarPlanner::pathOverlapRatio(
    const std::vector<geometry_msgs::PoseStamped>& first,
    const std::vector<geometry_msgs::PoseStamped>& second) const {
  if (first.empty() || second.empty()) return 0.0;

  auto directed_overlap =
      [this](const std::vector<geometry_msgs::PoseStamped>& source,
             const std::vector<geometry_msgs::PoseStamped>& target) {
        const size_t source_stride =
            std::max<size_t>(1, source.size() / 100);
        const size_t target_stride =
            std::max<size_t>(1, target.size() / 100);
        int samples = 0;
        int overlapping = 0;
        for (size_t i = 0; i < source.size(); i += source_stride) {
          double nearest = std::numeric_limits<double>::infinity();
          for (size_t j = 0; j < target.size(); j += target_stride) {
            const double dx =
                source[i].pose.position.x - target[j].pose.position.x;
            const double dy =
                source[i].pose.position.y - target[j].pose.position.y;
            nearest = std::min(nearest, std::hypot(dx, dy));
          }
          ++samples;
          if (nearest <= coarse_diversity_radius_) ++overlapping;
        }
        return samples > 0
                   ? static_cast<double>(overlapping) / samples
                   : 0.0;
      };

  // Symmetric overlap prevents a short sub-path from being classified as a
  // new channel merely because it covers only part of a longer path.
  return 0.5 * (directed_overlap(first, second) +
                directed_overlap(second, first));
}

double HybridAStarPlanner::pathLongestSeparatedLength(
    const std::vector<geometry_msgs::PoseStamped>& first,
    const std::vector<geometry_msgs::PoseStamped>& second,
    double separation_threshold) const {
  if (first.size() < 2 || second.empty()) return 0.0;

  auto directed_length = [separation_threshold](
      const std::vector<geometry_msgs::PoseStamped>& source,
      const std::vector<geometry_msgs::PoseStamped>& target) {
    const size_t source_stride = std::max<size_t>(1, source.size() / 160);
    const size_t target_stride = std::max<size_t>(1, target.size() / 160);
    double longest = 0.0;
    double current = 0.0;
    bool previous_separated = false;
    size_t previous_index = 0;
    size_t source_index = 0;
    while (source_index < source.size()) {
      double nearest = std::numeric_limits<double>::infinity();
      size_t target_index = 0;
      while (target_index < target.size()) {
        const double dx = source[source_index].pose.position.x -
                          target[target_index].pose.position.x;
        const double dy = source[source_index].pose.position.y -
                          target[target_index].pose.position.y;
        nearest = std::min(nearest, std::hypot(dx, dy));
        if (target_index == target.size() - 1) break;
        target_index = std::min(
            target.size() - 1, target_index + target_stride);
      }

      const bool separated = nearest >= separation_threshold;
      if (separated && previous_separated) {
        current += std::hypot(
            source[source_index].pose.position.x -
                source[previous_index].pose.position.x,
            source[source_index].pose.position.y -
                source[previous_index].pose.position.y);
      } else if (!separated) {
        current = 0.0;
      }
      longest = std::max(longest, current);
      previous_separated = separated;
      previous_index = source_index;
      if (source_index == source.size() - 1) break;
      source_index = std::min(
          source.size() - 1, source_index + source_stride);
    }
    return longest;
  };

  // Use the larger directed result. A useful detour may add a loop or bypass
  // segment to only one path, while the other path remains a shorter subset.
  return std::max(directed_length(first, second),
                  directed_length(second, first));
}

bool HybridAStarPlanner::isDistinctPath(
    const std::vector<geometry_msgs::PoseStamped>& candidate,
    const std::vector<std::vector<geometry_msgs::PoseStamped>>&
        accepted_paths) const {
  for (const auto& accepted : accepted_paths) {
    const double hausdorff =
        pathHausdorffDistance(candidate, accepted);
    const double separated_length = pathLongestSeparatedLength(
        candidate, accepted, coarse_min_hausdorff_);
    // Shared start/goal segments and long common terrain sections are normal.
    // A route is a separate channel when it sustains a metre-scale divergence
    // for a useful continuous segment.  Hausdorff rejects exact copies, while
    // the length condition prevents one noisy point or tiny bulge from
    // manufacturing an artificial fourth channel.
    const bool distinct =
        hausdorff >= coarse_min_hausdorff_ &&
        separated_length >= coarse_min_separated_length_;
    if (!distinct) return false;
  }
  return true;
}

void HybridAStarPlanner::publishPathMarkers(
    const ros::Publisher& publisher,
    const std::vector<std::vector<geometry_msgs::PoseStamped>>& paths,
    const std::string& marker_namespace,
    const std::string& frame,
    const ros::Time& stamp,
    double line_width,
    double alpha) const {
  visualization_msgs::MarkerArray array;

  // Clear the latched result from the previous navigation request.
  visualization_msgs::Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  const std::array<std::array<float, 3>, 6> colors = {{
      {{0.000f, 0.341f, 0.851f}},
      {{0.902f, 0.294f, 0.235f}},
      {{0.000f, 0.620f, 0.451f}},
      {{0.580f, 0.404f, 0.741f}},
      {{0.902f, 0.624f, 0.000f}},
      {{0.337f, 0.706f, 0.914f}}
  }};

  for (size_t index = 0; index < paths.size(); ++index) {
    if (paths[index].empty()) continue;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_width;
    const auto& color = colors[index % colors.size()];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = alpha;
    marker.points.reserve(paths[index].size());
    for (const auto& pose : paths[index]) {
      geometry_msgs::Point point = pose.pose.position;
      point.z += 0.08;
      marker.points.push_back(point);
    }
    array.markers.push_back(std::move(marker));
  }
  publisher.publish(array);
}

void HybridAStarPlanner::publishCorridorMarkers(
    const std::vector<std::vector<bool>>& corridor_masks,
    const std::string& frame,
    const ros::Time& stamp) const {
  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker clear;
  clear.header.frame_id = frame;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  if (!costmap_) {
    corridor_paths_pub_.publish(array);
    return;
  }

  const std::array<std::array<float, 3>, 6> colors = {{
      {{0.000f, 0.341f, 0.851f}},
      {{0.902f, 0.294f, 0.235f}},
      {{0.000f, 0.620f, 0.451f}},
      {{0.580f, 0.404f, 0.741f}},
      {{0.902f, 0.624f, 0.000f}},
      {{0.337f, 0.706f, 0.914f}}
  }};
  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const size_t expected_size = static_cast<size_t>(width) * height;
  const double cell_size = costmap_->getResolution();

  for (size_t channel = 0; channel < corridor_masks.size(); ++channel) {
    if (corridor_masks[channel].size() != expected_size) continue;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = stamp;
    marker.ns = "adaptive_corridors";
    marker.id = static_cast<int>(channel);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = cell_size;
    marker.scale.y = cell_size;
    const auto& color = colors[channel % colors.size()];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 0.16f;

    for (unsigned int my = 0; my < height; ++my) {
      for (unsigned int mx = 0; mx < width; ++mx) {
        const size_t index = static_cast<size_t>(my) * width + mx;
        if (!corridor_masks[channel][index]) continue;
        geometry_msgs::Point point;
        costmap_->mapToWorld(mx, my, point.x, point.y);
        point.z = getDEMZ(point.x, point.y) + 0.04;
        marker.points.push_back(point);
      }
    }
    array.markers.push_back(std::move(marker));
  }
  corridor_paths_pub_.publish(array);
}

// ==================== Costmap queries ====================

bool HybridAStarPlanner::isValid(double x, double y) const {
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return false;
  unsigned char c = costmap_->getCost(mx, my);
  if (c == costmap_2d::NO_INFORMATION) return true;
  return c < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool HybridAStarPlanner::isPathCollisionFree(double x0, double y0,
                                              double x1, double y1) const {
  double dx = x1 - x0, dy = y1 - y0;
  double dist = sqrt(dx*dx + dy*dy);
  double res = costmap_->getResolution();
  int steps = std::max(1, (int)ceil(dist / (res * 0.5)));
  for (int i = 0; i <= steps; ++i) {
    double t = (double)i / steps;
    // A centre-line that merely avoids lethal cells is not safe for the
    // 2.00 m x 1.20 m tracked vehicle.  Use the same inflated-clearance and
    // active-corridor constraints as motion-primitive expansion, otherwise a
    // terminal/analytic connector can undo the safety guaranteed by search.
    if (!isTraversable(x0 + t*dx, y0 + t*dy)) return false;
  }
  return true;
}

double HybridAStarPlanner::getCellCost(double x, double y) const {
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) return 255.0;
  unsigned char c = costmap_->getCost(mx, my);
  if (c == costmap_2d::NO_INFORMATION) return 0.0;
  return (double)c;
}

void HybridAStarPlanner::buildClearanceMap() {
  cm_width_ = costmap_->getSizeInCellsX();
  cm_height_ = costmap_->getSizeInCellsY();
  clearance_map_.assign(cm_width_ * cm_height_, true);

  if (min_clearance_ <= 0) return;

  double res = costmap_->getResolution();
  int r = (int)ceil(min_clearance_ / res);
  int blocked = 0;

  for (unsigned int j = 0; j < cm_height_; ++j) {
    for (unsigned int i = 0; i < cm_width_; ++i) {
      unsigned char c = costmap_->getCost(i, j);
      if (c == costmap_2d::LETHAL_OBSTACLE) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy > r * r) continue;
            int nx = (int)i + dx, ny = (int)j + dy;
            if (nx >= 0 && ny >= 0 && nx < (int)cm_width_ && ny < (int)cm_height_) {
              int idx = ny * cm_width_ + nx;
              if (clearance_map_[idx]) { clearance_map_[idx] = false; ++blocked; }
            }
          }
        }
      }
    }
  }
  ROS_INFO("Clearance map: radius=%.2fm, blocked %d cells", min_clearance_, blocked);
}

bool HybridAStarPlanner::isTraversable(double x, double y) const {
  ++benchmark_collision_checks_;
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    ++benchmark_collision_rejections_;
    return false;
  }
  const size_t index = static_cast<size_t>(my) * cm_width_ + mx;
  // `isInCorridor()` performs the same worldToMap conversion.  Motion
  // primitive expansion calls this function millions of times, so doing the
  // conversion twice dominated wall time even after shared multi-label search
  // reduced the number of expanded nodes.  Resolve the cell once and apply
  // both invariant binary masks to that same index.
  if (use_corridor_ &&
      (index >= corridor_mask_.size() || !corridor_mask_[index])) {
    ++benchmark_collision_rejections_;
    return false;
  }
  const bool traversable = !clearance_map_.empty()
      ? index < clearance_map_.size() &&
            static_cast<bool>(clearance_map_[index])
      : isValid(x, y);
  if (!traversable) ++benchmark_collision_rejections_;
  return traversable;
}

double HybridAStarPlanner::heuristic(double x, double y,
                                      double gx, double gy) const {
  return sqrt((gx-x)*(gx-x) + (gy-y)*(gy-y));
}

void HybridAStarPlanner::buildRiskHeuristicField(double gx, double gy) {
  if (!use_multi_heuristic_search_ ||
      !active_map_ || !active_map_->loaded) {
    risk_heuristic_field_.clear();
    return;
  }

  const int width = active_map_->cols;
  const int height = active_map_->rows;
  const double infinity = std::numeric_limits<double>::infinity();
  risk_heuristic_field_.assign(width * height, infinity);
  const int goal_x = static_cast<int>(
      std::floor((gx - active_map_->origin_x) / active_map_->res));
  const int goal_y = static_cast<int>(
      std::floor((gy - active_map_->origin_y) / active_map_->res));
  if (goal_x < 0 || goal_y < 0 ||
      goal_x >= width || goal_y >= height) return;

  using GridEntry = std::pair<double, int>;
  std::priority_queue<GridEntry, std::vector<GridEntry>,
                      std::greater<GridEntry>> queue;
  const int goal_index = goal_y * width + goal_x;
  risk_heuristic_field_[goal_index] = 0.0;
  queue.push({0.0, goal_index});

  static const int offsets[8][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1},
      {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  while (!queue.empty()) {
    const GridEntry entry = queue.top();
    queue.pop();
    const double current_cost = entry.first;
    const int index = entry.second;
    if (current_cost > risk_heuristic_field_[index] + 1e-9) continue;
    const int x = index % width;
    const int y = index / width;

    for (const auto& offset : offsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (nx < 0 || ny < 0 ||
          nx >= width || ny >= height) continue;
      const int neighbor_index = ny * width + nx;

      const double wx =
          active_map_->origin_x + (nx + 0.5) * active_map_->res;
      const double wy =
          active_map_->origin_y + (ny + 0.5) * active_map_->res;
      unsigned int map_x, map_y;
      if (!costmap_->worldToMap(wx, wy, map_x, map_y)) continue;
      if (!clearance_map_.empty() &&
          !clearance_map_[
              static_cast<size_t>(map_y * cm_width_ + map_x)]) {
        continue;
      }

      const double heading =
          std::atan2(y - ny, x - nx);
      const double distance =
          active_map_->res *
          ((offset[0] != 0 && offset[1] != 0)
               ? std::sqrt(2.0) : 1.0);
      const double risk = queryTerrainCost(wx, wy, heading);
      // The goal-conditioned risk field is a reusable search guide, not the
      // actual objective. Keep it independent of the candidate-specific
      // terrain weight so all coarse-channel searches can share one field.
      const double transition =
          distance * (1.0 + risk);
      const double candidate = current_cost + transition;
      if (candidate + 1e-9 < risk_heuristic_field_[neighbor_index]) {
        risk_heuristic_field_[neighbor_index] = candidate;
        queue.push({candidate, neighbor_index});
      }
    }
  }
}

void HybridAStarPlanner::buildHolonomicHeuristicField(
    double gx, double gy) {
  holonomic_heuristic_field_.clear();
  if (!active_map_ || !active_map_->loaded) return;

  const int width = active_map_->cols;
  const int height = active_map_->rows;
  const int goal_x = static_cast<int>(
      std::floor((gx - active_map_->origin_x) / active_map_->res));
  const int goal_y = static_cast<int>(
      std::floor((gy - active_map_->origin_y) / active_map_->res));
  if (goal_x < 0 || goal_y < 0 ||
      goal_x >= width || goal_y >= height) return;

  auto cell_is_free = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) return false;
    const size_t terrain_index =
        static_cast<size_t>(y * width + x);
    if (use_corridor_ &&
        (terrain_index >= corridor_mask_.size() ||
         !static_cast<bool>(corridor_mask_[terrain_index]))) {
      return false;
    }
    const double wx =
        active_map_->origin_x + (x + 0.5) * active_map_->res;
    const double wy =
        active_map_->origin_y + (y + 0.5) * active_map_->res;
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) return false;
    const size_t map_index =
        static_cast<size_t>(my * cm_width_ + mx);
    return clearance_map_.empty() ||
           (map_index < clearance_map_.size() &&
            static_cast<bool>(clearance_map_[map_index]));
  };

  if (!cell_is_free(goal_x, goal_y)) return;
  const double infinity = std::numeric_limits<double>::infinity();
  holonomic_heuristic_field_.assign(
      static_cast<size_t>(width * height), infinity);
  using GridEntry = std::pair<double, int>;
  std::priority_queue<GridEntry, std::vector<GridEntry>,
                      std::greater<GridEntry>> queue;
  const int goal_index = goal_y * width + goal_x;
  holonomic_heuristic_field_[goal_index] = 0.0;
  queue.push({0.0, goal_index});

  static const int offsets[8][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1},
      {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  while (!queue.empty()) {
    const GridEntry entry = queue.top();
    queue.pop();
    const double current_cost = entry.first;
    const int index = entry.second;
    if (current_cost > holonomic_heuristic_field_[index] + 1e-9)
      continue;
    const int x = index % width;
    const int y = index / width;
    for (const auto& offset : offsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (!cell_is_free(nx, ny)) continue;
      const int neighbor_index = ny * width + nx;
      const double transition = active_map_->res *
          ((offset[0] != 0 && offset[1] != 0)
               ? std::sqrt(2.0) : 1.0);
      const double candidate = current_cost + transition;
      if (candidate + 1e-9 <
          holonomic_heuristic_field_[neighbor_index]) {
        holonomic_heuristic_field_[neighbor_index] = candidate;
        queue.push({candidate, neighbor_index});
      }
    }
  }
}

bool HybridAStarPlanner::holonomicFieldReaches(double x, double y) const {
  if (!active_map_ || !active_map_->loaded ||
      holonomic_heuristic_field_.empty()) {
    return false;
  }
  const int col = static_cast<int>(
      std::floor((x - active_map_->origin_x) / active_map_->res));
  const int row = static_cast<int>(
      std::floor((y - active_map_->origin_y) / active_map_->res));
  if (col < 0 || row < 0 ||
      col >= active_map_->cols || row >= active_map_->rows) {
    return false;
  }
  const size_t index =
      static_cast<size_t>(row * active_map_->cols + col);
  return index < holonomic_heuristic_field_.size() &&
         std::isfinite(holonomic_heuristic_field_[index]);
}

double HybridAStarPlanner::holonomicHeuristic(
    double x, double y, double gx, double gy) const {
  const double euclidean = heuristic(x, y, gx, gy);
  if (!active_map_ || !active_map_->loaded ||
      holonomic_heuristic_field_.empty()) {
    return euclidean;
  }
  const int col = static_cast<int>(
      std::floor((x - active_map_->origin_x) / active_map_->res));
  const int row = static_cast<int>(
      std::floor((y - active_map_->origin_y) / active_map_->res));
  if (col < 0 || row < 0 ||
      col >= active_map_->cols || row >= active_map_->rows) {
    return euclidean;
  }
  const size_t index =
      static_cast<size_t>(row * active_map_->cols + col);
  if (index >= holonomic_heuristic_field_.size() ||
      !std::isfinite(holonomic_heuristic_field_[index])) {
    return euclidean;
  }
  // Both terms are distance lower bounds. Their maximum is still admissible
  // and avoids small cell-centre discretisation underestimates.
  return std::max(euclidean, holonomic_heuristic_field_[index]);
}

double HybridAStarPlanner::riskHeuristic(
    double x, double y, double theta, double gx, double gy) const {
  (void)theta;
  const double distance = heuristic(x, y, gx, gy);
  if (!active_map_ || !active_map_->loaded || distance < 1e-6)
    return distance;

  const int col = static_cast<int>(
      std::floor((x - active_map_->origin_x) / active_map_->res));
  const int row = static_cast<int>(
      std::floor((y - active_map_->origin_y) / active_map_->res));
  if (col >= 0 && row >= 0 &&
      col < active_map_->cols && row < active_map_->rows) {
    const size_t index =
        static_cast<size_t>(row * active_map_->cols + col);
    if (index < risk_heuristic_field_.size() &&
        std::isfinite(risk_heuristic_field_[index]))
      return risk_heuristic_field_[index];
  }
  return distance;
}

double HybridAStarPlanner::headingHeuristic(
    double x, double y, double theta,
    double gx, double gy, double gt) const {
  const double distance = heuristic(x, y, gx, gy);
  const double desired_heading =
      distance > goal_xy_tol_ ? std::atan2(gy - y, gx - x) : gt;
  return distance +
      mha_heading_gain_ * min_turning_radius_ *
      std::fabs(angleDiff(desired_heading, theta));
}

int HybridAStarPlanner::toKey(
    int ix, int iy, int itheta, int steering_index) const {
  // ix/iy follow the resolution of the active terrain layer.  In particular,
  // a 1 m coarse search must not retain the 5 cm costmap state lattice: doing
  // so creates hundreds of almost-identical states for each coarse cell and
  // exhausts the iteration budget on long routes.
  const int rows = active_map_ && active_map_->loaded
      ? active_map_->rows
      : static_cast<int>(costmap_->getSizeInCellsY());
  const int base_key = itheta + heading_bins_ * (iy + rows * ix);
  // Exact dynamic programming for a steering-change edge cost would append
  // previous steering to the state.  In this map that increased the fine
  // lattice sevenfold (425k stored nodes while still 16--21 m from the goal).
  // Keep the original 3-D dominance key and use the previous primitive only
  // to rank incoming edges.  This bounded approximation preserves the useful
  // smoothness preference without destroying long-range reachability.
  (void)steering_index;
  return base_key;
}

// ==================== Node expansion ====================

void HybridAStarPlanner::expand(const Node3D& node,
                                 std::vector<Node3D>& successors) const {
  successors.clear();
  double max_curv = (min_turning_radius_ > 1e-3) ? 1.0 / min_turning_radius_ : 5.0;
  int num_dirs = allow_reverse_ ? 2 : 1;
  double res = costmap_->getResolution();

  // Use active_map_ step if available, otherwise default
  double step = step_size_;

  for (int d = 0; d < num_dirs; ++d) {
    double dir = (d == 0) ? 1.0 : -1.0;
    for (int s = 0; s < num_steering_angles_; ++s) {
      ++benchmark_primitive_attempts_;
      double ratio = (num_steering_angles_ == 1) ? 0.0
                     : (double)s / (num_steering_angles_ - 1) * 2.0 - 1.0;
      double curvature = ratio * max_curv;

      double x = node.x, y = node.y, theta = node.theta;
      double ds = step * dir;
      bool collision = false;
      double terrain_risk_integral = 0.0;
      double pitch_risk_integral = 0.0;
      double roll_risk_integral = 0.0;
      double high_risk_length = 0.0;
      double pointwise_risk_integral = 0.0;
      double segment_peak_risk = 0.0;

      // The original half-cell spacing performed roughly 24 complete
      // collision/terrain queries for every 0.6 m primitive on a 5 cm
      // costmap.  The proposed planner already checks against a 1.35 m
      // clearance-inflated obstacle map, so a <=0.75-cell interval cannot
      // tunnel through a valid obstacle band and removes one third of these
      // repeated queries.  Keep the legacy/basic mode at the original 0.5
      // spacing so the comparison baseline is unchanged.
      const double check_fraction =
          (!basic_mode_ && pareto_terrain_mode_)
              ? primitive_collision_check_fraction_ : 0.5;
      int n_check = std::max(
          2, (int)ceil(fabs(step) / (res * check_fraction)));
      double sub = ds / n_check;

      for (int c = 0; c < n_check; ++c) {
        double dt = curvature * sub;
        x += sub * cos(theta + dt * 0.5);
        y += sub * sin(theta + dt * 0.5);
        theta = mod2pi(theta + dt);
        if (!isTraversable(x, y)) { collision = true; break; }
        if (active_map_ && active_map_->loaded) {
          const TerrainSample terrain = queryTerrainSample(x, y, theta);
          const double sample_risk = terrain.risk;
          const TerrainAttitude& attitude = terrain.attitude;
          if (!attitude.feasible) {
            collision = true;
            break;
          }
          const double sample_length = std::fabs(sub);
          terrain_risk_integral += sample_risk * sample_length;
          pitch_risk_integral += attitude.pitch_risk * sample_length;
          roll_risk_integral += attitude.roll_risk * sample_length;
          segment_peak_risk = std::max(segment_peak_risk, sample_risk);
          if (sample_risk >= pareto_high_risk_threshold_)
            high_risk_length += sample_length;
          pointwise_risk_integral +=
              std::pow(sample_risk, pointwise_risk_exponent_) *
              sample_length;
        }
      }
      if (collision) continue;

      Node3D succ;
      succ.x = x; succ.y = y; succ.theta = mod2pi(theta);
      unsigned int mx, my;
      if (!costmap_->worldToMap(succ.x, succ.y, mx, my)) continue;
      if (active_map_ && active_map_->loaded) {
        succ.ix = static_cast<int>(std::floor(
            (succ.x - active_map_->origin_x) / active_map_->res));
        succ.iy = static_cast<int>(std::floor(
            (succ.y - active_map_->origin_y) / active_map_->res));
        if (succ.ix < 0 || succ.iy < 0 ||
            succ.ix >= active_map_->cols || succ.iy >= active_map_->rows) {
          continue;
        }
      } else {
        succ.ix = static_cast<int>(mx);
        succ.iy = static_cast<int>(my);
      }
      succ.itheta = (int)(succ.theta / theta_res_) % heading_bins_;

      double cost = fabs(step);
      if (dir < 0) cost *= reverse_penalty_;
      if (node.reverse != (dir < 0)) cost += direction_change_penalty_;
      cost += obstacle_penalty_ * getCellCost(succ.x, succ.y) / 253.0;
      cost += curvature_penalty_ * fabs(ratio);
      if (active_curvature_change_weight_ > 0.0) {
        const double previous_ratio =
            max_curv > 1e-9 ? node.motion_curvature / max_curv : 0.0;
        const double curvature_change = ratio - previous_ratio;
        cost += active_curvature_change_weight_ *
                curvature_change * curvature_change;
      }

      if (active_map_ && active_map_->loaded) {
        cost += terrain_cost_weight_ * terrain_risk_integral;
        cost += terrain_cost_weight_ * active_attitude_risk_scale_ *
                (pitch_risk_weight_ * pitch_risk_integral +
                 roll_risk_weight_ * roll_risk_integral);
        // Keep the zero-risk coarse anchor geometrically shortest.  Attitude
        // penalties become active for risk-biased anchors and every robust
        // fine search, so Pareto still receives a genuine distance extreme.
        cost += active_high_risk_weight_ * high_risk_length;
        cost += active_pointwise_risk_weight_ *
                pointwise_risk_integral;
      }
      if (active_diversity_weight_ > 0.0 &&
          active_map_ == &coarse_map_) {
        cost += active_diversity_weight_ *
                queryDiversityPenalty(succ.x, succ.y) * fabs(step);
      }
      if (use_corridor_ && active_corridor_center_weight_ > 0.0) {
        const double normalized_distance =
            queryCorridorCenterDistance(succ.x, succ.y) /
            std::max(corridor_width_, 1e-3);
        cost += active_corridor_center_weight_ *
                normalized_distance * fabs(step);
      }

      succ.g = node.g + cost;
      succ.h = 0;
      succ.path_length = node.path_length + std::fabs(step);
      succ.risk_integral =
          node.risk_integral + terrain_risk_integral;
      succ.pitch_risk_integral =
          node.pitch_risk_integral + pitch_risk_integral;
      succ.roll_risk_integral =
          node.roll_risk_integral + roll_risk_integral;
      succ.high_risk_length =
          node.high_risk_length + high_risk_length;
      succ.pointwise_risk_integral =
          node.pointwise_risk_integral + pointwise_risk_integral;
      succ.peak_risk = std::max(node.peak_risk, segment_peak_risk);
      succ.motion_curvature = curvature;
      succ.steering_index = s;
      succ.motion_length = ds;
      succ.parent = -1;
      succ.reverse = (dir < 0);
      successors.push_back(succ);
    }
  }
}

// ==================== Dubins expansion ====================

bool HybridAStarPlanner::dubinsShot(const Node3D& from,
                                     double gx, double gy, double gt,
                                     std::vector<geometry_msgs::PoseStamped>& path,
                                     const std::string& frame,
                                     const ros::Time& stamp) const {
  if (min_turning_radius_ < 1e-3) return false;
  const double euclid = heuristic(from.x, from.y, gx, gy);
  const double len_limit =
      (euclid < 5.0) ? 20.0 * std::max(euclid, 0.5)
                     : 10.0 * std::max(euclid, 0.5);
  const double sample_step = costmap_->getResolution() * 0.5;

  auto build_best_candidate =
      [&](double terminal_yaw,
          std::vector<geometry_msgs::PoseStamped>& candidate,
          double& candidate_cost) -> bool {
    const std::vector<DubinsPath> paths = dubinsCandidates(
        from.x, from.y, from.theta,
        gx, gy, terminal_yaw, min_turning_radius_);

    candidate.clear();
    candidate_cost = std::numeric_limits<double>::infinity();
    for (const DubinsPath& dp : paths) {
      if (!dp.valid || dp.total_length > len_limit) continue;

      double x = from.x, y = from.y, theta = from.theta;
      std::vector<double> xs, ys, ts;
      xs.push_back(x);
      ys.push_back(y);
      ts.push_back(theta);
      for (int i = 0; i < 3; ++i) {
        sampleDubinsSegment(
            x, y, theta, dp.types[i], dp.segs[i],
            min_turning_radius_, sample_step, xs, ys, ts);
      }

      bool traversable = true;
      for (size_t i = 0; i < xs.size(); ++i) {
        if (!isTraversable(xs[i], ys[i])) {
          traversable = false;
          break;
        }
      }
      if (!traversable) continue;

      std::vector<geometry_msgs::PoseStamped> trial;
      trial.reserve(xs.size());
      for (size_t i = 0; i < xs.size(); ++i) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = frame;
        ps.header.stamp = stamp;
        ps.pose.position.x = xs[i];
        ps.pose.position.y = ys[i];
        ps.pose.position.z = getDEMZ(xs[i], ys[i]);
        tf2::Quaternion q;
        q.setRPY(0, 0, ts[i]);
        ps.pose.orientation = tf2::toMsg(q);
        trial.push_back(ps);
      }

      if (!isAnalyticPathTerrainSafe(trial)) continue;
      const double trial_cost = evaluateAnalyticPathCost(trial);
      if (trial_cost < candidate_cost) {
        candidate_cost = trial_cost;
        candidate = std::move(trial);
      }
    }
    return !candidate.empty();
  };

  // Compare requested and relaxed arrival headings in the same candidate set.
  // Returning the first fixed-yaw curve can create a long loop even when the
  // tracked vehicle could arrive naturally and rotate in place at the goal.
  double best_cost = std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::PoseStamped> best_path;
  double fixed_yaw_cost = std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::PoseStamped> fixed_yaw_path;
  if (build_best_candidate(gt, fixed_yaw_path, fixed_yaw_cost)) {
    best_cost = fixed_yaw_cost;
    best_path = std::move(fixed_yaw_path);
  }
  if (!allow_goal_yaw_relaxation_) {
    if (best_path.empty()) return false;
    path = std::move(best_path);
    return true;
  }

  // A tracked vehicle can rotate in place at the goal. If the fixed-yaw
  // Dubins arc leaves a finite map on a long route, search several geometric
  // arrival headings and let MPC perform only the final in-place yaw alignment.
  const double bearing = std::atan2(gy - from.y, gx - from.x);
  const std::vector<double> relaxed_headings = {
      bearing,
      bearing + M_PI / 4.0,
      bearing - M_PI / 4.0,
      bearing + M_PI / 2.0,
      bearing - M_PI / 2.0,
      bearing + M_PI};

  for (double relaxed_yaw : relaxed_headings) {
    relaxed_yaw = mod2pi(relaxed_yaw);
    std::vector<geometry_msgs::PoseStamped> candidate;
    double path_cost = std::numeric_limits<double>::infinity();
    if (!build_best_candidate(relaxed_yaw, candidate, path_cost)) continue;

    double yaw_delta = mod2pi(relaxed_yaw - gt);
    if (yaw_delta > M_PI) yaw_delta -= 2.0 * M_PI;
    const double candidate_cost =
        path_cost +
        goal_yaw_relaxation_penalty_ * std::fabs(yaw_delta);
    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_path = std::move(candidate);
    }
  }

  if (best_path.empty()) return false;
  path = std::move(best_path);
  return true;
}

double HybridAStarPlanner::evaluateAnalyticPathCost(
    const std::vector<geometry_msgs::PoseStamped>& path) const {
  if (path.size() < 2) return 0.0;

  double cost = 0.0;
  const double max_curvature =
      min_turning_radius_ > 1e-3 ? 1.0 / min_turning_radius_ : 1.0;
  const double lattice_step = std::max(step_size_, 1e-3);

  for (size_t i = 1; i < path.size(); ++i) {
    const auto& previous = path[i - 1];
    const auto& current = path[i];
    const double dx =
        current.pose.position.x - previous.pose.position.x;
    const double dy =
        current.pose.position.y - previous.pose.position.y;
    const double ds = std::hypot(dx, dy);
    if (ds < 1e-6) continue;

    const double theta = tf2::getYaw(current.pose.orientation);
    cost += ds;
    if (active_map_ && active_map_->loaded) {
      const TerrainSample terrain = queryTerrainSample(
          current.pose.position.x, current.pose.position.y, theta);
      const double terrain_cost = terrain.risk;
      const TerrainAttitude& attitude = terrain.attitude;
      cost += terrain_cost_weight_ * terrain_cost * ds;
      cost += terrain_cost_weight_ * active_attitude_risk_scale_ *
              (pitch_risk_weight_ * attitude.pitch_risk +
               roll_risk_weight_ * attitude.roll_risk) * ds;
      if (active_high_risk_weight_ > 0.0 &&
          terrain_cost >= pareto_high_risk_threshold_) {
        cost += active_high_risk_weight_ * ds;
      }
      if (active_pointwise_risk_weight_ > 0.0) {
        cost += active_pointwise_risk_weight_ *
                std::pow(terrain_cost, pointwise_risk_exponent_) * ds;
      }
    }

    const double lattice_fraction = ds / lattice_step;
    cost += obstacle_penalty_ *
            getCellCost(current.pose.position.x,
                        current.pose.position.y) /
            253.0 * lattice_fraction;

    const double previous_theta =
        tf2::getYaw(previous.pose.orientation);
    double heading_delta = mod2pi(theta - previous_theta);
    if (heading_delta > M_PI) heading_delta -= 2.0 * M_PI;
    const double curvature = std::fabs(heading_delta) / ds;
    const double curvature_ratio =
        std::min(1.0, curvature / std::max(max_curvature, 1e-3));
    cost += curvature_penalty_ * curvature_ratio * lattice_fraction;
  }
  return cost;
}

bool HybridAStarPlanner::isAnalyticPathTerrainSafe(
    const std::vector<geometry_msgs::PoseStamped>& path) const {
  if (!active_map_ || !active_map_->loaded || path.size() < 2)
    return true;

  // A connector may remain a risky Pareto candidate, but it must not violate
  // the deliberately generous physical attitude envelope.
  for (size_t i = 1; i < path.size(); ++i) {
    const double theta = tf2::getYaw(path[i].pose.orientation);
    if (!queryTerrainAttitude(path[i].pose.position.x,
                              path[i].pose.position.y, theta).feasible) {
      return false;
    }
  }

  // Pareto mode must observe risky direct connectors as candidates before it
  // can decide whether their distance saving justifies the risk. Traversability
  // is still checked separately, and normalized physical risk is capped at 1.
  if (pareto_terrain_mode_) return true;

  double risk_integral = 0.0;
  double total_length = 0.0;
  double peak_risk = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto& previous = path[i - 1];
    const auto& current = path[i];
    const double ds = std::hypot(
        current.pose.position.x - previous.pose.position.x,
        current.pose.position.y - previous.pose.position.y);
    if (ds < 1e-6) continue;

    const double theta = tf2::getYaw(current.pose.orientation);
    const double risk = queryTerrainCost(
        current.pose.position.x, current.pose.position.y, theta);
    if (!std::isfinite(risk)) return false;

    risk_integral += risk * ds;
    total_length += ds;
    peak_risk = std::max(peak_risk, risk);
    if (peak_risk > analytic_max_peak_terrain_cost_)
      return false;
  }

  if (total_length < 1e-6) return true;
  return risk_integral / total_length <=
         analytic_max_mean_terrain_cost_;
}

// ==================== Path utilities ====================

bool HybridAStarPlanner::smoothPath(
    std::vector<geometry_msgs::PoseStamped>& plan,
    const std::string& frame, const ros::Time& stamp) const {
  // A Dubins shot starts at the last search node, so the joined path can
  // contain duplicate consecutive poses. Remove them before constructing the
  // spline; near-zero knot intervals make the spline numerically unstable.
  std::vector<geometry_msgs::PoseStamped> knots;
  knots.reserve(plan.size());
  for (const auto& pose : plan) {
    if (knots.empty()) {
      knots.push_back(pose);
      continue;
    }
    const double dx = pose.pose.position.x - knots.back().pose.position.x;
    const double dy = pose.pose.position.y - knots.back().pose.position.y;
    if (std::hypot(dx, dy) > 1e-4)
      knots.push_back(pose);
    else
      knots.back().pose.orientation = pose.pose.orientation;
  }

  const int n = (int)knots.size();
  if (n < 4) {
    plan = knots;
    return false;
  }

  std::vector<double> wx(n), wy(n), wt(n);
  for (int i = 0; i < n; ++i) {
    wx[i] = knots[i].pose.position.x;
    wy[i] = knots[i].pose.position.y;
  }

  wt[0] = 0.0;
  for (int i = 1; i < n; ++i) {
    double dx = wx[i] - wx[i-1], dy = wy[i] - wy[i-1];
    wt[i] = wt[i-1] + sqrt(dx*dx + dy*dy);
  }
  double total = wt[n-1];
  if (total < 1e-3) return false;

  std::vector<double> h(n-1);
  for (int i = 0; i < n-1; ++i) {
    h[i] = wt[i+1] - wt[i];
    if (h[i] < 1e-9) h[i] = 1e-9;
  }

  struct Coeffs { std::vector<double> a, b, c, d; };
  const double start_yaw = tf2::getYaw(knots.front().pose.orientation);
  const double end_yaw = tf2::getYaw(knots.back().pose.orientation);

  // Clamped cubic spline. Unlike a natural spline, the endpoint derivatives
  // are fixed to the Hybrid A* headings, so smoothing cannot silently rotate
  // the path tangent away from the vehicle's actual initial orientation.
  auto solveAxis = [&](const std::vector<double>& y,
                       double start_derivative,
                       double end_derivative) -> Coeffs {
    Coeffs sc;
    sc.a = y;
    sc.b.resize(n - 1);
    sc.c.assign(n, 0.0);
    sc.d.resize(n - 1);

    std::vector<double> alpha(n, 0.0);
    alpha[0] =
        3.0 * ((y[1] - y[0]) / h[0] - start_derivative);
    for (int i = 1; i < n - 1; ++i) {
      alpha[i] = 3.0 *
          ((y[i+1] - y[i]) / h[i] -
           (y[i] - y[i-1]) / h[i-1]);
    }
    alpha[n-1] =
        3.0 * (end_derivative -
               (y[n-1] - y[n-2]) / h[n-2]);

    std::vector<double> l(n), mu(n), z(n);
    l[0] = 2.0 * h[0];
    mu[0] = 0.5;
    z[0] = alpha[0] / l[0];
    for (int i = 1; i < n - 1; ++i) {
      l[i] = 2.0 * (wt[i+1] - wt[i-1]) -
             h[i-1] * mu[i-1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }
    l[n-1] = h[n-2] * (2.0 - mu[n-2]);
    z[n-1] =
        (alpha[n-1] - h[n-2] * z[n-2]) / l[n-1];
    sc.c[n-1] = z[n-1];

    for (int i = n - 2; i >= 0; --i) {
      sc.c[i] = z[i] - mu[i] * sc.c[i+1];
      sc.b[i] = (y[i+1] - y[i]) / h[i] -
                h[i] * (sc.c[i+1] + 2.0 * sc.c[i]) / 3.0;
      sc.d[i] = (sc.c[i+1] - sc.c[i]) / (3.0 * h[i]);
    }
    return sc;
  };

  Coeffs sx = solveAxis(wx, std::cos(start_yaw), std::cos(end_yaw));
  Coeffs sy = solveAxis(wy, std::sin(start_yaw), std::sin(end_yaw));

  double sample_step = costmap_->getResolution() * 0.5;
  int ns = std::max(n, (int)ceil(total / sample_step));

  std::vector<geometry_msgs::PoseStamped> smooth;
  smooth.reserve(ns + 1);

  int seg = 0;
  for (int s = 0; s <= ns; ++s) {
    double u = (double)s / ns * total;
    while (seg < n - 2 && u > wt[seg+1]) ++seg;
    double dt = u - wt[seg];
    double dt2 = dt * dt, dt3 = dt2 * dt;

    double px = sx.a[seg] + sx.b[seg]*dt + sx.c[seg]*dt2 + sx.d[seg]*dt3;
    double py = sy.a[seg] + sy.b[seg]*dt + sy.c[seg]*dt2 + sy.d[seg]*dt3;
    double tx = sx.b[seg] + 2.0*sx.c[seg]*dt + 3.0*sx.d[seg]*dt2;
    double ty = sy.b[seg] + 2.0*sy.c[seg]*dt + 3.0*sy.d[seg]*dt2;
    double yaw = atan2(ty, tx);

    // Smoothing is allowed to improve geometry, but it must not cut across
    // the vehicle-clearance band that every Hybrid A* primitive respected.
    // Checking isValid() here used to test only lethal cells, which let a
    // spline pass close enough for the body (or a small tracking error) to
    // strike an obstacle even though the raw lattice path was collision-free.
    if (!isTraversable(px, py)) {
      ROS_WARN("Spline smoothing rejected: clearance/corridor violation "
               "at (%.2f,%.2f)", px, py);
      return false;
    }

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = stamp;
    ps.pose.position.x = px;
    ps.pose.position.y = py;
    ps.pose.position.z = getDEMZ(px, py);
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    ps.pose.orientation = tf2::toMsg(q);
    smooth.push_back(ps);
  }

  // Preserve the exact endpoint orientations against round-off in atan2.
  smooth.front().pose.orientation = knots.front().pose.orientation;
  smooth.back().pose.orientation = knots.back().pose.orientation;

  // Spline interpolation can overshoot the curvature of the Hybrid A*
  // primitives even though every raw motion primitive respects
  // min_turning_radius_. Reject such a path instead of asking MPC to follow an
  // impossible corner; the original Hybrid A* path remains in `plan`.
  double max_smoothed_curvature = 0.0;
  for (size_t i = 1; i + 1 < smooth.size(); ++i) {
    const double ax =
        smooth[i].pose.position.x - smooth[i-1].pose.position.x;
    const double ay =
        smooth[i].pose.position.y - smooth[i-1].pose.position.y;
    const double bx =
        smooth[i+1].pose.position.x - smooth[i].pose.position.x;
    const double by =
        smooth[i+1].pose.position.y - smooth[i].pose.position.y;
    const double a = std::hypot(ax, ay);
    const double b = std::hypot(bx, by);
    const double support = 0.5 * (a + b);
    if (a < 1e-6 || b < 1e-6 || support < 1e-6)
      continue;
    // Keep planner acceptance numerically identical to the offline and online
    // evaluator: wrapped heading change divided by the adjacent arc support.
    const double curvature = std::fabs(angleDiff(
        std::atan2(by, bx), std::atan2(ay, ax))) / support;
    max_smoothed_curvature =
        std::max(max_smoothed_curvature, curvature);
  }

  const double curvature_limit =
      min_turning_radius_ > 1e-3
          ? 1.10 / min_turning_radius_
          : std::numeric_limits<double>::infinity();
  if (max_smoothed_curvature > curvature_limit) {
    ROS_DEBUG("Spline smoothing rejected: curvature %.3f exceeds %.3f 1/m",
              max_smoothed_curvature, curvature_limit);
    return false;
  }

  // Do not let geometric smoothing undo terrain avoidance found by Hybrid A*.
  // Keep the original lattice path if the spline crosses a high-risk cell or
  // raises the same terrain/obstacle/curvature objective by more than 2%.
  if (active_map_ && active_map_->loaded) {
    if (!isAnalyticPathTerrainSafe(smooth)) {
      ROS_DEBUG("Spline smoothing rejected: terrain risk gate exceeded");
      return false;
    }
    const double raw_cost = evaluateAnalyticPathCost(knots);
    const double smooth_cost = evaluateAnalyticPathCost(smooth);
    if (smooth_cost > raw_cost * 1.02) {
      ROS_DEBUG("Spline smoothing rejected: cost %.2f exceeds raw %.2f",
                smooth_cost, raw_cost);
      return false;
    }
  }

  plan = smooth;
  ROS_INFO("Path smoothed: %d -> %d points, max curvature %.3f 1/m",
           n, (int)plan.size(), max_smoothed_curvature);
  return true;
}

bool HybridAStarPlanner::isDistinctFinePath(
    const std::vector<geometry_msgs::PoseStamped>& candidate,
    const std::vector<std::vector<geometry_msgs::PoseStamped>>&
        accepted_paths) const {
  for (const auto& accepted : accepted_paths) {
    if (areFinePathsGeometricallySimilar(candidate, accepted)) {
      return false;
    }
  }
  return true;
}

bool HybridAStarPlanner::areFinePathsGeometricallySimilar(
    const std::vector<geometry_msgs::PoseStamped>& first,
    const std::vector<geometry_msgs::PoseStamped>& second) const {
  const double hausdorff = pathHausdorffDistance(first, second);
  const double overlap = pathOverlapRatio(first, second);
  // Coarse channels need metre-scale topological separation. Fine paths may
  // still realise different risk compromises inside one broad passage, so
  // this predicate reports geometry only. The caller must additionally check
  // Pareto objectives before deciding that a similar path is redundant.
  return hausdorff < fine_min_hausdorff_ || overlap > fine_max_overlap_;
}

void HybridAStarPlanner::buildPath(const std::vector<Node3D>& nodes, int goal_idx,
                                    const std::string& frame, const ros::Time& stamp,
                                    std::vector<geometry_msgs::PoseStamped>& plan) const {
  std::vector<int> idx;
  for (int i = goal_idx; i >= 0; i = nodes[i].parent) idx.push_back(i);
  std::reverse(idx.begin(), idx.end());

  auto make_pose = [&](double x, double y, double theta) {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = stamp;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = getDEMZ(x, y);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    ps.pose.orientation = tf2::toMsg(q);
    return ps;
  };

  plan.clear();
  if (idx.empty()) return;
  plan.push_back(make_pose(
      nodes[idx.front()].x, nodes[idx.front()].y,
      nodes[idx.front()].theta));

  const double reconstruction_step = std::max(
      0.05, 0.5 * costmap_->getResolution());
  for (size_t chain_index = 1; chain_index < idx.size(); ++chain_index) {
    const Node3D& parent = nodes[idx[chain_index - 1]];
    const Node3D& child = nodes[idx[chain_index]];
    if (std::fabs(child.motion_length) < 1e-9) {
      // Start in-place rotation (or a legacy special state) has no travelled
      // arc. Keep its endpoint once; later geometry starts from this heading.
      plan.push_back(make_pose(child.x, child.y, child.theta));
      continue;
    }

    const int samples = std::max(
        1, static_cast<int>(std::ceil(
               std::fabs(child.motion_length) / reconstruction_step)));
    const double sub = child.motion_length / samples;
    double x = parent.x;
    double y = parent.y;
    double theta = parent.theta;
    for (int sample = 1; sample <= samples; ++sample) {
      const double delta_theta = child.motion_curvature * sub;
      x += sub * std::cos(theta + 0.5 * delta_theta);
      y += sub * std::sin(theta + 0.5 * delta_theta);
      theta = mod2pi(theta + delta_theta);
      if (sample == samples) {
        // Remove accumulated floating-point drift while preserving the arc
        // samples that encode the bounded curvature primitive.
        x = child.x;
        y = child.y;
        theta = child.theta;
      }
      plan.push_back(make_pose(x, y, theta));
    }
  }
}

// ==================== Core A* search ====================

bool HybridAStarPlanner::runSearch(
    double sx, double sy, double st,
    double gx, double gy, double gt,
    const std::string& frame, const ros::Time& stamp,
    std::vector<geometry_msgs::PoseStamped>& plan,
    double step_override, int max_iter_override,
    bool use_multi_heuristic) {

  ++benchmark_search_calls_;

  last_search_closest_goal_distance_ =
      std::numeric_limits<double>::infinity();
  last_search_terminal_position_visits_ = 0;
  last_search_processed_iterations_ = 0;
  last_search_anchor_queue_exhausted_ = false;

  double saved_step = step_size_;
  if (step_override > 0) step_size_ = step_override;
  int max_iter = (max_iter_override > 0) ? max_iter_override : max_iterations_;

  plan.clear();

  std::vector<Node3D> nodes;
  nodes.reserve(max_iter);
  {
    Node3D s;
    s.x = sx; s.y = sy; s.theta = st;
    unsigned int mx, my;
    costmap_->worldToMap(sx, sy, mx, my);
    if (active_map_ && active_map_->loaded) {
      s.ix = static_cast<int>(std::floor(
          (sx - active_map_->origin_x) / active_map_->res));
      s.iy = static_cast<int>(std::floor(
          (sy - active_map_->origin_y) / active_map_->res));
    } else {
      s.ix = static_cast<int>(mx);
      s.iy = static_cast<int>(my);
    }
    s.itheta = (int)(st / theta_res_) % heading_bins_;
    s.steering_index = num_steering_angles_ / 2;
    s.g = 0; s.h = holonomicHeuristic(sx, sy, gx, gy);
    s.parent = -1; s.reverse = false;
    nodes.push_back(s);
  }

  using OpenQueue =
      std::priority_queue<std::pair<double, int>,
                          std::vector<std::pair<double, int>>,
                          CompareNode>;
  std::array<OpenQueue, 3> open;
  auto terminal_heading_penalty = [&](const Node3D& node) {
    if (!enforce_terminal_goal_yaw_) return 0.0;
    const double distance = heuristic(node.x, node.y, gx, gy);
    const double blend = std::max(
        0.0, std::min(1.0,
                      1.0 - distance / terminal_heading_blend_distance_));
    if (blend <= 0.0) return 0.0;
    const double bearing = distance > 1e-6
        ? std::atan2(gy - node.y, gx - node.x) : gt;
    const double desired_heading =
        bearing + blend * angleDiff(gt, bearing);
    return terminal_heading_soft_weight_ * blend * min_turning_radius_ *
           std::fabs(angleDiff(desired_heading, node.theta));
  };
  auto push_to_queues = [&](int node_index) {
    const Node3D& node = nodes[node_index];
    const double h_distance =
        holonomicHeuristic(node.x, node.y, gx, gy);
    const double h_terminal = terminal_heading_penalty(node);
    // Weighted A* is used only by the proposed planner's fine corridor
    // searches.  The terrain objective g is untouched; w=1.05 merely changes
    // expansion order and gives a conservative 5% scalar-cost bound when
    // the distance heuristic is admissible.  Coarse search and both
    // standalone comparison algorithms retain their original keys.
    const bool accelerated_fine_search =
        !basic_mode_ && pareto_terrain_mode_ && use_corridor_ &&
        active_map_ && active_map_ == &fine_map_;
    const double anchor_weight = accelerated_fine_search
        ? fine_heuristic_weight_ : mha_anchor_weight_;
    open[0].push(
        {node.g + anchor_weight * h_distance + h_terminal,
         node_index});
    if (use_multi_heuristic) {
      open[1].push(
          {node.g + mha_risk_weight_ *
                        riskHeuristic(node.x, node.y, node.theta, gx, gy) +
                        h_terminal,
           node_index});
      open[2].push(
          {node.g + mha_heading_weight_ *
                        headingHeuristic(
                            node.x, node.y, node.theta, gx, gy, gt) +
                        h_terminal,
           node_index});
    }
  };
  push_to_queues(0);

  // A skid-steer tracked vehicle can rotate at zero translational speed.
  // Without these start states, a forward-only Hybrid A* must draw a large
  // loop whenever the initial vehicle yaw points away from the goal. Seed all
  // heading bins at the same start position and charge the equivalent
  // minimum-radius arc length, so the planner may choose a short initial turn
  // but will not rotate gratuitously.
  if (allow_start_in_place_rotation_) {
    const Node3D start_node = nodes.front();
    for (int heading_index = 0;
         heading_index < heading_bins_;
         ++heading_index) {
      const double candidate_yaw = heading_index * theta_res_;
      const double yaw_change =
          std::fabs(angleDiff(candidate_yaw, st));
      if (yaw_change < 0.5 * theta_res_) continue;

      Node3D rotated = start_node;
      rotated.theta = candidate_yaw;
      rotated.itheta = heading_index;
      rotated.g =
          min_turning_radius_ * yaw_change;
      rotated.path_length = rotated.g;
      rotated.h = holonomicHeuristic(sx, sy, gx, gy);
      rotated.parent = 0;
      const int rotated_index = static_cast<int>(nodes.size());
      nodes.push_back(rotated);
      push_to_queues(rotated_index);
    }
  }

  // best_g suppresses duplicate states before they enter the open queues.
  // The former closed-only check allowed the same lattice state to be queued
  // through many parents before its first expansion (about 4 nodes per
  // iteration in the reported failure), consuming memory and iterations.
  std::unordered_map<int, double> best_g;
  best_g.reserve(static_cast<size_t>(max_iter));
  for (size_t i = 0; i < nodes.size(); ++i) {
    const int state_key = toKey(
        nodes[i].ix, nodes[i].iy, nodes[i].itheta,
        nodes[i].steering_index);
    const auto previous = best_g.find(state_key);
    if (previous == best_g.end() || nodes[i].g < previous->second) {
      best_g[state_key] = nodes[i].g;
    }
  }

  std::unordered_map<int, double> closed;
  closed.reserve(static_cast<size_t>(max_iter));
  auto prune_stale_entries = [&](OpenQueue& queue) {
    while (!queue.empty()) {
      const Node3D& node = nodes[queue.top().second];
      const int key = toKey(
          node.ix, node.iy, node.itheta, node.steering_index);
      const auto best_entry = best_g.find(key);
      const auto closed_entry = closed.find(key);
      const bool superseded =
          best_entry != best_g.end() && best_entry->second + 1e-9 < node.g;
      const bool already_expanded =
          closed_entry != closed.end() && closed_entry->second <= node.g;
      if (!superseded && !already_expanded) {
        break;
      }
      queue.pop();
    }
  };
  double best_analytic_cost = std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::PoseStamped> best_analytic_plan;

  bool found = false;
  std::array<int, 3> queue_expansions{{0, 0, 0}};
  int next_auxiliary_queue = 1;
  int processed_iterations = 0;
  bool anchor_queue_exhausted = false;
  double closest_goal_distance = heuristic(sx, sy, gx, gy);
  int terminal_position_visits = 0;
  double best_terminal_heading_error =
      std::numeric_limits<double>::infinity();
  double best_terminal_lateral_error =
      std::numeric_limits<double>::infinity();
  int best_terminal_node = -1;
  double best_terminal_score = std::numeric_limits<double>::infinity();
  int first_terminal_candidate_iteration = -1;
  const bool coarse_search =
      active_map_ && active_map_ == &coarse_map_;
  double progress_reference_distance = closest_goal_distance;
  int last_meaningful_progress_iteration = 0;
  bool progress_stalled = false;
  for (int iter = 0; iter < max_iter; ++iter) {
    ++benchmark_iterations_;
    processed_iterations = iter + 1;
    for (OpenQueue& queue : open) prune_stale_entries(queue);
    if (open[0].empty()) {
      anchor_queue_exhausted = true;
      break;
    }

    int queue_index = 0;
    if (use_multi_heuristic) {
      const double anchor_key = open[0].top().first;
      for (int attempt = 0; attempt < 2; ++attempt) {
        const int candidate = next_auxiliary_queue;
        next_auxiliary_queue = candidate == 1 ? 2 : 1;
        if (!open[candidate].empty() &&
            open[candidate].top().first <=
                mha_queue_bound_ * anchor_key) {
          queue_index = candidate;
          break;
        }
      }
    }

    auto top = open[queue_index].top();
    open[queue_index].pop();
    ++queue_expansions[queue_index];
    int ci = top.second;
    const Node3D& cur = nodes[ci];
    // Length is not a Pareto weight.  It is a proven eligibility limit for
    // the optional adaptive-safety route.  Euclidean remaining distance is a
    // lower bound, so a node exceeding this sum cannot later satisfy the
    // configured detour bound and may be discarded before collision-heavy
    // expansion without changing any eligible solution.
    if (std::isfinite(active_search_max_path_length_) &&
        cur.path_length + heuristic(cur.x, cur.y, gx, gy) >
            active_search_max_path_length_ + 1e-9) {
      ++benchmark_length_bound_pruned_nodes_;
      continue;
    }
    int key = toKey(
        cur.ix, cur.iy, cur.itheta, cur.steering_index);
    if (closed.count(key) && closed[key] <= cur.g) continue;
    closed[key] = cur.g;
    ++benchmark_expanded_nodes_;

    double dx = cur.x - gx, dy = cur.y - gy;
    double dist_to_goal = sqrt(dx*dx + dy*dy);
    closest_goal_distance = std::min(closest_goal_distance, dist_to_goal);
    if (dist_to_goal <=
        progress_reference_distance - fine_stagnation_progress_m_) {
      progress_reference_distance = dist_to_goal;
      last_meaningful_progress_iteration = iter;
    }

    // Only corridor-constrained fine search uses this conservative dead-end
    // guard. Coarse search and the full-map fallback retain their complete
    // budgets. Under the reverse 2-D distance heuristic, tens of thousands of
    // expansions with no 0.1 m geometric progress while still outside the
    // terminal neighbourhood indicate a kinematically blocked corridor.
    const double terminal_neighbourhood =
        std::max(goal_xy_tol_, 1.5 * step_size_);
    if (!coarse_search && use_corridor_ &&
        iter >= fine_stagnation_min_iterations_ &&
        iter - last_meaningful_progress_iteration >=
            fine_stagnation_window_iterations_ &&
        closest_goal_distance > terminal_neighbourhood) {
      progress_stalled = true;
      break;
    }

    // The coarse layer identifies a traversable topological channel; it must
    // not be required to hit the fine layer's 0.5 m / terminal-yaw capture
    // state with a 2 m primitive.  Accept a coarse node within one primitive
    // when the remaining segment is collision-free, then let the independent
    // fine search inside this corridor enforce the exact position, curvature
    // and terminal heading.  Without this resolution-aware termination, many
    // perfectly reachable long-distance goals lie between coarse lattice
    // samples and the search eventually scans the whole map.
    const double coarse_capture_radius =
        std::max(goal_xy_tol_, step_size_);
    if (coarse_search && dist_to_goal <= coarse_capture_radius &&
        isPathCollisionFree(cur.x, cur.y, gx, gy)) {
      buildPath(nodes, ci, frame, stamp, plan);
      if (dist_to_goal > 1e-4) {
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = frame;
        goal_pose.header.stamp = stamp;
        goal_pose.pose.position.x = gx;
        goal_pose.pose.position.y = gy;
        goal_pose.pose.position.z = getDEMZ(gx, gy);
        tf2::Quaternion goal_q;
        goal_q.setRPY(0.0, 0.0, gt);
        goal_pose.pose.orientation = tf2::toMsg(goal_q);
        plan.push_back(goal_pose);
      }
      ROS_INFO("Hybrid A*: coarse channel reached, iter=%d nodes=%zu "
               "capture_gap=%.2fm step=%.2fm",
               iter, nodes.size(), dist_to_goal, step_size_);
      found = true;
      break;
    }

    // Near the goal, Dubins completes the discretized lattice and preserves
    // the commanded final yaw. Farther away it is only a candidate: score the
    // full Hybrid-A* prefix plus the terrain/obstacle/curvature cost of the
    // analytic segment, but never terminate the search merely because the
    // first collision-free direct curve exists.
    const int analytic_skip = std::max(1, analytic_expansion_skip_);
    const bool near_goal_connector =
        dist_to_goal <= dubins_shot_max_distance_;
    const double prefix_distance = std::hypot(cur.x - sx, cur.y - sy);
    const bool evaluate_far_candidate =
        prefix_distance >= analytic_min_prefix_distance_ &&
        iter % analytic_skip == 0;
    if (enable_dubins_shot_ &&
        (near_goal_connector || evaluate_far_candidate)) {
      std::vector<geometry_msgs::PoseStamped> dp;
      if (dubinsShot(cur, gx, gy, gt, dp, frame, stamp)) {
        const double candidate_cost =
            cur.g + evaluateAnalyticPathCost(dp);
        if (candidate_cost < best_analytic_cost) {
          best_analytic_cost = candidate_cost;
          buildPath(nodes, ci, frame, stamp, best_analytic_plan);
          best_analytic_plan.insert(
              best_analytic_plan.end(), dp.begin(), dp.end());
        }

        if (near_goal_connector) {
          plan = best_analytic_plan;
          smoothPath(plan, frame, stamp);
          if (!plan.empty()) {
            tf2::Quaternion goal_q;
            goal_q.setRPY(0, 0, gt);
            plan.back().pose.orientation = tf2::toMsg(goal_q);
          }
          ROS_INFO("Hybrid A*: near-goal Dubins, iter=%d dist=%.1f "
                   "cost=%.2f step=%.2f",
                   iter, dist_to_goal, best_analytic_cost, step_size_);
          found = true;
          break;
        }
      }
    }

    // Terminal yaw is a continuous objective, not a lattice acceptance gate.
    // The old hard heading threshold rejected hundreds of states only a few
    // centimetres from the goal whenever the discretized yaw happened to lie
    // just outside that threshold.  The open-list penalty above starts turning
    // before the goal; here we retain the best nearby state according to path
    // cost plus residual heading/lateral error.  A short settling window lets
    // competing terminal states be compared without exhausting the full map.
    // The curvature- and collision-checked Hermite transition in makePlan()
    // subsequently imposes the exact requested tangent at the final goal.
    const double terminal_heading_error =
        std::fabs(angleDiff(cur.theta, gt));
    // A bearing computed from a node only a few centimetres from the goal is
    // numerically unsuitable as a terminal-tangent test: a 2 cm lateral
    // offset makes atan2 report roughly 90 degrees even when the vehicle and
    // the reserved terminal segment have the correct yaw.  Resolve the
    // residual in the goal-aligned frame instead.  The small lateral residual
    // is bounded geometrically, while yaw remains a continuously scored
    // quantity and is completed by the final tangent transition.
    const double goal_frame_lateral_error = std::fabs(
        -std::sin(gt) * (cur.x - gx) +
         std::cos(gt) * (cur.y - gy));
    const double terminal_lateral_tolerance = std::max(
        2.0 * costmap_->getResolution(), 0.25 * goal_xy_tol_);
    if (dist_to_goal < goal_xy_tol_) {
      ++terminal_position_visits;
      best_terminal_heading_error = std::min(
          best_terminal_heading_error, terminal_heading_error);
      best_terminal_lateral_error = std::min(
          best_terminal_lateral_error, goal_frame_lateral_error);
    }
    const bool terminal_connection_clear =
        dist_to_goal <= 1e-4 ||
        isPathCollisionFree(cur.x, cur.y, gx, gy);
    const bool terminal_candidate =
        dist_to_goal < goal_xy_tol_ && terminal_connection_clear &&
        (!enforce_terminal_goal_yaw_ ||
         (!cur.reverse &&
          goal_frame_lateral_error <= terminal_lateral_tolerance));
    if (terminal_candidate) {
      const double terminal_score =
          cur.g + terminal_heading_soft_weight_ * min_turning_radius_ *
                      terminal_heading_error +
          dist_to_goal + 2.0 * goal_frame_lateral_error;
      if (terminal_score + 1e-9 < best_terminal_score) {
        best_terminal_score = terminal_score;
        best_terminal_node = ci;
      }
      if (first_terminal_candidate_iteration < 0)
        first_terminal_candidate_iteration = iter;
      if (iter - first_terminal_candidate_iteration >=
          terminal_candidate_settle_iterations_) {
        break;
      }
    }

    std::vector<Node3D> succs;
    expand(cur, succs);
    benchmark_generated_nodes_ += succs.size();
    for (auto& s : succs) {
      s.h = holonomicHeuristic(s.x, s.y, gx, gy);
      if (std::isfinite(active_search_max_path_length_) &&
          s.path_length + heuristic(s.x, s.y, gx, gy) >
              active_search_max_path_length_ + 1e-9) {
        ++benchmark_length_bound_pruned_nodes_;
        continue;
      }
      int sk = toKey(s.ix, s.iy, s.itheta, s.steering_index);
      if (closed.count(sk) && closed[sk] <= s.g) continue;
      const auto previous = best_g.find(sk);
      if (previous != best_g.end() && previous->second <= s.g) continue;
      best_g[sk] = s.g;
      s.parent = ci;
      int ni = (int)nodes.size();
      nodes.push_back(s);
      push_to_queues(ni);
    }
    benchmark_max_stored_nodes_ = std::max(
        benchmark_max_stored_nodes_,
        static_cast<std::uint64_t>(nodes.size()));
  }

  ROS_INFO("Hybrid A* queue expansions: anchor=%d risk=%d heading=%d",
           queue_expansions[0], queue_expansions[1], queue_expansions[2]);
  benchmark_max_stored_nodes_ = std::max(
      benchmark_max_stored_nodes_,
      static_cast<std::uint64_t>(nodes.size()));

  if (!found && best_terminal_node >= 0) {
    const Node3D& terminal = nodes[best_terminal_node];
    const double terminal_gap =
        std::hypot(terminal.x - gx, terminal.y - gy);
    const double terminal_heading_error =
        std::fabs(angleDiff(terminal.theta, gt));
    const double terminal_lateral_error = std::fabs(
        -std::sin(gt) * (terminal.x - gx) +
         std::cos(gt) * (terminal.y - gy));
    buildPath(nodes, best_terminal_node, frame, stamp, plan);

    tf2::Quaternion goal_q;
    goal_q.setRPY(0.0, 0.0, gt);
    if (terminal_gap > 1e-4) {
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = frame;
      goal_pose.header.stamp = stamp;
      goal_pose.pose.position.x = gx;
      goal_pose.pose.position.y = gy;
      goal_pose.pose.position.z = getDEMZ(gx, gy);
      goal_pose.pose.orientation = tf2::toMsg(goal_q);
      plan.push_back(goal_pose);
    }
    smoothPath(plan, frame, stamp);
    if (!plan.empty()) plan.back().pose.orientation = tf2::toMsg(goal_q);
    ROS_INFO("Hybrid A*: soft terminal candidate selected, nodes=%zu "
             "gap=%.3fm lateral_error=%.3fm heading_error=%.1fdeg "
             "score=%.3f candidates=%d final_yaw=%.1fdeg",
             nodes.size(), terminal_gap, terminal_lateral_error,
             terminal_heading_error * 180.0 / M_PI,
             best_terminal_score, terminal_position_visits,
             gt * 180.0 / M_PI);
    found = true;
  }

  if (!found && !best_analytic_plan.empty()) {
    plan = best_analytic_plan;
    smoothPath(plan, frame, stamp);
    if (!plan.empty()) {
      tf2::Quaternion goal_q;
      goal_q.setRPY(0, 0, gt);
      plan.back().pose.orientation = tf2::toMsg(goal_q);
    }
    ROS_WARN("Hybrid A*: lattice search exhausted; using lowest-cost "
             "terrain-scored Dubins fallback (cost=%.2f, points=%zu)",
             best_analytic_cost, plan.size());
    found = true;
  }

  if (!found) {
    ROS_WARN("Hybrid A*: search failed: reason=%s iterations=%d/%d "
             "nodes=%zu expansions=(%d,%d,%d) step=%.2f "
             "closest_goal=%.2fm layer=%s terminal_visits=%d "
             "best_heading=%.1fdeg best_lateral=%.3fm",
             anchor_queue_exhausted
                 ? "anchor_queue_exhausted"
                 : (progress_stalled ? "progress_stalled"
                                     : "iteration_limit"),
             processed_iterations, max_iter, nodes.size(),
             queue_expansions[0], queue_expansions[1],
             queue_expansions[2], step_size_, closest_goal_distance,
             coarse_search ? "coarse" : "fine",
             terminal_position_visits,
             std::isfinite(best_terminal_heading_error)
                 ? best_terminal_heading_error * 180.0 / M_PI : -1.0,
             std::isfinite(best_terminal_lateral_error)
                 ? best_terminal_lateral_error : -1.0);
  }

  last_search_closest_goal_distance_ = closest_goal_distance;
  last_search_terminal_position_visits_ = terminal_position_visits;
  last_search_processed_iterations_ = processed_iterations;
  last_search_anchor_queue_exhausted_ = anchor_queue_exhausted;

  step_size_ = saved_step;
  return found;
}

bool HybridAStarPlanner::runParetoCoarseSearch(
    double sx, double sy, double st,
    double gx, double gy, double gt,
    const std::string& frame, const ros::Time& stamp,
    std::vector<std::vector<geometry_msgs::PoseStamped>>& plans,
    double step_override, int max_iter_override,
    CoarseSearchAudit* audit) {
  plans.clear();
  if (!active_map_ || active_map_ != &coarse_map_ ||
      !coarse_map_.loaded) {
    return false;
  }

  ++benchmark_search_calls_;
  const double saved_step = step_size_;
  const double saved_terrain_weight = terrain_cost_weight_;
  const double saved_high_risk_weight = active_high_risk_weight_;
  const double saved_pointwise_weight = active_pointwise_risk_weight_;
  const double saved_attitude_scale = active_attitude_risk_scale_;
  const double saved_diversity_weight = active_diversity_weight_;
  if (step_override > 0.0) step_size_ = step_override;
  const int max_iter = max_iter_override > 0
      ? max_iter_override : max_iterations_;

  // Pareto labels carry terrain objectives explicitly.  Leaving a terrain or
  // diversity weight inside scalar g would bias every queue toward one
  // preselected compromise and reproduce the missing-channel problem.
  terrain_cost_weight_ = 0.0;
  active_high_risk_weight_ = 0.0;
  active_pointwise_risk_weight_ = 0.0;
  active_attitude_risk_scale_ = 1.0;
  active_diversity_weight_ = 0.0;
  // Keep the shared multi-label coarse anchor neutral and Euclidean.  The
  // obstacle-aware field improved wall time in nav_7, but also changed the
  // auxiliary-queue admission order enough to remove useful goal labels:
  // long_01 lost its second Pareto route and long_03/04 lost one channel.
  // Obstacle-aware guidance remains active in every independent fine search,
  // where topology has already been fixed and candidate diversity cannot be
  // collapsed by the heuristic.
  buildRiskHeuristicField(gx, gy);

  using OpenQueue =
      std::priority_queue<std::pair<double, int>,
                          std::vector<std::pair<double, int>>,
                          CompareNode>;
  std::array<OpenQueue, 3> open;
  std::vector<Node3D> nodes;
  std::vector<bool> active;
  std::vector<bool> expanded;
  nodes.reserve(static_cast<size_t>(max_iter));
  active.reserve(static_cast<size_t>(max_iter));
  expanded.reserve(static_cast<size_t>(max_iter));

  Node3D start;
  start.x = sx;
  start.y = sy;
  start.theta = st;
  unsigned int start_mx, start_my;
  if (!costmap_->worldToMap(sx, sy, start_mx, start_my)) {
    step_size_ = saved_step;
    terrain_cost_weight_ = saved_terrain_weight;
    active_high_risk_weight_ = saved_high_risk_weight;
    active_pointwise_risk_weight_ = saved_pointwise_weight;
    active_attitude_risk_scale_ = saved_attitude_scale;
    active_diversity_weight_ = saved_diversity_weight;
    return false;
  }
  start.ix = static_cast<int>(std::floor(
      (sx - coarse_map_.origin_x) / coarse_map_.res));
  start.iy = static_cast<int>(std::floor(
      (sy - coarse_map_.origin_y) / coarse_map_.res));
  start.itheta = static_cast<int>(st / theta_res_) % heading_bins_;
  start.steering_index = num_steering_angles_ / 2;
  start.g = 0.0;
  start.h = heuristic(sx, sy, gx, gy);
  start.parent = -1;
  start.reverse = false;
  nodes.push_back(start);
  active.push_back(true);
  expanded.push_back(false);

  const double route_dx = gx - sx;
  const double route_dy = gy - sy;
  const double route_length = std::max(
      1e-6, std::hypot(route_dx, route_dy));
  const double route_ux = route_dx / route_length;
  const double route_uy = route_dy / route_length;
  auto topology_signature_key = [&](const Node3D& node) {
    return (static_cast<std::uint32_t>(node.topology_gate_mask) << 16) |
           static_cast<std::uint32_t>(node.topology_signature);
  };
  auto same_topology_signature = [&](const Node3D& lhs,
                                     const Node3D& rhs) {
    return !coarse_topology_signature_enabled_ ||
           topology_signature_key(lhs) == topology_signature_key(rhs);
  };
  auto update_topology_signature = [&](const Node3D& current,
                                       Node3D& successor) {
    successor.topology_signature = current.topology_signature;
    successor.topology_gate_mask = current.topology_gate_mask;
    if (!coarse_topology_signature_enabled_) return;

    const double current_dx = current.x - sx;
    const double current_dy = current.y - sy;
    const double successor_dx = successor.x - sx;
    const double successor_dy = successor.y - sy;
    const double current_progress =
        current_dx * route_ux + current_dy * route_uy;
    const double successor_progress =
        successor_dx * route_ux + successor_dy * route_uy;
    const std::array<double, 2> gate_progress{{
        coarse_topology_gate_first_ratio_ * route_length,
        coarse_topology_gate_second_ratio_ * route_length}};
    for (size_t gate = 0; gate < gate_progress.size(); ++gate) {
      const std::uint8_t gate_bit = static_cast<std::uint8_t>(1u << gate);
      if ((successor.topology_gate_mask & gate_bit) != 0u ||
          successor_progress + 1e-9 < gate_progress[gate]) {
        continue;
      }
      double interpolation = 1.0;
      const double progress_delta = successor_progress - current_progress;
      if (current_progress < gate_progress[gate] &&
          progress_delta > 1e-9) {
        interpolation = std::max(
            0.0, std::min(1.0,
                          (gate_progress[gate] - current_progress) /
                              progress_delta));
      }
      const double crossing_x =
          current.x + interpolation * (successor.x - current.x);
      const double crossing_y =
          current.y + interpolation * (successor.y - current.y);
      const double crossing_dx = crossing_x - sx;
      const double crossing_dy = crossing_y - sy;
      const double lateral_offset =
          -route_uy * crossing_dx + route_ux * crossing_dy;
      // Only preserve the topological side of the route at each gate.  Five
      // lateral bins per gate retained many geometrically equivalent labels
      // (nav_32 admitted 78k signature labels) and multiplied fine searches
      // without revealing another usable passage.  Left/centre/right keeps
      // the missing-side discovery while remaining deliberately coarse.
      int signed_bin = 0;
      const double side_threshold =
          0.5 * coarse_topology_lateral_bin_width_;
      if (lateral_offset > side_threshold) {
        signed_bin = 1;
      } else if (lateral_offset < -side_threshold) {
        signed_bin = -1;
      }
      const std::uint16_t encoded_bin =
          static_cast<std::uint16_t>(signed_bin + 1);
      const int shift = static_cast<int>(gate) * 3;
      successor.topology_signature = static_cast<std::uint16_t>(
          (successor.topology_signature &
           static_cast<std::uint16_t>(~(0x7u << shift))) |
          static_cast<std::uint16_t>(encoded_bin << shift));
      successor.topology_gate_mask = static_cast<std::uint8_t>(
          successor.topology_gate_mask | gate_bit);
    }
  };

  auto label_risk = [&](const Node3D& node) {
    const double length = std::max(node.path_length, 1e-3);
    const double mean = node.risk_integral / length;
    const double exposure = node.high_risk_length / length;
    const double pointwise = node.pointwise_risk_integral / length;
    const double pitch = node.pitch_risk_integral / length;
    const double roll = node.roll_risk_integral / length;
    return 0.25 * mean + 0.15 * exposure + 0.15 * node.peak_risk +
           0.10 * pointwise + 0.15 * pitch + 0.20 * roll;
  };

  auto terminal_heading_penalty = [&](const Node3D& node) {
    if (!enforce_terminal_goal_yaw_) return 0.0;
    const double distance = heuristic(node.x, node.y, gx, gy);
    const double blend = std::max(
        0.0, std::min(1.0,
                      1.0 - distance / terminal_heading_blend_distance_));
    if (blend <= 0.0) return 0.0;
    const double bearing = distance > 1e-6
        ? std::atan2(gy - node.y, gx - node.x) : gt;
    const double desired = bearing + blend * angleDiff(gt, bearing);
    return terminal_heading_soft_weight_ * blend * min_turning_radius_ *
           std::fabs(angleDiff(desired, node.theta));
  };

  auto push_queues = [&](int node_index) {
    const Node3D& node = nodes[node_index];
    const double terminal = terminal_heading_penalty(node);
    const double distance_h = heuristic(node.x, node.y, gx, gy);
    open[0].push({node.g + mha_anchor_weight_ * distance_h + terminal,
                  node_index});
    // All keys retain motion cost and metre-scale cost-to-go, so the standard
    // bounded auxiliary-queue scheduler compares quantities of similar scale.
    open[1].push({
        node.g + 0.5 * pareto_anchor_strength_ *
                     label_risk(node) * std::max(node.path_length, step_size_) +
            mha_risk_weight_ *
                riskHeuristic(node.x, node.y, node.theta, gx, gy) + terminal,
        node_index});
    open[2].push({
        node.g + mha_heading_weight_ *
                     headingHeuristic(
                         node.x, node.y, node.theta, gx, gy, gt) + terminal,
        node_index});
  };
  push_queues(0);

  std::unordered_map<int, std::vector<int>> label_archive;
  label_archive.reserve(static_cast<size_t>(max_iter / 2));
  label_archive[toKey(start.ix, start.iy, start.itheta,
                      start.steering_index)].push_back(0);

  auto epsilon_dominates = [&](const Node3D& lhs, const Node3D& rhs) {
    const double length_eps = coarse_label_epsilon_length_;
    const double risk_eps = coarse_label_epsilon_risk_ *
                            std::max(1.0, rhs.path_length);
    const double peak_eps = coarse_label_epsilon_risk_;
    const bool no_worse =
        lhs.path_length <= rhs.path_length + length_eps &&
        lhs.risk_integral <= rhs.risk_integral + risk_eps &&
        lhs.pitch_risk_integral <=
            rhs.pitch_risk_integral + risk_eps &&
        lhs.roll_risk_integral <= rhs.roll_risk_integral + risk_eps &&
        lhs.high_risk_length <= rhs.high_risk_length + risk_eps &&
        lhs.pointwise_risk_integral <=
            rhs.pointwise_risk_integral + risk_eps &&
        lhs.peak_risk <= rhs.peak_risk + peak_eps;
    const bool better =
        lhs.path_length < rhs.path_length - length_eps ||
        lhs.risk_integral < rhs.risk_integral - risk_eps ||
        lhs.pitch_risk_integral <
            rhs.pitch_risk_integral - risk_eps ||
        lhs.roll_risk_integral < rhs.roll_risk_integral - risk_eps ||
        lhs.high_risk_length < rhs.high_risk_length - risk_eps ||
        lhs.pointwise_risk_integral <
            rhs.pointwise_risk_integral - risk_eps ||
        lhs.peak_risk < rhs.peak_risk - peak_eps;
    return no_worse && better;
  };

  auto retain_representative_labels = [&](std::vector<int> pool,
                                           int label_limit) {
    pool.erase(std::remove_if(pool.begin(), pool.end(),
                              [&](int index) { return !active[index]; }),
               pool.end());
    if (static_cast<int>(pool.size()) <= label_limit)
      return pool;

    std::vector<int> kept;
    auto add_unique = [&](int index) {
      if (std::find(kept.begin(), kept.end(), index) == kept.end())
        kept.push_back(index);
    };
    const int shortest = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          if (std::fabs(nodes[lhs].path_length - nodes[rhs].path_length) >
              1e-9)
            return nodes[lhs].path_length < nodes[rhs].path_length;
          return nodes[lhs].g < nodes[rhs].g;
        });
    const int lowest_mean_risk = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          const double lhs_risk = nodes[lhs].risk_integral /
              std::max(nodes[lhs].path_length, 1e-3);
          const double rhs_risk = nodes[rhs].risk_integral /
              std::max(nodes[rhs].path_length, 1e-3);
          if (std::fabs(lhs_risk - rhs_risk) > 1e-9)
            return lhs_risk < rhs_risk;
          return nodes[lhs].path_length < nodes[rhs].path_length;
        });
    const int lowest_exposure = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          const double lhs_exposure = nodes[lhs].high_risk_length /
              std::max(nodes[lhs].path_length, 1e-3);
          const double rhs_exposure = nodes[rhs].high_risk_length /
              std::max(nodes[rhs].path_length, 1e-3);
          if (std::fabs(lhs_exposure - rhs_exposure) > 1e-9)
            return lhs_exposure < rhs_exposure;
          if (std::fabs(nodes[lhs].peak_risk - nodes[rhs].peak_risk) > 1e-9)
            return nodes[lhs].peak_risk < nodes[rhs].peak_risk;
          return nodes[lhs].path_length < nodes[rhs].path_length;
        });
    const int lowest_pitch_risk = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          const double lhs_pitch = nodes[lhs].pitch_risk_integral /
              std::max(nodes[lhs].path_length, 1e-3);
          const double rhs_pitch = nodes[rhs].pitch_risk_integral /
              std::max(nodes[rhs].path_length, 1e-3);
          if (std::fabs(lhs_pitch - rhs_pitch) > 1e-9)
            return lhs_pitch < rhs_pitch;
          return nodes[lhs].path_length < nodes[rhs].path_length;
        });
    const int lowest_roll_risk = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          const double lhs_roll = nodes[lhs].roll_risk_integral /
              std::max(nodes[lhs].path_length, 1e-3);
          const double rhs_roll = nodes[rhs].roll_risk_integral /
              std::max(nodes[rhs].path_length, 1e-3);
          if (std::fabs(lhs_roll - rhs_roll) > 1e-9)
            return lhs_roll < rhs_roll;
          return nodes[lhs].path_length < nodes[rhs].path_length;
        });
    const int lowest_local_risk = *std::min_element(
        pool.begin(), pool.end(), [&](int lhs, int rhs) {
          if (std::fabs(nodes[lhs].peak_risk - nodes[rhs].peak_risk) > 1e-9)
            return nodes[lhs].peak_risk < nodes[rhs].peak_risk;
          const double lhs_pointwise = nodes[lhs].pointwise_risk_integral /
              std::max(nodes[lhs].path_length, 1e-3);
          const double rhs_pointwise = nodes[rhs].pointwise_risk_integral /
              std::max(nodes[rhs].path_length, 1e-3);
          if (std::fabs(lhs_pointwise - rhs_pointwise) > 1e-9)
            return lhs_pointwise < rhs_pointwise;
          return nodes[lhs].path_length < nodes[rhs].path_length;
        });
    add_unique(shortest);
    add_unique(lowest_mean_risk);
    // Reserve at most one archive slot for the best label belonging to an
    // unseen spatial signature. Objective extrema still occupy the remaining
    // slots, so topology awareness cannot multiply the four-label state cap.
    if (coarse_topology_signature_enabled_ &&
        static_cast<int>(kept.size()) < label_limit) {
      auto signature_already_kept = [&](int candidate) {
        const std::uint32_t signature =
            topology_signature_key(nodes[candidate]);
        return std::any_of(
            kept.begin(), kept.end(), [&](int selected) {
              return topology_signature_key(nodes[selected]) == signature;
            });
      };
      int best_unseen_signature = -1;
      double best_unseen_score = std::numeric_limits<double>::infinity();
      for (int candidate : pool) {
        if (signature_already_kept(candidate)) continue;
        const double score = nodes[candidate].path_length *
            (1.0 + pareto_anchor_strength_ * label_risk(nodes[candidate]));
        if (score < best_unseen_score - 1e-9 ||
            (std::fabs(score - best_unseen_score) <= 1e-9 &&
             (best_unseen_signature < 0 ||
              nodes[candidate].g < nodes[best_unseen_signature].g))) {
          best_unseen_signature = candidate;
          best_unseen_score = score;
        }
      }
      if (best_unseen_signature >= 0) {
        add_unique(best_unseen_signature);
        if (audit) ++audit->topology_signature_cap_protected;
      }
    }
    if (static_cast<int>(kept.size()) < label_limit)
      add_unique(lowest_pitch_risk);
    if (static_cast<int>(kept.size()) < label_limit)
      add_unique(lowest_roll_risk);
    if (static_cast<int>(kept.size()) < label_limit)
      add_unique(lowest_exposure);
    if (static_cast<int>(kept.size()) < label_limit)
      add_unique(lowest_local_risk);

    double min_length = std::numeric_limits<double>::infinity();
    double max_length = -std::numeric_limits<double>::infinity();
    double min_risk = std::numeric_limits<double>::infinity();
    double max_risk = -std::numeric_limits<double>::infinity();
    for (int index : pool) {
      min_length = std::min(min_length, nodes[index].path_length);
      max_length = std::max(max_length, nodes[index].path_length);
      min_risk = std::min(min_risk, label_risk(nodes[index]));
      max_risk = std::max(max_risk, label_risk(nodes[index]));
    }
    const double length_range = std::max(1e-6, max_length - min_length);
    const double risk_range = std::max(1e-6, max_risk - min_risk);
    while (static_cast<int>(kept.size()) < label_limit) {
      int best = -1;
      double best_distance = -1.0;
      for (int candidate : pool) {
        if (std::find(kept.begin(), kept.end(), candidate) != kept.end())
          continue;
        double minimum_distance = std::numeric_limits<double>::infinity();
        const double candidate_length =
            (nodes[candidate].path_length - min_length) / length_range;
        const double candidate_risk =
            (label_risk(nodes[candidate]) - min_risk) / risk_range;
        for (int selected : kept) {
          const double selected_length =
              (nodes[selected].path_length - min_length) / length_range;
          const double selected_risk =
              (label_risk(nodes[selected]) - min_risk) / risk_range;
          minimum_distance = std::min(
              minimum_distance,
              std::hypot(candidate_length - selected_length,
                         candidate_risk - selected_risk));
        }
        if (minimum_distance > best_distance + 1e-9 ||
            (std::fabs(minimum_distance - best_distance) <= 1e-9 &&
             (best < 0 || nodes[candidate].g < nodes[best].g))) {
          best = candidate;
          best_distance = minimum_distance;
        }
      }
      if (best < 0) break;
      kept.push_back(best);
    }
    return kept;
  };

  auto insert_label = [&](int node_index) {
    Node3D& candidate = nodes[node_index];
    const int key = toKey(candidate.ix, candidate.iy, candidate.itheta,
                          candidate.steering_index);
    std::vector<int>& archive = label_archive[key];
    archive.erase(std::remove_if(archive.begin(), archive.end(),
                                 [&](int index) { return !active[index]; }),
                  archive.end());
    const bool unseen_signature = coarse_topology_signature_enabled_ &&
        candidate.topology_gate_mask != 0u &&
        std::none_of(archive.begin(), archive.end(), [&](int existing) {
          return same_topology_signature(nodes[existing], candidate);
        });
    if (unseen_signature && audit)
      ++audit->topology_signature_labels_admitted;
    for (int existing : archive) {
      if (!same_topology_signature(nodes[existing], candidate)) continue;
      if (epsilon_dominates(nodes[existing], candidate)) {
        active[node_index] = false;
        if (audit) ++audit->label_dominance_pruned;
        return false;
      }
    }
    for (int existing : archive) {
      if (!same_topology_signature(nodes[existing], candidate)) continue;
      if (epsilon_dominates(candidate, nodes[existing])) {
        active[existing] = false;
        if (audit) ++audit->label_dominance_pruned;
      }
    }
    archive.erase(std::remove_if(archive.begin(), archive.end(),
                                 [&](int index) { return !active[index]; }),
                  archive.end());
    archive.push_back(node_index);
    const std::vector<int> kept = retain_representative_labels(
        archive, coarse_labels_per_state_);
    for (int existing : archive) {
      if (std::find(kept.begin(), kept.end(), existing) == kept.end()) {
        active[existing] = false;
        if (audit) ++audit->label_cap_pruned;
      }
    }
    archive = kept;
    return static_cast<bool>(active[node_index]);
  };

  auto prune_queue = [&](OpenQueue& queue) {
    while (!queue.empty()) {
      const int index = queue.top().second;
      if (index >= 0 && index < static_cast<int>(active.size()) &&
          active[index] && !expanded[index])
        break;
      queue.pop();
    }
  };

  std::vector<int> goal_labels;
  auto add_goal_label = [&](int node_index) -> bool {
    for (int existing : goal_labels) {
      if (!same_topology_signature(nodes[existing], nodes[node_index]))
        continue;
      if (epsilon_dominates(nodes[existing], nodes[node_index])) return false;
    }
    const std::vector<int> previous_labels = goal_labels;
    goal_labels.erase(
        std::remove_if(goal_labels.begin(), goal_labels.end(),
                       [&](int existing) {
                         return same_topology_signature(
                                    nodes[existing], nodes[node_index]) &&
                                epsilon_dominates(
                              nodes[node_index], nodes[existing]);
                       }),
        goal_labels.end());
    goal_labels.push_back(node_index);
    if (static_cast<int>(goal_labels.size()) > coarse_goal_label_limit_) {
      goal_labels = retain_representative_labels(
          goal_labels, coarse_goal_label_limit_);
    }
    return goal_labels != previous_labels;
  };

  auto count_goal_topologies = [&]() {
    std::vector<std::vector<geometry_msgs::PoseStamped>> distinct_paths;
    distinct_paths.reserve(goal_labels.size());
    for (int goal_index : goal_labels) {
      std::vector<geometry_msgs::PoseStamped> path;
      buildPath(nodes, goal_index, frame, stamp, path);
      if (isDistinctPath(path, distinct_paths))
        distinct_paths.push_back(std::move(path));
    }
    return static_cast<int>(distinct_paths.size());
  };

  auto count_goal_signatures = [&]() {
    std::vector<std::uint32_t> signatures;
    signatures.reserve(goal_labels.size());
    for (int goal_index : goal_labels) {
      const std::uint32_t signature =
          topology_signature_key(nodes[goal_index]);
      if (std::find(signatures.begin(), signatures.end(), signature) ==
          signatures.end()) {
        signatures.push_back(signature);
      }
    }
    return static_cast<int>(signatures.size());
  };

  const ros::WallTime search_start = ros::WallTime::now();
  int last_goal_pool_change_iteration = -1;
  int observed_goal_topologies = 0;
  int observed_goal_signatures = 0;
  int next_auxiliary_queue = 1;
  std::array<int, 3> queue_expansions{{0, 0, 0}};
  int processed_iterations = 0;
  std::string coarse_stop_reason = "iteration_limit";
  for (int iteration = 0; iteration < max_iter; ++iteration) {
    if (coarse_multi_label_time_limit_ > 0.0 &&
        (ros::WallTime::now() - search_start).toSec() >=
            coarse_multi_label_time_limit_) {
      coarse_stop_reason = "wall_time_limit";
      break;
    }
    ++benchmark_iterations_;
    processed_iterations = iteration + 1;
    for (OpenQueue& queue : open) prune_queue(queue);
    if (open[0].empty()) {
      coarse_stop_reason = "anchor_queue_exhausted";
      break;
    }

    int queue_index = 0;
    const double anchor_key = open[0].top().first;
    for (int attempt = 0; attempt < 2; ++attempt) {
      const int candidate_queue = next_auxiliary_queue;
      next_auxiliary_queue = candidate_queue == 1 ? 2 : 1;
      if (!open[candidate_queue].empty() &&
          open[candidate_queue].top().first <=
              mha_queue_bound_ * anchor_key) {
        queue_index = candidate_queue;
        break;
      }
    }

    const int current_index = open[queue_index].top().second;
    open[queue_index].pop();
    ++queue_expansions[queue_index];
    if (!active[current_index] || expanded[current_index]) continue;
    expanded[current_index] = true;
    ++benchmark_expanded_nodes_;
    const Node3D& current = nodes[current_index];

    const double distance_to_goal =
        std::hypot(current.x - gx, current.y - gy);
    const double coarse_capture_radius = std::max(goal_xy_tol_, step_size_);
    if (distance_to_goal <= coarse_capture_radius &&
        isPathCollisionFree(current.x, current.y, gx, gy)) {
      const bool goal_pool_changed = add_goal_label(current_index);
      if (goal_pool_changed) {
        last_goal_pool_change_iteration = iteration;
        const int topology_count = count_goal_topologies();
        // Track the current retained pool, not only its historical maximum.
        // A later representative-label replacement can remove a topology; in
        // that case the search must not stop as though all passages remained.
        observed_goal_topologies = topology_count;
        observed_goal_signatures = count_goal_signatures();
      }

      // Raw goal-label count is not route diversity: nav_8 collected twelve
      // labels in 0.43 s, ten of which collapsed onto existing corridors.  An
      // Waiting for max_coarse_channels passages is wasteful when the map has
      // only two genuine topologies.  Stop after a minimum useful topology
      // set and a sufficiently rich goal-label pool have both remained
      // unchanged for the settling window. The maximum channel count remains
      // an output cap, not an assumption about map topology.
      const int stable_topology_target = std::min(
          max_coarse_channels_, coarse_min_stable_channels_);
      const bool multi_topology_settled =
          observed_goal_topologies >= stable_topology_target &&
          static_cast<int>(goal_labels.size()) >=
              coarse_min_goal_labels_for_settle_ &&
          last_goal_pool_change_iteration >= 0 &&
          iteration - last_goal_pool_change_iteration >=
              coarse_goal_settle_iterations_;

      // A map may contain only one useful topological passage.  Requiring two
      // passages then burns the whole time budget although the retained goal
      // Pareto pool has converged.  Use a longer settling window and require
      // the anchor frontier to be unable to improve the best retained scalar
      // goal cost by more than a small slack.  The risk/heading queues have
      // continued to run throughout that window, so this does not terminate
      // immediately after the first shortest-path arrival.
      double best_goal_cost = std::numeric_limits<double>::infinity();
      for (int goal_index : goal_labels)
        best_goal_cost = std::min(best_goal_cost, nodes[goal_index].g);
      const bool anchor_frontier_settled =
          std::isfinite(best_goal_cost) &&
          anchor_key >=
              (1.0 - coarse_single_topology_anchor_slack_) * best_goal_cost;
      const bool single_topology_settled =
          observed_goal_topologies == 1 &&
          // Once labels with different gate signatures have reached the goal,
          // keep searching until their actual geometry is resolved.  Without
          // this guard a spatial alternative can arrive just before the old
          // single-topology settling condition and never become a corridor.
          observed_goal_signatures <= 1 &&
          static_cast<int>(goal_labels.size()) >=
              coarse_min_goal_labels_for_settle_ &&
          last_goal_pool_change_iteration >= 0 &&
          iteration - last_goal_pool_change_iteration >=
              coarse_single_topology_settle_iterations_ &&
          anchor_frontier_settled;
      if (multi_topology_settled || single_topology_settled) {
        coarse_stop_reason = single_topology_settled
            ? "single_topology_stable_frontier"
            : "multi_topology_stable_pool";
        ROS_INFO("Pareto coarse search settled: topologies=%d signatures=%d "
                 "labels=%zu "
                 "stable_iterations=%d reason=%s anchor=%.3f goal=%.3f",
                 observed_goal_topologies, observed_goal_signatures,
                 goal_labels.size(),
                 single_topology_settled
                     ? coarse_single_topology_settle_iterations_
                     : coarse_goal_settle_iterations_,
                 coarse_stop_reason.c_str(), anchor_key, best_goal_cost);
        break;
      }
      continue;
    }

    std::vector<Node3D> successors;
    expand(current, successors);
    benchmark_generated_nodes_ += successors.size();
    // nodes.push_back() below may reallocate the vector, so do not retain a
    // reference into nodes while assigning every successor's signature.
    const Node3D topology_parent = current;
    for (Node3D& successor : successors) {
      successor.h = heuristic(successor.x, successor.y, gx, gy);
      update_topology_signature(topology_parent, successor);
      successor.parent = current_index;
      const int successor_index = static_cast<int>(nodes.size());
      nodes.push_back(successor);
      active.push_back(true);
      expanded.push_back(false);
      if (audit) ++audit->label_generated;
      if (insert_label(successor_index)) push_queues(successor_index);
    }
    benchmark_max_stored_nodes_ = std::max(
        benchmark_max_stored_nodes_,
        static_cast<std::uint64_t>(nodes.size()));
  }

  observed_goal_topologies = count_goal_topologies();
  observed_goal_signatures = count_goal_signatures();
  if (audit) {
    audit->goal_labels = static_cast<int>(goal_labels.size());
    audit->goal_topology_count = observed_goal_topologies;
    audit->goal_signature_count = observed_goal_signatures;
    audit->coarse_stop_iteration = processed_iterations;
    audit->coarse_stop_reason = coarse_stop_reason;
  }
  benchmark_max_stored_nodes_ = std::max(
      benchmark_max_stored_nodes_,
      static_cast<std::uint64_t>(nodes.size()));

  std::vector<std::vector<geometry_msgs::PoseStamped>> raw_plans;
  std::vector<PathMetrics> raw_metrics;
  std::vector<std::uint32_t> raw_signature_keys;
  for (int goal_index : goal_labels) {
    std::vector<geometry_msgs::PoseStamped> path;
    buildPath(nodes, goal_index, frame, stamp, path);
    const Node3D& terminal = nodes[goal_index];
    const double gap = std::hypot(terminal.x - gx, terminal.y - gy);
    if (gap > 1e-4) {
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = frame;
      goal_pose.header.stamp = stamp;
      goal_pose.pose.position.x = gx;
      goal_pose.pose.position.y = gy;
      goal_pose.pose.position.z = getDEMZ(gx, gy);
      tf2::Quaternion goal_q;
      goal_q.setRPY(0.0, 0.0, gt);
      goal_pose.pose.orientation = tf2::toMsg(goal_q);
      path.push_back(goal_pose);
    }
    raw_metrics.push_back(evaluatePathMetrics(path));
    raw_signature_keys.push_back(topology_signature_key(terminal));
    raw_plans.push_back(std::move(path));
  }

  if (!raw_plans.empty()) {
    double shortest_length = std::numeric_limits<double>::infinity();
    for (const PathMetrics& item : raw_metrics)
      shortest_length = std::min(shortest_length, item.length);
    const double length_limit =
        shortest_length * (1.0 + coarse_max_detour_ratio_);
    std::vector<bool> keep(raw_plans.size(), true);
    for (size_t index = 0; index < raw_plans.size(); ++index) {
      if (raw_metrics[index].length > length_limit + 1e-9) {
        keep[index] = false;
        if (audit) ++audit->detour_rejections;
      }
    }
    for (size_t lhs = 0; lhs < raw_plans.size(); ++lhs) {
      if (!keep[lhs]) continue;
      for (size_t rhs = 0; rhs < raw_plans.size(); ++rhs) {
        if (lhs == rhs || !keep[rhs]) continue;
        // A coarse approximation that is currently objective-dominated may
        // still refine into a valuable route inside its independent corridor.
        // Apply final coarse dominance only within the same spatial signature;
        // geometric duplicate filtering after this function remains the final
        // guard against artificial extra channels.
        if (coarse_topology_signature_enabled_ &&
            raw_signature_keys[lhs] != raw_signature_keys[rhs]) {
          continue;
        }
        const PathMetrics& a = raw_metrics[rhs];
        const PathMetrics& b = raw_metrics[lhs];
        const bool no_worse =
            a.length <= b.length + 1e-9 &&
            a.combined_risk_objective <=
                b.combined_risk_objective + 1e-9 &&
            a.pitch_objective <= b.pitch_objective + 1e-9 &&
            a.roll_objective <= b.roll_objective + 1e-9;
        const bool strictly_better =
            a.length < b.length - 1e-9 ||
            a.combined_risk_objective <
                b.combined_risk_objective - 1e-9 ||
            a.pitch_objective < b.pitch_objective - 1e-9 ||
            a.roll_objective < b.roll_objective - 1e-9;
        if (no_worse && strictly_better) {
          keep[lhs] = false;
          if (audit) ++audit->label_dominance_pruned;
          break;
        }
      }
    }
    std::vector<size_t> order;
    for (size_t index = 0; index < keep.size(); ++index)
      if (keep[index]) order.push_back(index);
    std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
      return raw_metrics[lhs].length < raw_metrics[rhs].length;
    });
    for (size_t index : order) plans.push_back(std::move(raw_plans[index]));
  }

  ROS_INFO("Pareto coarse MHA*: success=%s plans=%zu goal_labels=%zu "
           "goal_topologies=%d goal_signatures=%d "
           "iterations=%d nodes=%zu expansions=(%d,%d,%d) "
           "label_pruned=(dominance:%d,cap:%d) time=%.3fs stop=%s",
           plans.empty() ? "false" : "true", plans.size(),
           goal_labels.size(), observed_goal_topologies,
           audit ? audit->goal_signature_count : 0,
           processed_iterations, nodes.size(),
           queue_expansions[0], queue_expansions[1], queue_expansions[2],
           audit ? audit->label_dominance_pruned : 0,
           audit ? audit->label_cap_pruned : 0,
           (ros::WallTime::now() - search_start).toSec(),
           coarse_stop_reason.c_str());

  step_size_ = saved_step;
  terrain_cost_weight_ = saved_terrain_weight;
  active_high_risk_weight_ = saved_high_risk_weight;
  active_pointwise_risk_weight_ = saved_pointwise_weight;
  active_attitude_risk_scale_ = saved_attitude_scale;
  active_diversity_weight_ = saved_diversity_weight;
  return !plans.empty();
}

void HybridAStarPlanner::publishPlannerStatistics(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    const std::vector<geometry_msgs::PoseStamped>& plan,
    bool success, const std::string& failure_reason,
    int coarse_channels, int fine_candidates, int selected_candidate,
    double coarse_time_s) const {
  const double planning_time_ms = benchmark_plan_start_.isZero()
      ? 0.0
      : (ros::WallTime::now() - benchmark_plan_start_).toSec() * 1000.0;

  double length = 0.0;
  double risk_integral = 0.0;
  double peak_risk = 0.0;
  for (size_t index = 1; index < plan.size(); ++index) {
    const double dx = plan[index].pose.position.x -
        plan[index - 1].pose.position.x;
    const double dy = plan[index].pose.position.y -
        plan[index - 1].pose.position.y;
    const double ds = std::hypot(dx, dy);
    if (ds <= 1e-9) continue;
    const double yaw = std::atan2(dy, dx);
    const double risk = queryTerrainCost(
        0.5 * (plan[index].pose.position.x +
               plan[index - 1].pose.position.x),
        0.5 * (plan[index].pose.position.y +
               plan[index - 1].pose.position.y), yaw);
    length += ds;
    risk_integral += risk * ds;
    peak_risk = std::max(peak_risk, risk);
  }
  const PathMetrics final_metrics = evaluatePathMetrics(plan);

  std::ostringstream json;
  json << std::fixed << std::setprecision(9)
       << "{\"schema_version\":1"
       << ",\"algorithm\":\""
       << (basic_mode_ ? "legacy_basic_hybrid_astar"
                       : "multi_corridor_pareto_hybrid_astar")
       << "\""
       << ",\"uses_terrain_risk\":"
       << (basic_mode_ ? "false" : "true")
       << ",\"success\":" << (success ? "true" : "false")
       << ",\"failure_reason\":\"" << failure_reason << "\""
       << ",\"stamp_s\":" << ros::Time::now().toSec()
       << ",\"start_x\":" << start.pose.position.x
       << ",\"start_y\":" << start.pose.position.y
       << ",\"start_yaw\":" << tf2::getYaw(start.pose.orientation)
       << ",\"goal_x\":" << goal.pose.position.x
       << ",\"goal_y\":" << goal.pose.position.y
       << ",\"goal_yaw\":" << tf2::getYaw(goal.pose.orientation)
       << ",\"planning_time_ms\":" << planning_time_ms
       << ",\"search_calls\":" << benchmark_search_calls_
       << ",\"iterations\":" << benchmark_iterations_
       << ",\"expanded_nodes\":" << benchmark_expanded_nodes_
       << ",\"generated_nodes\":" << benchmark_generated_nodes_
       << ",\"coarse_search_calls\":"
       << benchmark_coarse_search_calls_
       << ",\"coarse_iterations\":" << benchmark_coarse_iterations_
       << ",\"coarse_expanded_nodes\":"
       << benchmark_coarse_expanded_nodes_
       << ",\"coarse_generated_nodes\":"
       << benchmark_coarse_generated_nodes_
       << ",\"max_stored_nodes\":" << benchmark_max_stored_nodes_
       << ",\"primitive_attempts\":" << benchmark_primitive_attempts_
       << ",\"collision_checks\":" << benchmark_collision_checks_
       << ",\"collision_rejections\":"
       << benchmark_collision_rejections_
       << ",\"length_bound_pruned_nodes\":"
       << benchmark_length_bound_pruned_nodes_
       << ",\"fine_heuristic_weight\":"
       << ((!basic_mode_ && pareto_terrain_mode_)
               ? fine_heuristic_weight_ : 1.0)
       << ",\"fine_curvature_change_weight\":"
       << ((!basic_mode_ && pareto_terrain_mode_)
               ? fine_curvature_change_weight_ : 0.0)
       << ",\"curvature_change_augments_state\":false"
       << ",\"primitive_collision_check_fraction\":"
       << ((!basic_mode_ && pareto_terrain_mode_)
               ? primitive_collision_check_fraction_ : 0.5)
       << ",\"path_points\":" << plan.size()
       << ",\"path_length_m\":" << length
       << ",\"terrain_cost_integral\":" << risk_integral
       << ",\"mean_terrain_risk\":"
       << (length > 1e-9 ? risk_integral / length : 0.0)
       << ",\"peak_terrain_risk\":" << peak_risk
       << ",\"mean_abs_pitch_deg\":"
       << final_metrics.mean_abs_pitch_deg
       << ",\"p95_abs_pitch_deg\":"
       << final_metrics.p95_abs_pitch_deg
       << ",\"max_abs_pitch_deg\":"
       << final_metrics.max_abs_pitch_deg
       << ",\"pitch_exposure_fraction\":"
       << final_metrics.pitch_exposure_fraction
       << ",\"pitch_objective\":" << final_metrics.pitch_objective
       << ",\"mean_abs_roll_deg\":"
       << final_metrics.mean_abs_roll_deg
       << ",\"p95_abs_roll_deg\":"
       << final_metrics.p95_abs_roll_deg
       << ",\"max_abs_roll_deg\":"
       << final_metrics.max_abs_roll_deg
       << ",\"roll_exposure_fraction\":"
       << final_metrics.roll_exposure_fraction
       << ",\"roll_objective\":" << final_metrics.roll_objective
       << ",\"coarse_channels\":" << coarse_channels
       << ",\"fine_candidates\":" << fine_candidates
       << ",\"selected_candidate\":" << selected_candidate
       << ",\"coarse_time_ms\":" << 1000.0 * coarse_time_s
       << "}";
  std_msgs::String message;
  message.data = json.str();
  planner_statistics_pub_.publish(message);
}

// ==================== Two-level makePlan ====================

bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) { ROS_ERROR("HybridAStarPlanner not initialized"); return false; }
  benchmark_plan_start_ = ros::WallTime::now();
  benchmark_search_calls_ = 0;
  benchmark_iterations_ = 0;
  benchmark_expanded_nodes_ = 0;
  benchmark_generated_nodes_ = 0;
  benchmark_max_stored_nodes_ = 0;
  benchmark_coarse_search_calls_ = 0;
  benchmark_coarse_iterations_ = 0;
  benchmark_coarse_expanded_nodes_ = 0;
  benchmark_coarse_generated_nodes_ = 0;
  benchmark_primitive_attempts_ = 0;
  benchmark_collision_checks_ = 0;
  benchmark_collision_rejections_ = 0;
  benchmark_length_bound_pruned_nodes_ = 0;
  // Coarse search compares terrain channels and must not carry a previous
  // steering state.  The curvature-change term is enabled only after the
  // independent corridors have been constructed for fine trajectory search.
  active_curvature_change_weight_ = 0.0;
  active_search_max_path_length_ =
      std::numeric_limits<double>::infinity();
  plan.clear();
  holonomic_heuristic_field_.clear();
  costmap_ = costmap_ros_->getCostmap();
  buildClearanceMap();

  std::string frame = costmap_ros_->getGlobalFrameID();
  ros::Time stamp = ros::Time::now();

  double sx = start.pose.position.x, sy = start.pose.position.y;
  double st = mod2pi(tf2::getYaw(start.pose.orientation));
  double gx = goal.pose.position.x,  gy = goal.pose.position.y;
  double gt = mod2pi(tf2::getYaw(goal.pose.orientation));

  if (!isValid(sx, sy)) {
    ROS_WARN("Hybrid A*: start in obstacle");
    publishPlannerStatistics(start, goal, plan, false,
                             "start_not_traversable", 0, 0, -1, 0.0);
    return false;
  }
  if (!isValid(gx, gy)) {
    ROS_WARN("Hybrid A*: goal in obstacle");
    publishPlannerStatistics(start, goal, plan, false,
                             "goal_not_traversable", 0, 0, -1, 0.0);
    return false;
  }

  const double final_gx = gx;
  const double final_gy = gy;
  const double start_goal_distance =
      std::hypot(final_gx - sx, final_gy - sy);

  // Reserve terminal manoeuvring space behind the commanded goal yaw. Hybrid
  // A* is softly biased toward that tangent while approaching the pre-goal;
  // after search, a curvature-continuous Hermite transition uses this space to
  // reach the exact position and yaw.  No discrete lattice node is required to
  // equal the final yaw, and the controller does not need an in-place turn.
  // Shrink the reserved region only when bounds or clearance require it.
  double terminal_approach_distance = std::min(
      terminal_approach_distance_,
      std::max(0.0, start_goal_distance - start_straight_distance_ -
                        2.0 * goal_xy_tol_));
  const double terminal_check_step =
      std::max(0.05, 0.5 * costmap_->getResolution());
  auto terminal_segment_is_clear = [&](double distance) {
    const int samples = std::max(
        1, static_cast<int>(std::ceil(distance / terminal_check_step)));
    for (int i = 0; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / samples;
      const double x = final_gx -
          (1.0 - ratio) * distance * std::cos(gt);
      const double y = final_gy -
          (1.0 - ratio) * distance * std::sin(gt);
      unsigned int mx, my;
      if (!costmap_->worldToMap(x, y, mx, my)) return false;
      if (!clearance_map_.empty() &&
          !clearance_map_[my * cm_width_ + mx]) return false;
      if (!isValid(x, y)) return false;
    }
    return true;
  };
  while (terminal_approach_distance > terminal_check_step &&
         !terminal_segment_is_clear(terminal_approach_distance)) {
    terminal_approach_distance -= terminal_check_step;
  }
  if (terminal_approach_distance <= terminal_check_step ||
      !terminal_segment_is_clear(terminal_approach_distance)) {
    terminal_approach_distance = 0.0;
  }

  gx = final_gx - terminal_approach_distance * std::cos(gt);
  gy = final_gy - terminal_approach_distance * std::sin(gt);
  ROS_INFO("Terminal approach: search_goal=(%.2f,%.2f), final=(%.2f,%.2f), "
           "yaw=%.1fdeg straight=%.2fm",
           gx, gy, final_gx, final_gy, gt * 180.0 / M_PI,
           terminal_approach_distance);

  // Do not let the search choose a different heading at the vehicle's exact
  // start pose.  Move the lattice-search origin a short distance along the
  // measured vehicle yaw, then prepend that straight segment to every path.
  // The searched part starts with the same clamped tangent, so the join is C1
  // continuous: position and heading are continuous, without an in-place
  // rotation at the beginning of the global plan.
  const double straight_distance = std::min(
      start_straight_distance_,
      std::max(0.0, start_goal_distance - goal_xy_tol_));
  const double search_sx = sx + straight_distance * std::cos(st);
  const double search_sy = sy + straight_distance * std::sin(st);

  std::vector<geometry_msgs::PoseStamped> start_prefix;
  if (straight_distance > 1e-4) {
    const double sample_step =
        std::max(0.05, 0.5 * costmap_->getResolution());
    const int sample_count =
        std::max(1, static_cast<int>(std::ceil(
                        straight_distance / sample_step)));
    start_prefix.reserve(sample_count + 1);
    for (int i = 0; i <= sample_count; ++i) {
      const double ratio = static_cast<double>(i) / sample_count;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame;
      pose.header.stamp = stamp;
      pose.pose.position.x = sx + ratio * straight_distance * std::cos(st);
      pose.pose.position.y = sy + ratio * straight_distance * std::sin(st);
      pose.pose.position.z =
          getDEMZ(pose.pose.position.x, pose.pose.position.y);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, st);
      pose.pose.orientation = tf2::toMsg(q);
      start_prefix.push_back(pose);
    }
  }

  auto prepend_start_prefix = [&start_prefix](
      std::vector<geometry_msgs::PoseStamped>& path) {
    if (start_prefix.empty() || path.empty()) return;
    std::vector<geometry_msgs::PoseStamped> joined;
    joined.reserve(start_prefix.size() + path.size() - 1);
    joined.insert(joined.end(), start_prefix.begin(), start_prefix.end());
    // runSearch starts at the last prefix pose. Skip that duplicate pose so
    // downstream CTE and path-length calculations see one continuous path.
    joined.insert(joined.end(), std::next(path.begin()), path.end());
    path.swap(joined);
  };

  // Coarse search is responsible only for identifying a topological terrain
  // channel.  Extend its centreline to the commanded goal so the corridor
  // mask covers the complete terminal region, but do not reject a coarse
  // channel using fine-layer curvature constraints.  The actual trajectory
  // receives the curvature-checked Hermite transition after fine search.
  auto append_coarse_terminal_extension = [&](
      std::vector<geometry_msgs::PoseStamped>& path) -> bool {
    if (path.empty()) return false;
    tf2::Quaternion goal_q;
    goal_q.setRPY(0.0, 0.0, gt);
    path.back().pose.orientation = tf2::toMsg(goal_q);
    if (terminal_approach_distance <= 1e-4) return true;

    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(
               terminal_approach_distance / terminal_check_step)));
    path.reserve(path.size() + sample_count);
    for (int i = 1; i <= sample_count; ++i) {
      const double ratio = static_cast<double>(i) / sample_count;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame;
      pose.header.stamp = stamp;
      pose.pose.position.x =
          gx + ratio * terminal_approach_distance * std::cos(gt);
      pose.pose.position.y =
          gy + ratio * terminal_approach_distance * std::sin(gt);
      if (!isTraversable(pose.pose.position.x, pose.pose.position.y))
        return false;
      pose.pose.position.z =
          getDEMZ(pose.pose.position.x, pose.pose.position.y);
      pose.pose.orientation = tf2::toMsg(goal_q);
      path.push_back(pose);
    }
    path.back().pose.position.x = final_gx;
    path.back().pose.position.y = final_gy;
    path.back().pose.position.z = getDEMZ(final_gx, final_gy);
    return true;
  };

  // A corridor is a search accelerator, not a physical obstacle. The final
  // tangent may legitimately leave its coarse mask by a few cells while still
  // remaining collision-free. Validate terminal geometry against map bounds
  // and the inflated vehicle-clearance map, independently of use_corridor_.
  auto is_terminal_physically_traversable =
      [&](double x, double y) -> bool {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) return false;
    if (!clearance_map_.empty())
      return static_cast<bool>(clearance_map_[my * cm_width_ + mx]);
    return isValid(x, y);
  };

  std::string terminal_transition_failure_status =
      "terminal_transition_failed";
  auto append_terminal_approach = [&](
      std::vector<geometry_msgs::PoseStamped>& path) -> bool {
    terminal_transition_failure_status = "terminal_transition_failed";
    if (path.empty()) return false;
    tf2::Quaternion goal_q;
    goal_q.setRPY(0.0, 0.0, gt);
    if (terminal_approach_distance <= 1e-4) {
      path.back().pose.orientation = tf2::toMsg(goal_q);
      return true;
    }

    // Replace the old fixed straight segment with a tangent-matched cubic
    // Hermite transition. Search may arrive at the pre-alignment point with a
    // residual heading error; a straight append turned that error into a hard
    // corner whenever whole-path spline smoothing was rejected.
    const double curvature_limit =
        min_turning_radius_ > 1e-3
            ? 1.10 / min_turning_radius_
            : std::numeric_limits<double>::infinity();
    const double physical_backtrack =
        terminal_approach_distance +
        2.0 * std::max(0.0, min_turning_radius_);
    const double backtrack_limit = std::max(
        terminal_transition_max_backtrack_, physical_backtrack);
    const std::vector<double> tangent_scale_multipliers = {
        0.65, 0.85, 1.0, 1.20, 1.50};
    double removed_length = 0.0;
    int tested_transitions = 0;
    int obstacle_rejections = 0;
    int curvature_rejections = 0;
    int singular_rejections = 0;
    double best_rejected_curvature =
        std::numeric_limits<double>::infinity();
    for (size_t splice = path.size(); splice-- > 1;) {
      if (splice + 1 < path.size()) {
        removed_length += std::hypot(
            path[splice + 1].pose.position.x -
                path[splice].pose.position.x,
            path[splice + 1].pose.position.y -
                path[splice].pose.position.y);
      }
      if (removed_length > backtrack_limit + 1e-9)
        break;

      const double x0 = path[splice].pose.position.x;
      const double y0 = path[splice].pose.position.y;
      const double incoming_dx =
          x0 - path[splice - 1].pose.position.x;
      const double incoming_dy =
          y0 - path[splice - 1].pose.position.y;
      if (std::hypot(incoming_dx, incoming_dy) < 1e-4) continue;
      const double incoming_yaw = std::atan2(incoming_dy, incoming_dx);
      const double chord = std::hypot(final_gx - x0, final_gy - y0);
      if (chord < 1e-4) continue;

      for (double scale_multiplier : tangent_scale_multipliers) {
        ++tested_transitions;
        const double tangent_scale =
            terminal_transition_tangent_scale_ * scale_multiplier;
        const double tangent_length = tangent_scale * chord;
        const double m0x = tangent_length * std::cos(incoming_yaw);
        const double m0y = tangent_length * std::sin(incoming_yaw);
        const double m1x = tangent_length * std::cos(gt);
        const double m1y = tangent_length * std::sin(gt);
        const int sample_count = std::max(
            10, static_cast<int>(std::ceil(
                    2.0 * chord / terminal_check_step)));

        std::vector<geometry_msgs::PoseStamped> transition;
        transition.reserve(sample_count + 1);
        bool valid = true;
        double max_curvature = 0.0;
        for (int i = 0; i <= sample_count; ++i) {
          const double t = static_cast<double>(i) / sample_count;
          const double t2 = t * t;
          const double t3 = t2 * t;
          const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
          const double h10 = t3 - 2.0 * t2 + t;
          const double h01 = -2.0 * t3 + 3.0 * t2;
          const double h11 = t3 - t2;
          const double px =
              h00 * x0 + h10 * m0x + h01 * final_gx + h11 * m1x;
          const double py =
              h00 * y0 + h10 * m0y + h01 * final_gy + h11 * m1y;
          const double dx =
              (6.0 * t2 - 6.0 * t) * x0 +
              (3.0 * t2 - 4.0 * t + 1.0) * m0x +
              (-6.0 * t2 + 6.0 * t) * final_gx +
              (3.0 * t2 - 2.0 * t) * m1x;
          const double dy =
              (6.0 * t2 - 6.0 * t) * y0 +
              (3.0 * t2 - 4.0 * t + 1.0) * m0y +
              (-6.0 * t2 + 6.0 * t) * final_gy +
              (3.0 * t2 - 2.0 * t) * m1y;
          const double ddx =
              (12.0 * t - 6.0) * x0 + (6.0 * t - 4.0) * m0x +
              (-12.0 * t + 6.0) * final_gx +
              (6.0 * t - 2.0) * m1x;
          const double ddy =
              (12.0 * t - 6.0) * y0 + (6.0 * t - 4.0) * m0y +
              (-12.0 * t + 6.0) * final_gy +
              (6.0 * t - 2.0) * m1y;
          const double speed_sq = dx * dx + dy * dy;
          if (speed_sq < 1e-10) {
            ++singular_rejections;
            valid = false;
            break;
          }
          if (!is_terminal_physically_traversable(px, py)) {
            ++obstacle_rejections;
            valid = false;
            break;
          }
          const double curvature =
              std::fabs(dx * ddy - dy * ddx) /
              std::pow(speed_sq, 1.5);
          max_curvature = std::max(max_curvature, curvature);
          if (max_curvature > curvature_limit) {
            best_rejected_curvature = std::min(
                best_rejected_curvature, max_curvature);
            ++curvature_rejections;
            valid = false;
            break;
          }

          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = frame;
          pose.header.stamp = stamp;
          pose.pose.position.x = px;
          pose.pose.position.y = py;
          pose.pose.position.z = getDEMZ(px, py);
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, std::atan2(dy, dx));
          pose.pose.orientation = tf2::toMsg(q);
          transition.push_back(pose);
        }
        if (!valid) continue;

        // The analytic Hermite curvature above does not cover the discrete
        // three-point stencil at the splice with the existing lattice path.
        // The evaluator and MPC consume the published polyline, so validate
        // the exact same heading-change / arc-support quantity here.  This
        // closes the gap that produced isolated 1.5--2.3 1/m spikes roughly
        // one terminal-approach distance before the goal.
        double max_discrete_curvature = 0.0;
        auto update_discrete_curvature = [&](double ax, double ay,
                                             double bx, double by,
                                             double cx, double cy) {
          const double first_length = std::hypot(bx - ax, by - ay);
          const double second_length = std::hypot(cx - bx, cy - by);
          const double support = 0.5 * (first_length + second_length);
          if (first_length < 1e-6 || second_length < 1e-6 ||
              support < 1e-6) {
            return;
          }
          const double first_heading = std::atan2(by - ay, bx - ax);
          const double second_heading = std::atan2(cy - by, cx - bx);
          max_discrete_curvature = std::max(
              max_discrete_curvature,
              std::fabs(angleDiff(second_heading, first_heading)) / support);
        };
        // Curvature at the last untouched lattice sample uses the segment
        // before it and the first segment into the Hermite splice.  This is a
        // different stencil from curvature at the splice itself; omitting it
        // caused the reproducible medium_01/long_03 pre-terminal spikes.
        if (splice >= 2 && !transition.empty()) {
          update_discrete_curvature(
              path[splice - 2].pose.position.x,
              path[splice - 2].pose.position.y,
              path[splice - 1].pose.position.x,
              path[splice - 1].pose.position.y,
              transition[0].pose.position.x,
              transition[0].pose.position.y);
        }
        // Curvature at the splice uses the incoming lattice segment and the
        // first non-zero Hermite segment.
        if (transition.size() >= 2) {
          update_discrete_curvature(
              path[splice - 1].pose.position.x,
              path[splice - 1].pose.position.y,
              transition[0].pose.position.x,
              transition[0].pose.position.y,
              transition[1].pose.position.x,
              transition[1].pose.position.y);
        }
        for (size_t i = 1; i + 1 < transition.size(); ++i) {
          update_discrete_curvature(
              transition[i - 1].pose.position.x,
              transition[i - 1].pose.position.y,
              transition[i].pose.position.x,
              transition[i].pose.position.y,
              transition[i + 1].pose.position.x,
              transition[i + 1].pose.position.y);
        }
        max_curvature = std::max(max_curvature, max_discrete_curvature);
        if (max_discrete_curvature > curvature_limit) {
          best_rejected_curvature = std::min(
              best_rejected_curvature, max_discrete_curvature);
          ++curvature_rejections;
          continue;
        }

        path.resize(splice + 1);
        path.back().pose.orientation = transition.front().pose.orientation;
        path.insert(path.end(), std::next(transition.begin()),
                    transition.end());
        path.back().pose.position.x = final_gx;
        path.back().pose.position.y = final_gy;
        path.back().pose.position.z = getDEMZ(final_gx, final_gy);
        path.back().pose.orientation = tf2::toMsg(goal_q);
        ROS_INFO("Terminal Hermite transition: chord=%.2fm backtrack=%.2fm "
                 "heading_change=%.1fdeg tangent_scale=%.2f "
                 "max_curvature=%.3f/%.3f 1/m discrete=%.3f tested=%d",
                 chord, removed_length,
                 std::fabs(angleDiff(gt, incoming_yaw)) * 180.0 / M_PI,
                 tangent_scale, max_curvature, curvature_limit,
                 max_discrete_curvature,
                 tested_transitions);
        return true;
      }
    }

    const double rejected_curvature_log =
        std::isfinite(best_rejected_curvature)
            ? best_rejected_curvature : -1.0;
    if (obstacle_rejections > 0 && curvature_rejections == 0 &&
        singular_rejections == 0) {
      terminal_transition_failure_status = "terminal_obstacle_failed";
    } else if (curvature_rejections > 0 && obstacle_rejections == 0 &&
               singular_rejections == 0) {
      terminal_transition_failure_status = "terminal_curvature_failed";
    } else if (singular_rejections > 0 && obstacle_rejections == 0 &&
               curvature_rejections == 0) {
      terminal_transition_failure_status = "terminal_singular_failed";
    } else {
      terminal_transition_failure_status = "terminal_mixed_failed";
    }
    ROS_WARN("Terminal transition rejected after %d trials: "
             "physical_obstacle=%d curvature=%d singular=%d "
             "backtrack_limit=%.2fm curvature_limit=%.3f 1/m "
             "best_rejected_curvature=%.3f 1/m",
             tested_transitions, obstacle_rejections,
             curvature_rejections, singular_rejections,
             backtrack_limit, curvature_limit,
             rejected_curvature_log);
    return false;
  };

  ROS_INFO("Initial-heading constraint: straight %.2fm from yaw %.1fdeg, "
           "search starts at (%.2f, %.2f)",
           straight_distance, st * 180.0 / M_PI, search_sx, search_sy);

  const double saved_terrain_weight = terrain_cost_weight_;
  std::vector<std::vector<geometry_msgs::PoseStamped>> coarse_paths;
  std::vector<std::vector<bool>> coarse_corridors;
  // The adaptive corridor is not represented by the binary mask alone.  Each
  // channel also owns a distance-to-centre field used by the fine-search soft
  // constraint.  Keeping these arrays paired prevents channel N from silently
  // reusing the last generated corridor's centreline.
  std::vector<std::vector<float>> coarse_corridor_distances;
  std::vector<double> coarse_weights;
  std::vector<double> coarse_high_risk_weights;
  std::vector<std::string> coarse_roles;
  double fine_mean_risk_weight = 0.0;
  double fine_high_risk_weight = 0.0;
  double fine_pointwise_risk_weight = 0.0;
  CoarseSearchAudit coarse_audit;
  auto configure_fine_risk_objective = [&](const PathMetrics& reference) {
    if (!pareto_terrain_mode_ || fine_risk_strength_ <= 0.0) {
      fine_mean_risk_weight = 0.0;
      fine_high_risk_weight = 0.0;
      fine_pointwise_risk_weight = 0.0;
      coarse_audit.fine_mean_risk_weight = 0.0;
      coarse_audit.fine_high_risk_weight = 0.0;
      coarse_audit.fine_pointwise_risk_weight = 0.0;
      return;
    }

    // Fine search is not another topology search.  Its job is to improve the
    // realised route inside one corridor.  nav_23/nav_24 kept excellent peak
    // risk but left mean risk above the scalar risk-weighted baseline, so mean
    // risk and sustained threshold exposure now receive equal shares.  The
    // local r^p term remains unchanged to preserve isolated-peak avoidance.
    // Floors are derived from the current map/safety envelope.  In particular,
    // a zero-exposure reference must not disable the exposure term and permit
    // the fine path to enter a dangerous patch for free.
    const double mean_reference = std::max(
        reference.mean_risk, 0.25 * pareto_high_risk_threshold_);
    const double exposure_reference = std::max(
        reference.high_risk_fraction,
        std::max(1e-3, pareto_max_high_risk_fraction_));
    const double pointwise_reference = std::max(
        1e-3,
        std::pow(std::max(reference.peak_risk,
                          pareto_high_risk_threshold_),
                 pointwise_risk_exponent_));
    fine_mean_risk_weight =
        0.35 * fine_risk_strength_ / mean_reference;
    fine_high_risk_weight =
        0.35 * fine_risk_strength_ / exposure_reference;
    fine_pointwise_risk_weight =
        0.30 * fine_risk_strength_ / pointwise_reference;
    coarse_audit.fine_mean_risk_weight = fine_mean_risk_weight;
    coarse_audit.fine_high_risk_weight = fine_high_risk_weight;
    coarse_audit.fine_pointwise_risk_weight =
        fine_pointwise_risk_weight;
    ROS_INFO("Unified fine risk objective: mean=%.3f exposure=%.3f "
             "pointwise=%.3f threshold=%.3f strength=%.2f",
             fine_mean_risk_weight, fine_high_risk_weight,
             fine_pointwise_risk_weight, pareto_high_risk_threshold_,
             fine_risk_strength_);
  };
  const ros::WallTime coarse_phase_start = ros::WallTime::now();

  // Phase 1: coarse channel generation. The first single-queue search is the
  // shortest-path anchor. Later shared-state MHA* searches reuse one reverse
  // risk field and are softly repelled from already accepted channels.
  use_corridor_ = false;
  active_diversity_weight_ = 0.0;
  active_pointwise_risk_weight_ = 0.0;
  if (coarse_map_.loaded) {
    active_map_ = &coarse_map_;
    const double coarse_step = coarse_map_.res * 2.0;
    const int coarse_max_iter = std::max(1000, max_iterations_ / 4);
    const int channel_limit =
        (pareto_terrain_mode_ && !basic_mode_)
            ? max_coarse_channels_ : 1;
    const int coarse_pool_target =
        channel_limit > 1
            ? channel_limit + coarse_pool_extra_candidates_
            : channel_limit;
    if (channel_limit <= 1) {
      coarse_audit.risk_anchor_status = "disabled";
    } else if (!use_multi_heuristic_search_) {
      coarse_audit.risk_anchor_status = "multi_heuristic_disabled";
    }
    double effective_additional_budget =
        coarse_search_time_budget_ +
        coarse_search_time_per_meter_ * start_goal_distance;
    if (coarse_search_time_max_ > 0.0) {
      effective_additional_budget =
          std::min(effective_additional_budget, coarse_search_time_max_);
    }
    coarse_audit.time_budget_s = effective_additional_budget;
    clearDiversityPenalty();
    buildRiskHeuristicField(gx, gy);
    ROS_INFO("Phase 1: coarse channel search "
             "(%.2fm, step=%.2f, selected=%d, pool=%d, "
             "additional_budget=%.2fs)...",
             coarse_map_.res, coarse_step, channel_limit,
             coarse_pool_target,
             effective_additional_budget);

    bool used_multi_label_search = false;
    if (channel_limit > 1 && pareto_terrain_mode_ && !basic_mode_ &&
        use_multi_heuristic_search_ && coarse_multi_label_enabled_) {
      coarse_audit.time_budget_s = coarse_multi_label_time_limit_;
      std::vector<std::vector<geometry_msgs::PoseStamped>> label_paths;
      coarse_audit.risk_anchor_status = "multi_label_shared_search";
      coarse_audit.exposure_anchor_status = "multi_label_shared_search";
      if (runParetoCoarseSearch(
              search_sx, search_sy, st, gx, gy, gt, frame, stamp,
              label_paths, coarse_step, coarse_max_iter, &coarse_audit)) {
        std::vector<PathMetrics> accepted_metrics;
        accepted_metrics.reserve(label_paths.size());
        // Topology identity must be evaluated on the search-produced core
        // paths. The executable paths below receive the same initial-heading
        // prefix and terminal transition; including those common segments in
        // the overlap ratio made two genuinely different goal topologies
        // collapse into one channel on long routes (nav_10: 2 -> 1).
        std::vector<std::vector<geometry_msgs::PoseStamped>>
            topology_representatives;
        topology_representatives.reserve(label_paths.size());
        for (auto& candidate : label_paths) {
          const std::vector<geometry_msgs::PoseStamped> topology_core =
              candidate;
          if (!isDistinctPath(topology_core, topology_representatives)) {
            ++coarse_audit.duplicate_rejections;
            continue;
          }
          if (!append_coarse_terminal_extension(candidate)) {
            ++coarse_audit.failed_attempts;
            continue;
          }
          prepend_start_prefix(candidate);
          const PathMetrics metrics = evaluatePathMetrics(candidate);
          topology_representatives.push_back(topology_core);
          coarse_paths.push_back(std::move(candidate));
          accepted_metrics.push_back(metrics);
          coarse_weights.push_back(0.0);
          coarse_high_risk_weights.push_back(0.0);
          coarse_roles.push_back("pareto_label");
          buildCorridorMask(coarse_paths.back());
          coarse_corridors.push_back(corridor_mask_);
          coarse_corridor_distances.push_back(
              corridor_center_distance_field_);
        }

        if (!coarse_paths.empty()) {
          used_multi_label_search = true;
          coarse_audit.generated_pool_count =
              static_cast<int>(label_paths.size());
          size_t shortest_index = 0;
          size_t safest_index = 0;
          size_t lowest_roll_index = 0;
          auto aggregate_risk = [&](const PathMetrics& metrics) {
            return 0.35 * metrics.mean_risk +
                   0.25 * metrics.high_risk_fraction +
                   0.25 * metrics.peak_risk +
                   0.15 * metrics.tail_risk;
          };
          for (size_t index = 1; index < accepted_metrics.size(); ++index) {
            if (accepted_metrics[index].length <
                accepted_metrics[shortest_index].length) {
              shortest_index = index;
            }
            const double risk = aggregate_risk(accepted_metrics[index]);
            const double safest_risk =
                aggregate_risk(accepted_metrics[safest_index]);
            if (risk < safest_risk - 1e-9 ||
                (std::fabs(risk - safest_risk) <= 1e-9 &&
                 accepted_metrics[index].length <
                     accepted_metrics[safest_index].length)) {
              safest_index = index;
            }
            if (accepted_metrics[index].max_abs_roll_deg <
                    accepted_metrics[lowest_roll_index].max_abs_roll_deg -
                        1e-9 ||
                (std::fabs(
                     accepted_metrics[index].max_abs_roll_deg -
                     accepted_metrics[lowest_roll_index].max_abs_roll_deg) <=
                     1e-9 &&
                 accepted_metrics[index].length <
                     accepted_metrics[lowest_roll_index].length)) {
              lowest_roll_index = index;
            }
          }
          coarse_roles[shortest_index] = "pareto_shortest_label";
          if (safest_index != shortest_index)
            coarse_roles[safest_index] = "pareto_safest_label";
          if (lowest_roll_index != shortest_index &&
              lowest_roll_index != safest_index)
            coarse_roles[lowest_roll_index] = "pareto_low_roll_label";

          const PathMetrics& shortest_metrics =
              accepted_metrics[shortest_index];
          configure_fine_risk_objective(shortest_metrics);
          ROS_INFO("Shared Pareto coarse search retained %zu/%zu "
                   "topologically distinct labels",
                   coarse_paths.size(), label_paths.size());
        }
      }
      if (!used_multi_label_search) {
        coarse_audit.generated_pool_count = 0;
        coarse_audit.risk_anchor_status =
            "multi_label_failed_legacy_fallback";
        coarse_audit.exposure_anchor_status =
            "multi_label_failed_legacy_fallback";
        ROS_WARN("Shared Pareto coarse search produced no usable channel; "
                 "falling back to legacy weighted searches");
      }
    }

    if (!used_multi_label_search) {
      std::vector<geometry_msgs::PoseStamped> shortest_path;
      terrain_cost_weight_ = 0.0;
      if (runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                  shortest_path, coarse_step, coarse_max_iter, false) &&
          append_coarse_terminal_extension(shortest_path)) {
      prepend_start_prefix(shortest_path);
      coarse_paths.push_back(shortest_path);
      coarse_weights.push_back(0.0);
      coarse_high_risk_weights.push_back(0.0);
      coarse_roles.push_back("shortest_anchor");
      buildCorridorMask(shortest_path);
      coarse_corridors.push_back(corridor_mask_);
      coarse_corridor_distances.push_back(corridor_center_distance_field_);
      addDiversityPenalty(shortest_path);
      const PathMetrics shortest_metrics =
          evaluatePathMetrics(shortest_path);
      const double max_coarse_length =
          shortest_metrics.length * (1.0 + coarse_max_detour_ratio_);
      configure_fine_risk_objective(shortest_metrics);
      ROS_INFO("Coarse channel 1: shortest anchor, length=%.2fm "
               "mean_risk=%.3f peak_risk=%.3f",
               shortest_metrics.length, shortest_metrics.mean_risk,
               shortest_metrics.peak_risk);

      if (channel_limit > 1 && use_multi_heuristic_search_ &&
          shortest_metrics.mean_risk >= 1e-4) {
        // Build objective anchors from the measured shortest-route metrics.
        // This normalization is map/goal adaptive: each anchor's terrain term
        // has the same order of magnitude as route length, then the common
        // dimensionless strength pushes it toward the corresponding extreme.
        const double mean_risk_weight =
            pareto_anchor_strength_ /
            std::max(1e-3, shortest_metrics.mean_risk);
        const double exposure_weight =
            shortest_metrics.high_risk_fraction >= 1e-4
                ? pareto_anchor_strength_ /
                      shortest_metrics.high_risk_fraction
                : 0.0;

        // Always compute a risk-guided anchor before applying any diversity
        // repulsion or wall-time budget.  Previously the first alternative was
        // simultaneously asked to reduce terrain risk and escape the shortest
        // path's diversity field.  On long routes that search often exhausted
        // its reduced iteration allowance, leaving Pareto mode with only the
        // zero-risk-weight shortest candidate.  The two Pareto extremes are
        // now generated independently: shortest first, low-risk second.
        terrain_cost_weight_ = mean_risk_weight;
        active_high_risk_weight_ = 0.0;
        active_diversity_weight_ = 0.0;
        std::vector<geometry_msgs::PoseStamped> risk_anchor;
        ++coarse_audit.risk_anchor_attempts;
        const int risk_anchor_max_iter =
            std::max(coarse_max_iter, max_iterations_ / 2);
        if (runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                      risk_anchor, coarse_step, risk_anchor_max_iter, true) &&
            append_coarse_terminal_extension(risk_anchor)) {
          prepend_start_prefix(risk_anchor);
          const PathMetrics risk_metrics = evaluatePathMetrics(risk_anchor);
          if (risk_metrics.length > max_coarse_length + 1e-9) {
            coarse_audit.risk_anchor_status = "rejected_detour";
            ++coarse_audit.detour_rejections;
            ROS_INFO("Low-risk anchor rejected as excessive detour: "
                     "length=%.2fm limit=%.2fm",
                     risk_metrics.length, max_coarse_length);
          } else if (isDistinctPath(risk_anchor, coarse_paths)) {
            coarse_paths.push_back(risk_anchor);
            coarse_weights.push_back(mean_risk_weight);
            coarse_high_risk_weights.push_back(0.0);
            coarse_roles.push_back("mean_risk_anchor");
            buildCorridorMask(risk_anchor);
            coarse_corridors.push_back(corridor_mask_);
            coarse_corridor_distances.push_back(
                corridor_center_distance_field_);
            addDiversityPenalty(risk_anchor);
            coarse_audit.risk_anchor_status = "accepted";
            ROS_INFO("Coarse channel %zu: low-risk anchor, length=%.2fm "
                     "mean_risk=%.3f peak_risk=%.3f weight=%.3f",
                     coarse_paths.size(), risk_metrics.length,
                     risk_metrics.mean_risk, risk_metrics.peak_risk,
                     mean_risk_weight);
          } else {
            coarse_audit.risk_anchor_status = "duplicate";
            ++coarse_audit.duplicate_rejections;
            ROS_INFO("Low-risk anchor rejected as duplicate: length=%.2fm "
                     "mean_risk=%.3f peak_risk=%.3f",
                     risk_metrics.length, risk_metrics.mean_risk,
                     risk_metrics.peak_risk);
          }
        } else {
          coarse_audit.risk_anchor_status = "failed";
          ++coarse_audit.failed_attempts;
          ROS_WARN("Low-risk coarse anchor failed; diversity search will "
                   "continue from the shortest anchor");
        }

        // A second terrain extreme minimizes time spent above the map-adaptive
        // high-risk threshold.  It is intentionally separate from mean risk:
        // a short severe patch and a long moderate slope need not rank alike.
        if (static_cast<int>(coarse_paths.size()) < coarse_pool_target &&
            exposure_weight > 0.0) {
          terrain_cost_weight_ = 0.0;
          active_high_risk_weight_ = exposure_weight;
          active_diversity_weight_ = 0.0;
          std::vector<geometry_msgs::PoseStamped> exposure_anchor;
          ++coarse_audit.exposure_anchor_attempts;
          const int exposure_anchor_max_iter =
              std::max(coarse_max_iter, max_iterations_ / 2);
          if (runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                        exposure_anchor, coarse_step,
                        exposure_anchor_max_iter, true) &&
              append_coarse_terminal_extension(exposure_anchor)) {
            prepend_start_prefix(exposure_anchor);
            const PathMetrics exposure_metrics =
                evaluatePathMetrics(exposure_anchor);
            if (exposure_metrics.length > max_coarse_length + 1e-9) {
              coarse_audit.exposure_anchor_status = "rejected_detour";
              ++coarse_audit.detour_rejections;
              ROS_INFO("Low-exposure anchor rejected as excessive detour: "
                       "length=%.2fm limit=%.2fm",
                       exposure_metrics.length, max_coarse_length);
            } else if (isDistinctPath(exposure_anchor, coarse_paths)) {
              coarse_paths.push_back(exposure_anchor);
              coarse_weights.push_back(0.0);
              coarse_high_risk_weights.push_back(exposure_weight);
              coarse_roles.push_back("high_risk_exposure_anchor");
              buildCorridorMask(exposure_anchor);
              coarse_corridors.push_back(corridor_mask_);
              coarse_corridor_distances.push_back(
                  corridor_center_distance_field_);
              addDiversityPenalty(exposure_anchor);
              coarse_audit.exposure_anchor_status = "accepted";
              ROS_INFO("Coarse channel %zu: low-exposure anchor, "
                       "length=%.2fm mean_risk=%.3f high_risk=%.1f%% "
                       "weight=%.3f",
                       coarse_paths.size(), exposure_metrics.length,
                       exposure_metrics.mean_risk,
                       100.0 * exposure_metrics.high_risk_fraction,
                       exposure_weight);
            } else {
              coarse_audit.exposure_anchor_status = "duplicate";
              ++coarse_audit.duplicate_rejections;
              ROS_INFO("Low-exposure anchor rejected as duplicate: "
                       "length=%.2fm mean_risk=%.3f high_risk=%.1f%%",
                       exposure_metrics.length, exposure_metrics.mean_risk,
                       100.0 * exposure_metrics.high_risk_fraction);
            }
          } else {
            coarse_audit.exposure_anchor_status = "failed";
            ++coarse_audit.failed_attempts;
            ROS_WARN("Low-exposure coarse anchor failed; balanced search "
                     "will continue with available anchors");
          }
        } else if (exposure_weight <= 0.0) {
          coarse_audit.exposure_anchor_status =
              "skipped_no_high_risk_exposure";
        } else {
          coarse_audit.exposure_anchor_status =
              "skipped_channel_limit";
        }

        // The time budget applies only to additional diversity channels.  It
        // must not prevent construction of the two objective anchors above.
        const ros::WallTime diversity_start = ros::WallTime::now();
        int search_attempt = 0;
        const int max_generation_attempts = std::max(
            coarse_pool_target,
            coarse_pool_target + max_duplicate_attempts_);
        // Moderate normalized objective pairs sample the useful interior of
        // the distance-risk trade-off.  The former fixed 0.5/0.5 search could
        // return only one extreme topology; increasing a cumulative diversity
        // field then pushed the next route around the map boundary.
        const std::array<std::array<double, 2>, 8> tradeoff_schedule = {{
            {{0.25, 0.25}}, {{0.50, 0.25}}, {{0.25, 0.50}},
            {{0.50, 0.50}}, {{0.75, 0.25}}, {{0.25, 0.75}},
            {{0.75, 0.50}}, {{0.50, 0.75}}
        }};
        while (static_cast<int>(coarse_paths.size()) < coarse_pool_target &&
               search_attempt < max_generation_attempts) {
          if (effective_additional_budget > 0.0 &&
              (ros::WallTime::now() - diversity_start).toSec() >=
                  effective_additional_budget) {
            ROS_INFO("Additional coarse-channel generation stopped at "
                     "%.2fs budget",
                     effective_additional_budget);
            break;
          }

          ++search_attempt;
          ++coarse_audit.additional_attempts;
          const auto& objective_pair = tradeoff_schedule[
              static_cast<size_t>(search_attempt - 1) %
              tradeoff_schedule.size()];
          terrain_cost_weight_ = objective_pair[0] * mean_risk_weight;
          active_high_risk_weight_ =
              objective_pair[1] * exposure_weight;

          // Repel from one accepted route at a time, cycling the reference.
          // This opens another topological passage without the old cumulative
          // field that made every interior channel expensive simultaneously.
          clearDiversityPenalty();
          const size_t reference_index =
              static_cast<size_t>(search_attempt - 1) % coarse_paths.size();
          addDiversityPenalty(coarse_paths[reference_index]);
          active_diversity_weight_ = coarse_diversity_weight_;
          std::vector<geometry_msgs::PoseStamped> candidate;
          if (!runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                         candidate, coarse_step, coarse_max_iter, true)) {
            ROS_WARN("Coarse alternative attempt %d failed",
                     search_attempt);
            ++coarse_audit.failed_attempts;
            continue;
          }
          if (!append_coarse_terminal_extension(candidate)) {
            ROS_WARN("Coarse alternative attempt %d rejected: terminal "
                     "corridor extension blocked", search_attempt);
            ++coarse_audit.failed_attempts;
            continue;
          }
          prepend_start_prefix(candidate);

          if (!isDistinctPath(candidate, coarse_paths)) {
            ROS_INFO("Coarse alternative attempt %d rejected as duplicate",
                     search_attempt);
            ++coarse_audit.duplicate_rejections;
            continue;
          }

          const PathMetrics candidate_metrics =
              evaluatePathMetrics(candidate);
          if (candidate_metrics.length > max_coarse_length + 1e-9) {
            ++coarse_audit.detour_rejections;
            ROS_INFO("Coarse alternative attempt %d rejected as excessive "
                     "detour: length=%.2fm limit=%.2fm",
                     search_attempt, candidate_metrics.length,
                     max_coarse_length);
            continue;
          }
          coarse_paths.push_back(candidate);
          coarse_weights.push_back(terrain_cost_weight_);
          coarse_high_risk_weights.push_back(active_high_risk_weight_);
          coarse_roles.push_back("balanced_diverse");
          buildCorridorMask(candidate);
          coarse_corridors.push_back(corridor_mask_);
          coarse_corridor_distances.push_back(
              corridor_center_distance_field_);
          ROS_INFO("Coarse channel %zu accepted: balanced/diverse "
                   "length=%.2fm mean_risk=%.3f peak_risk=%.3f "
                   "high_risk=%.1f%% weights=(%.2f,%.2f) ref=%zu",
                   coarse_paths.size(), candidate_metrics.length,
                   candidate_metrics.mean_risk,
                   candidate_metrics.peak_risk,
                   100.0 * candidate_metrics.high_risk_fraction,
                   objective_pair[0], objective_pair[1],
                   reference_index + 1);
        }
      } else if (channel_limit > 1 && use_multi_heuristic_search_) {
        coarse_audit.risk_anchor_status = "skipped_negligible_risk";
        coarse_audit.exposure_anchor_status =
            "skipped_negligible_risk";
        ROS_INFO("Low-risk anchor skipped: shortest-path mean risk %.6f is "
                 "already negligible", shortest_metrics.mean_risk);
      }
      } else {
        ROS_WARN("Phase 1: shortest coarse anchor failed; "
                 "fine search will use the full map");
      }
    }

    if (coarse_audit.generated_pool_count <= 0) {
      coarse_audit.generated_pool_count =
          static_cast<int>(coarse_paths.size());
    }
    if (static_cast<int>(coarse_paths.size()) > channel_limit) {
      // Select representative channels from the bounded proposal pool instead
      // of accepting the first four searches.  Keep both objective endpoints
      // (shortest and lowest normalized terrain risk), then use farthest-point
      // sampling in equal-weight distance/risk group space augmented by actual
      // geometric novelty.  The small length penalty only breaks equally
      // informative choices and cannot remove a genuinely new topology.
      std::vector<PathMetrics> pool_metrics;
      pool_metrics.reserve(coarse_paths.size());
      for (const auto& path : coarse_paths)
        pool_metrics.push_back(evaluatePathMetrics(path));

      std::array<double, 5> minima;
      std::array<double, 5> maxima;
      minima.fill(std::numeric_limits<double>::infinity());
      maxima.fill(-std::numeric_limits<double>::infinity());
      for (const auto& item : pool_metrics) {
        const std::array<double, 5> values = {{
            item.length, item.mean_risk, item.peak_risk,
            item.tail_risk, item.high_risk_fraction}};
        for (size_t axis = 0; axis < values.size(); ++axis) {
          minima[axis] = std::min(minima[axis], values[axis]);
          maxima[axis] = std::max(maxima[axis], values[axis]);
        }
      }
      auto normalized_metric = [&](size_t index, size_t axis) {
        const PathMetrics& item = pool_metrics[index];
        const std::array<double, 5> values = {{
            item.length, item.mean_risk, item.peak_risk,
            item.tail_risk, item.high_risk_fraction}};
        const double range = maxima[axis] - minima[axis];
        return range > 1e-9
                   ? (values[axis] - minima[axis]) / range
                   : 0.0;
      };
      auto normalized_risk = [&](size_t index) {
        double squared_sum = 0.0;
        for (size_t axis = 1; axis < 5; ++axis) {
          const double value = normalized_metric(index, axis);
          squared_sum += value * value;
        }
        return std::sqrt(0.25 * squared_sum);
      };

      std::vector<size_t> selected_pool_indices;
      std::vector<bool> selected_from_pool(coarse_paths.size(), false);
      selected_pool_indices.push_back(0);  // shortest anchor is invariant
      selected_from_pool[0] = true;

      size_t safest_index = 0;
      double safest_risk = normalized_risk(0);
      for (size_t index = 1; index < coarse_paths.size(); ++index) {
        const double risk = normalized_risk(index);
        if (risk < safest_risk - 1e-9 ||
            (std::fabs(risk - safest_risk) <= 1e-9 &&
             pool_metrics[index].length <
                 pool_metrics[safest_index].length)) {
          safest_index = index;
          safest_risk = risk;
        }
      }
      if (!selected_from_pool[safest_index] && channel_limit > 1) {
        selected_pool_indices.push_back(safest_index);
        selected_from_pool[safest_index] = true;
      }

      // Preserve the route with the smallest observed maximum body roll.
      // Aggregate terrain risk alone can hide this lateral-attitude extreme,
      // and nav_23/nav_24 showed that losing it leaves the final path about
      // 0.5 deg above the dedicated risk-weighted planner.  This consumes no
      // extra search: it only chooses one representative from the pool that
      // has already been generated by the shared coarse search.
      size_t lowest_roll_index = 0;
      for (size_t index = 1; index < coarse_paths.size(); ++index) {
        if (pool_metrics[index].max_abs_roll_deg <
                pool_metrics[lowest_roll_index].max_abs_roll_deg - 1e-9 ||
            (std::fabs(pool_metrics[index].max_abs_roll_deg -
                       pool_metrics[lowest_roll_index].max_abs_roll_deg) <=
                 1e-9 &&
             pool_metrics[index].length <
                 pool_metrics[lowest_roll_index].length)) {
          lowest_roll_index = index;
        }
      }
      if (!selected_from_pool[lowest_roll_index] && channel_limit > 2) {
        selected_pool_indices.push_back(lowest_roll_index);
        selected_from_pool[lowest_roll_index] = true;
      }
      if (coarse_roles[lowest_roll_index] == "pareto_label")
        coarse_roles[lowest_roll_index] = "pareto_low_roll_label";

      // Preserve the shortest route that already satisfies the adaptive
      // terrain-safety gate.  This is the practically important compromise
      // that nav_5/nav_6 could lose when objective-space farthest sampling
      // preferred a more exotic but longer topology.
      size_t shortest_safe_index = coarse_paths.size();
      for (size_t index = 0; index < coarse_paths.size(); ++index) {
        if (!isParetoSafetyFeasible(pool_metrics[index])) continue;
        if (shortest_safe_index == coarse_paths.size() ||
            pool_metrics[index].length <
                pool_metrics[shortest_safe_index].length) {
          shortest_safe_index = index;
        }
      }
      if (shortest_safe_index < coarse_paths.size() &&
          !selected_from_pool[shortest_safe_index] &&
          static_cast<int>(selected_pool_indices.size()) < channel_limit) {
        selected_pool_indices.push_back(shortest_safe_index);
        selected_from_pool[shortest_safe_index] = true;
      }

      while (static_cast<int>(selected_pool_indices.size()) < channel_limit) {
        int best_index = -1;
        double best_coverage =
            -std::numeric_limits<double>::infinity();
        for (size_t candidate_index = 0;
             candidate_index < coarse_paths.size(); ++candidate_index) {
          if (selected_from_pool[candidate_index]) continue;
          double minimum_objective_distance =
              std::numeric_limits<double>::infinity();
          double minimum_topology_novelty =
              std::numeric_limits<double>::infinity();
          for (size_t selected_index : selected_pool_indices) {
            const double length_delta =
                normalized_metric(candidate_index, 0) -
                normalized_metric(selected_index, 0);
            double risk_squared_sum = 0.0;
            for (size_t axis = 1; axis < 5; ++axis) {
              const double delta =
                  normalized_metric(candidate_index, axis) -
                  normalized_metric(selected_index, axis);
              risk_squared_sum += delta * delta;
            }
            const double risk_delta = std::sqrt(0.25 * risk_squared_sum);
            const double objective_distance = std::sqrt(
                0.5 * (length_delta * length_delta +
                       risk_delta * risk_delta));
            minimum_objective_distance = std::min(
                minimum_objective_distance, objective_distance);

            const double hausdorff = pathHausdorffDistance(
                coarse_paths[candidate_index], coarse_paths[selected_index]);
            const double hausdorff_scale = std::max(
                coarse_diversity_radius_, 2.0 * coarse_min_hausdorff_);
            const double hausdorff_novelty =
                std::min(1.0, hausdorff / std::max(1e-3, hausdorff_scale));
            const double separated_length = pathLongestSeparatedLength(
                coarse_paths[candidate_index], coarse_paths[selected_index],
                coarse_min_hausdorff_);
            const double separated_length_novelty = std::min(
                1.0, separated_length /
                         std::max(1e-3,
                                  2.0 * coarse_min_separated_length_));
            minimum_topology_novelty = std::min(
                minimum_topology_novelty,
                0.4 * hausdorff_novelty +
                    0.6 * separated_length_novelty);
          }
          const double coverage =
              0.60 * minimum_objective_distance +
              0.40 * minimum_topology_novelty -
              0.10 * normalized_metric(candidate_index, 0);
          if (coverage > best_coverage + 1e-9 ||
              (std::fabs(coverage - best_coverage) <= 1e-9 &&
               (best_index < 0 || pool_metrics[candidate_index].length <
                                      pool_metrics[best_index].length))) {
            best_coverage = coverage;
            best_index = static_cast<int>(candidate_index);
          }
        }
        if (best_index < 0) break;
        selected_from_pool[best_index] = true;
        selected_pool_indices.push_back(static_cast<size_t>(best_index));
      }

      decltype(coarse_paths) representative_paths;
      decltype(coarse_corridors) representative_corridors;
      decltype(coarse_corridor_distances) representative_distances;
      decltype(coarse_weights) representative_weights;
      decltype(coarse_high_risk_weights) representative_high_risk_weights;
      decltype(coarse_roles) representative_roles;
      for (size_t index : selected_pool_indices) {
        representative_paths.push_back(std::move(coarse_paths[index]));
        representative_corridors.push_back(
            std::move(coarse_corridors[index]));
        representative_distances.push_back(
            std::move(coarse_corridor_distances[index]));
        representative_weights.push_back(coarse_weights[index]);
        representative_high_risk_weights.push_back(
            coarse_high_risk_weights[index]);
        representative_roles.push_back(coarse_roles[index]);
      }
      coarse_audit.pruned_pool_count =
          coarse_audit.generated_pool_count -
          static_cast<int>(representative_paths.size());
      coarse_paths.swap(representative_paths);
      coarse_corridors.swap(representative_corridors);
      coarse_corridor_distances.swap(representative_distances);
      coarse_weights.swap(representative_weights);
      coarse_high_risk_weights.swap(representative_high_risk_weights);
      coarse_roles.swap(representative_roles);
      ROS_INFO("Coarse proposal pool selected %zu/%d representative "
               "channels (shortest + safest + lowest-roll + coverage)",
               coarse_paths.size(), coarse_audit.generated_pool_count);
    }

    // A converged single-topology pool normally needs no extra search.  The
    // exception is a sole route that still crosses a severe terrain/attitude
    // patch: Pareto cannot trade distance against safety when it receives only
    // that route.  Trigger exactly one scalar safety-extreme search in this
    // case.  This is deliberately conditional, so the seven benign
    // single-candidate cases do not all pay for another global search.
    if (pareto_terrain_mode_ && adaptive_safe_candidate_enabled_ &&
        channel_limit > 1 && coarse_paths.size() == 1) {
      const PathMetrics reference = evaluatePathMetrics(coarse_paths.front());
      coarse_audit.adaptive_safe_generation_detour_limit =
          coarse_max_detour_ratio_;
      coarse_audit.adaptive_safe_reference_length_m = reference.length;
      coarse_audit.adaptive_safe_reference_mean_risk = reference.mean_risk;
      coarse_audit.adaptive_safe_reference_combined_risk =
          reference.combined_risk_objective;
      coarse_audit.adaptive_safe_reference_peak_risk = reference.peak_risk;
      coarse_audit.adaptive_safe_reference_pitch_objective =
          reference.pitch_objective;
      coarse_audit.adaptive_safe_reference_roll_objective =
          reference.roll_objective;
      coarse_audit.adaptive_safe_reference_max_pitch_deg =
          reference.max_abs_pitch_deg;
      coarse_audit.adaptive_safe_reference_max_roll_deg =
          reference.max_abs_roll_deg;
      const double effective_peak_trigger = std::min(
          adaptive_safe_peak_risk_trigger_,
          std::min(1.0, pareto_high_risk_threshold_ +
                            adaptive_safe_peak_quantile_margin_));
      coarse_audit.adaptive_safe_effective_peak_trigger =
          effective_peak_trigger;
      const bool peak_trigger =
          reference.peak_risk >= effective_peak_trigger;
      const bool pitch_trigger =
          reference.max_abs_pitch_deg * M_PI / 180.0 >=
          adaptive_safe_pitch_trigger_rad_;
      const bool roll_trigger =
          reference.max_abs_roll_deg * M_PI / 180.0 >=
          adaptive_safe_roll_trigger_rad_;
      if (peak_trigger || pitch_trigger || roll_trigger) {
        std::ostringstream trigger;
        bool has_trigger = false;
        if (peak_trigger) {
          trigger << "peak";
          has_trigger = true;
        }
        if (pitch_trigger) {
          trigger << (has_trigger ? "+pitch" : "pitch");
          has_trigger = true;
        }
        if (roll_trigger)
          trigger << (has_trigger ? "+roll" : "roll");
        coarse_audit.adaptive_safe_trigger = trigger.str();
        coarse_audit.adaptive_safe_status = "searching";
        ++coarse_audit.adaptive_safe_attempts;

        const double saved_search_terrain_weight = terrain_cost_weight_;
        const double saved_search_high_risk_weight =
            active_high_risk_weight_;
        const double saved_search_pointwise_weight =
            active_pointwise_risk_weight_;
        const double saved_search_attitude_scale =
            active_attitude_risk_scale_;
        const double saved_search_diversity_weight =
            active_diversity_weight_;

        // Reuse the map-normalized fine objective, then strengthen the
        // terrain and attitude groups independently.  This avoids fixed raw
        // weights whose meaning changes with each map's risk distribution.
        terrain_cost_weight_ = adaptive_safe_risk_scale_ *
            std::max(fine_mean_risk_weight, 1e-3);
        active_high_risk_weight_ = adaptive_safe_risk_scale_ *
            fine_high_risk_weight;
        active_pointwise_risk_weight_ = adaptive_safe_risk_scale_ *
            fine_pointwise_risk_weight;
        active_attitude_risk_scale_ = adaptive_safe_attitude_scale_;
        active_diversity_weight_ = 0.0;
        clearDiversityPenalty();

        std::vector<geometry_msgs::PoseStamped> safe_candidate;
        const int safe_max_iter = std::max(
            coarse_max_iter, max_iterations_ / 3);
        const double saved_search_length_bound =
            active_search_max_path_length_;
        // Every accepted result receives the same fixed start prefix.  Remove
        // that known length from the 30% coarse-generation envelope; leave
        // terminal-connector length unsubtracted, which makes this bound
        // conservative rather than capable of rejecting an eligible route.
        const double safe_total_length_limit = reference.length *
            (1.0 + coarse_max_detour_ratio_);
        active_search_max_path_length_ = std::max(
            heuristic(search_sx, search_sy, gx, gy),
            safe_total_length_limit - straight_distance);
        coarse_audit.adaptive_safe_coarse_length_bound_m =
            active_search_max_path_length_;
        const std::uint64_t pruned_before_safe_search =
            benchmark_length_bound_pruned_nodes_;
        const bool search_ok = runSearch(
            search_sx, search_sy, st, gx, gy, gt, frame, stamp,
            safe_candidate, coarse_step, safe_max_iter, true) &&
            append_coarse_terminal_extension(safe_candidate);
        coarse_audit.adaptive_safe_coarse_length_pruned =
            benchmark_length_bound_pruned_nodes_ -
            pruned_before_safe_search;
        active_search_max_path_length_ = saved_search_length_bound;
        if (search_ok) prepend_start_prefix(safe_candidate);

        terrain_cost_weight_ = saved_search_terrain_weight;
        active_high_risk_weight_ = saved_search_high_risk_weight;
        active_pointwise_risk_weight_ = saved_search_pointwise_weight;
        active_attitude_risk_scale_ = saved_search_attitude_scale;
        active_diversity_weight_ = saved_search_diversity_weight;

        if (!search_ok) {
          coarse_audit.adaptive_safe_status = "search_failed";
          ++coarse_audit.failed_attempts;
          ROS_WARN("Adaptive safety candidate failed (%s)",
                   coarse_audit.adaptive_safe_trigger.c_str());
        } else {
          const PathMetrics safe_metrics =
              evaluatePathMetrics(safe_candidate);
          coarse_audit.adaptive_safe_candidate_length_m =
              safe_metrics.length;
          coarse_audit.adaptive_safe_candidate_detour_ratio =
              reference.length > 1e-9
                  ? safe_metrics.length / reference.length - 1.0 : 0.0;
          coarse_audit.adaptive_safe_candidate_mean_risk =
              safe_metrics.mean_risk;
          coarse_audit.adaptive_safe_candidate_combined_risk =
              safe_metrics.combined_risk_objective;
          coarse_audit.adaptive_safe_candidate_peak_risk =
              safe_metrics.peak_risk;
          coarse_audit.adaptive_safe_candidate_pitch_objective =
              safe_metrics.pitch_objective;
          coarse_audit.adaptive_safe_candidate_roll_objective =
              safe_metrics.roll_objective;
          coarse_audit.adaptive_safe_candidate_max_pitch_deg =
              safe_metrics.max_abs_pitch_deg;
          coarse_audit.adaptive_safe_candidate_max_roll_deg =
              safe_metrics.max_abs_roll_deg;

          // This is a coarse-route generation gate.  Admit a safety route
          // under the same 30% pool bound as every other coarse candidate,
          // then let its independent fine corridor shorten and refine it.
          // The stricter Pareto decision bound (normally 20%) is still
          // applied later to final fine candidates, so this does not allow
          // an excessively long route to win the final decision.
          const double max_length = reference.length *
              (1.0 + coarse_max_detour_ratio_);
          const bool meaningful_improvement =
              safe_metrics.combined_risk_objective <=
                  0.95 * reference.combined_risk_objective ||
              safe_metrics.peak_risk <= reference.peak_risk - 0.05 ||
              safe_metrics.pitch_objective <=
                  0.90 * reference.pitch_objective ||
              safe_metrics.roll_objective <=
                  0.90 * reference.roll_objective ||
              safe_metrics.max_abs_pitch_deg <=
                  reference.max_abs_pitch_deg - 2.0 ||
              safe_metrics.max_abs_roll_deg <=
                  reference.max_abs_roll_deg - 2.0;
          if (safe_metrics.length > max_length + 1e-9) {
            coarse_audit.adaptive_safe_status = "rejected_detour";
            ++coarse_audit.detour_rejections;
          } else if (!safe_metrics.attitude_feasible) {
            coarse_audit.adaptive_safe_status =
                "rejected_attitude_limit";
          } else if (!meaningful_improvement) {
            coarse_audit.adaptive_safe_status =
                "rejected_no_safety_gain";
          } else if (!isDistinctFinePath(safe_candidate, coarse_paths)) {
            coarse_audit.adaptive_safe_status = "rejected_duplicate";
            ++coarse_audit.duplicate_rejections;
          } else {
            coarse_paths.push_back(std::move(safe_candidate));
            coarse_weights.push_back(0.0);
            coarse_high_risk_weights.push_back(0.0);
            coarse_roles.push_back("adaptive_attitude_safe_anchor");
            buildCorridorMask(coarse_paths.back());
            coarse_corridors.push_back(corridor_mask_);
            coarse_corridor_distances.push_back(
                corridor_center_distance_field_);
            coarse_audit.adaptive_safe_status = "accepted";
            ++coarse_audit.adaptive_safe_candidates;
            ++coarse_audit.generated_pool_count;
            ROS_INFO("Adaptive safety candidate accepted (%s): "
                     "length %.2f->%.2fm peak %.3f->%.3f "
                     "pitch %.1f->%.1fdeg roll %.1f->%.1fdeg",
                     coarse_audit.adaptive_safe_trigger.c_str(),
                     reference.length, safe_metrics.length,
                     reference.peak_risk, safe_metrics.peak_risk,
                     reference.max_abs_pitch_deg,
                     safe_metrics.max_abs_pitch_deg,
                     reference.max_abs_roll_deg,
                     safe_metrics.max_abs_roll_deg);
          }
          if (coarse_audit.adaptive_safe_status != "accepted") {
            ROS_INFO("Adaptive safety candidate %s (%s): "
                     "length %.2f->%.2fm detour=%.1f%% limit=%.1f%% "
                     "combined %.3f->%.3f peak %.3f->%.3f "
                     "pitch %.1f->%.1fdeg roll %.1f->%.1fdeg",
                     coarse_audit.adaptive_safe_status.c_str(),
                     coarse_audit.adaptive_safe_trigger.c_str(),
                     reference.length, safe_metrics.length,
                     100.0 * coarse_audit.adaptive_safe_candidate_detour_ratio,
                     100.0 * coarse_max_detour_ratio_,
                     reference.combined_risk_objective,
                     safe_metrics.combined_risk_objective,
                     reference.peak_risk, safe_metrics.peak_risk,
                     reference.max_abs_pitch_deg,
                     safe_metrics.max_abs_pitch_deg,
                     reference.max_abs_roll_deg,
                     safe_metrics.max_abs_roll_deg);
          }
        }
      } else {
        coarse_audit.adaptive_safe_status = "not_needed";
      }
    } else if (!adaptive_safe_candidate_enabled_) {
      coarse_audit.adaptive_safe_status = "disabled";
    } else if (coarse_paths.size() > 1) {
      coarse_audit.adaptive_safe_status = "not_needed_multi_candidate";
    }
  }

  coarse_audit.accepted_channels =
      static_cast<int>(coarse_paths.size());
  coarse_audit.elapsed_s =
      (ros::WallTime::now() - coarse_phase_start).toSec();
  benchmark_coarse_search_calls_ = benchmark_search_calls_;
  benchmark_coarse_iterations_ = benchmark_iterations_;
  benchmark_coarse_expanded_nodes_ = benchmark_expanded_nodes_;
  benchmark_coarse_generated_nodes_ = benchmark_generated_nodes_;

  publishPathMarkers(coarse_paths_pub_, coarse_paths, "coarse_paths",
                     frame, stamp, 0.18, 1.0);
  // Publish the actual per-cell masks. A fixed-width line would incorrectly
  // show the new obstacle/risk-adaptive asymmetric corridors as symmetric.
  publishCorridorMarkers(coarse_corridors, frame, stamp);

  terrain_cost_weight_ = saved_terrain_weight;
  active_diversity_weight_ = 0.0;
  active_high_risk_weight_ = 0.0;
  active_pointwise_risk_weight_ = 0.0;
  active_attitude_risk_scale_ = 1.0;
  active_curvature_change_weight_ =
      (!basic_mode_ && pareto_terrain_mode_)
          ? fine_curvature_change_weight_ : 0.0;
  risk_heuristic_field_.clear();

  // Phase 2: every coarse path owns one independent corridor. The fine search
  // is deliberately single-queue Hybrid A*: the coarse stage has already
  // identified distinct terrain channels, so repeating MHA* here only adds
  // computation without creating another topology.
  active_map_ = fine_map_.loaded ? &fine_map_ : nullptr;
  std::vector<std::vector<geometry_msgs::PoseStamped>> fine_candidates;
  std::vector<PathMetrics> fine_metrics;
  std::vector<int> fine_source_channels;
  std::vector<std::string> fine_candidate_roles;
  // Geometric similarity alone is not sufficient to reject a fine route.
  // Two paths in the same broad passage may exchange a small length increase
  // for a meaningful peak-risk or attitude reduction. Use the exact four
  // objective groups consumed by the Pareto selector; tolerances below only
  // absorb floating-point copies and do not encode a safety preference.
  const double fine_length_equivalence_epsilon = 1e-4;
  const double fine_objective_equivalence_epsilon = 1e-6;
  auto fine_metrics_equivalent = [&](const PathMetrics& lhs,
                                     const PathMetrics& rhs) {
    return std::fabs(lhs.length - rhs.length) <=
               fine_length_equivalence_epsilon &&
           std::fabs(lhs.combined_risk_objective -
                     rhs.combined_risk_objective) <=
               fine_objective_equivalence_epsilon &&
           std::fabs(lhs.pitch_objective - rhs.pitch_objective) <=
               fine_objective_equivalence_epsilon &&
           std::fabs(lhs.roll_objective - rhs.roll_objective) <=
               fine_objective_equivalence_epsilon;
  };
  auto fine_metrics_dominate = [&](const PathMetrics& lhs,
                                   const PathMetrics& rhs) {
    const bool no_worse =
        lhs.length <= rhs.length + fine_length_equivalence_epsilon &&
        lhs.combined_risk_objective <=
            rhs.combined_risk_objective +
                fine_objective_equivalence_epsilon &&
        lhs.pitch_objective <=
            rhs.pitch_objective + fine_objective_equivalence_epsilon &&
        lhs.roll_objective <=
            rhs.roll_objective + fine_objective_equivalence_epsilon;
    const bool strictly_better =
        lhs.length < rhs.length - fine_length_equivalence_epsilon ||
        lhs.combined_risk_objective <
            rhs.combined_risk_objective -
                fine_objective_equivalence_epsilon ||
        lhs.pitch_objective <
            rhs.pitch_objective - fine_objective_equivalence_epsilon ||
        lhs.roll_objective <
            rhs.roll_objective - fine_objective_equivalence_epsilon;
    return no_worse && strictly_better;
  };
  coarse_audit.fine_channel_roles = coarse_roles;
  coarse_audit.fine_channel_status.assign(
      coarse_corridors.size(), "not_attempted");
  ROS_INFO("Phase 2: independent-corridor fine search "
           "(%.2fm, corridors=%zu)...",
           fine_map_.loaded ? fine_map_.res : 0.0f,
           coarse_corridors.size());

  // Solve the shortest coarse hypothesis first.  Once one executable fine
  // route exists, it is a certified upper bound on the minimum fine-path
  // length.  Every later route that already exceeds the final Pareto detour
  // envelope cannot become decision-eligible, regardless of its risk, and
  // can be pruned before collision-heavy expansion.  This changes only search
  // effort: the same 20% eligibility rule was already applied after all paths
  // were generated by selectParetoCandidate().
  std::vector<PathMetrics> coarse_reference_metrics(coarse_paths.size());
  for (size_t channel = 0; channel < coarse_paths.size(); ++channel) {
    coarse_reference_metrics[channel] =
        evaluatePathMetrics(coarse_paths[channel]);
  }

  // Coarse paths are cheap hypotheses and remain visible for topology audit;
  // fine Hybrid A* is the expensive stage.  Refine only objective-useful
  // hypotheses instead of blindly paying one full lattice search per drawn
  // corridor.  The prefilter uses exactly the four objective groups used by
  // the final Pareto decision, so a skipped coarse path cannot improve every
  // decision dimension over the retained set.
  std::vector<size_t> coarse_front;
  std::vector<bool> coarse_dominated(
      coarse_reference_metrics.size(), false);
  for (size_t candidate = 0;
       candidate < coarse_reference_metrics.size(); ++candidate) {
    bool dominated = false;
    for (size_t other = 0; other < coarse_reference_metrics.size(); ++other) {
      if (candidate == other) continue;
      if (fine_metrics_dominate(coarse_reference_metrics[other],
                                coarse_reference_metrics[candidate])) {
        dominated = true;
        coarse_dominated[candidate] = true;
        break;
      }
    }
    if (!dominated) coarse_front.push_back(candidate);
  }
  if (coarse_front.empty() && !coarse_reference_metrics.empty())
    coarse_front.push_back(0);

  const size_t invalid_channel = std::numeric_limits<size_t>::max();
  size_t shortest_coarse_channel = invalid_channel;
  for (size_t channel : coarse_front) {
    if (shortest_coarse_channel == invalid_channel ||
        coarse_reference_metrics[channel].length <
            coarse_reference_metrics[shortest_coarse_channel].length) {
      shortest_coarse_channel = channel;
    }
  }

  // A coarse path that is dominated in the aggregate four-objective audit
  // can still be the unique minimum-maximum-roll hypothesis, and fine search
  // may improve its other objectives inside the corridor.  Keep this
  // specialist independently of the coarse Pareto-front prefilter.
  size_t lowest_roll_coarse_channel = invalid_channel;
  for (size_t channel = 0;
       channel < coarse_reference_metrics.size(); ++channel) {
    if (lowest_roll_coarse_channel == invalid_channel ||
        coarse_reference_metrics[channel].max_abs_roll_deg <
            coarse_reference_metrics[lowest_roll_coarse_channel]
                    .max_abs_roll_deg -
                1e-9 ||
        (std::fabs(
             coarse_reference_metrics[channel].max_abs_roll_deg -
             coarse_reference_metrics[lowest_roll_coarse_channel]
                 .max_abs_roll_deg) <= 1e-9 &&
         coarse_reference_metrics[channel].length <
             coarse_reference_metrics[lowest_roll_coarse_channel].length)) {
      lowest_roll_coarse_channel = channel;
    }
  }

  std::array<double, 4> objective_min{{
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()}};
  std::array<double, 4> objective_max{{
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()}};
  auto coarse_objectives = [&](size_t channel) {
    const PathMetrics& item = coarse_reference_metrics[channel];
    return std::array<double, 4>{{
        item.length, item.combined_risk_objective,
        item.pitch_objective, item.roll_objective}};
  };
  for (size_t channel : coarse_front) {
    const std::array<double, 4> values = coarse_objectives(channel);
    for (size_t dimension = 0; dimension < values.size(); ++dimension) {
      objective_min[dimension] =
          std::min(objective_min[dimension], values[dimension]);
      objective_max[dimension] =
          std::max(objective_max[dimension], values[dimension]);
    }
  }
  auto normalized_objectives = [&](size_t channel) {
    std::array<double, 4> normalized{{0.0, 0.0, 0.0, 0.0}};
    const std::array<double, 4> values = coarse_objectives(channel);
    for (size_t dimension = 0; dimension < values.size(); ++dimension) {
      const double range = objective_max[dimension] -
                           objective_min[dimension];
      normalized[dimension] = range > 1e-9
          ? (values[dimension] - objective_min[dimension]) / range : 0.0;
    }
    return normalized;
  };

  // The safety representative minimizes the worst normalized regret among
  // combined terrain, pitch and roll groups.  No fixed map-specific weight
  // can hide a poor attitude dimension behind an already-low mean risk.
  size_t safest_coarse_channel = invalid_channel;
  double safest_worst_regret = std::numeric_limits<double>::infinity();
  double safest_total_regret = std::numeric_limits<double>::infinity();
  for (size_t channel : coarse_front) {
    const std::array<double, 4> normalized =
        normalized_objectives(channel);
    const double worst_regret =
        std::max(normalized[1], std::max(normalized[2], normalized[3]));
    const double total_regret = normalized[1] + normalized[2] + normalized[3];
    if (worst_regret < safest_worst_regret - 1e-9 ||
        (std::fabs(worst_regret - safest_worst_regret) <= 1e-9 &&
         total_regret < safest_total_regret - 1e-9)) {
      safest_coarse_channel = channel;
      safest_worst_regret = worst_regret;
      safest_total_regret = total_regret;
    }
  }

  std::vector<size_t> fine_channel_order;
  const size_t fine_limit = std::min(
      coarse_reference_metrics.size(),
      static_cast<size_t>(max_fine_channels_));
  auto add_fine_channel = [&](size_t channel) {
    if (channel == invalid_channel ||
        fine_channel_order.size() >= fine_limit ||
        std::find(fine_channel_order.begin(), fine_channel_order.end(),
                  channel) != fine_channel_order.end()) {
      return;
    }
    fine_channel_order.push_back(channel);
  };
  add_fine_channel(shortest_coarse_channel);
  add_fine_channel(safest_coarse_channel);
  add_fine_channel(lowest_roll_coarse_channel);

  // If two roles refer to the same corridor, fill the remaining slot with the
  // best non-dominated knee hypothesis.
  // Coarse paths are already geometrically independent, so maximizing
  // objective distance here would favour an anomalously long or risky
  // extreme (the nav_32 regression).  Minimax normalized regret retains the
  // most useful length/risk/attitude compromise for the final fine Pareto set.
  while (fine_channel_order.size() < fine_limit) {
    size_t best_channel = invalid_channel;
    double best_worst_regret = std::numeric_limits<double>::infinity();
    double best_total_regret = std::numeric_limits<double>::infinity();
    for (size_t channel : coarse_front) {
      if (std::find(fine_channel_order.begin(), fine_channel_order.end(),
                    channel) != fine_channel_order.end()) {
        continue;
      }
      const std::array<double, 4> normalized =
          normalized_objectives(channel);
      const double worst_regret = *std::max_element(
          normalized.begin(), normalized.end());
      const double total_regret = std::accumulate(
          normalized.begin(), normalized.end(), 0.0);
      if (worst_regret < best_worst_regret - 1e-9 ||
          (std::fabs(worst_regret - best_worst_regret) <= 1e-9 &&
           total_regret < best_total_regret - 1e-9)) {
        best_worst_regret = worst_regret;
        best_total_regret = total_regret;
        best_channel = channel;
      }
    }
    if (best_channel == invalid_channel) break;
    add_fine_channel(best_channel);
  }

  coarse_audit.fine_refinement_limit = max_fine_channels_;
  coarse_audit.fine_refinement_selected =
      static_cast<int>(fine_channel_order.size());
  coarse_audit.fine_budget_pruned = static_cast<int>(
      coarse_corridors.size() - fine_channel_order.size());
  for (size_t channel = 0; channel < coarse_corridors.size(); ++channel) {
    if (std::find(fine_channel_order.begin(), fine_channel_order.end(),
                  channel) == fine_channel_order.end()) {
      coarse_audit.fine_channel_status[channel] =
          coarse_dominated[channel]
              ? "coarse_objective_dominated"
              : "fine_budget_pruned";
    }
  }

  // Search the retained hypotheses from short to long so the first feasible
  // fine path immediately supplies the existing Pareto length upper bound.
  std::stable_sort(
      fine_channel_order.begin(), fine_channel_order.end(),
      [&](size_t lhs, size_t rhs) {
        const double lhs_length = lhs < coarse_reference_metrics.size()
            ? coarse_reference_metrics[lhs].length
            : std::numeric_limits<double>::infinity();
        const double rhs_length = rhs < coarse_reference_metrics.size()
            ? coarse_reference_metrics[rhs].length
            : std::numeric_limits<double>::infinity();
        return lhs_length < rhs_length;
      });

  ROS_INFO("Phase 2 prefilter: coarse=%zu non_dominated=%zu refine=%zu/%d "
           "shortest=%zu safest=%zu",
           coarse_corridors.size(), coarse_front.size(),
           fine_channel_order.size(), max_fine_channels_,
           shortest_coarse_channel == invalid_channel
               ? 0 : shortest_coarse_channel + 1,
           safest_coarse_channel == invalid_channel
               ? 0 : safest_coarse_channel + 1);

  for (size_t order_index = 0;
       order_index < fine_channel_order.size(); ++order_index) {
    const size_t channel = fine_channel_order[order_index];
    corridor_mask_ = coarse_corridors[channel];
    if (channel < coarse_corridor_distances.size()) {
      corridor_center_distance_field_ = coarse_corridor_distances[channel];
    } else {
      // Defensive fallback for any future caller that constructs only a mask.
      // Disabling the soft centre constraint is safer than applying a field
      // belonging to another terrain channel.
      corridor_center_distance_field_.clear();
    }
    use_corridor_ = true;
    // A coarse route defines only a topological channel. Every fine search
    // must solve the same robust objective; otherwise the shortest corridor
    // ignores terrain while the risk corridor ignores length, and the final
    // Pareto comparison mixes paths that were optimized for different tasks.
    if (pareto_terrain_mode_) {
      terrain_cost_weight_ = fine_mean_risk_weight;
      active_high_risk_weight_ = fine_high_risk_weight;
      active_pointwise_risk_weight_ = fine_pointwise_risk_weight;
    } else {
      terrain_cost_weight_ =
          channel < coarse_weights.size()
              ? coarse_weights[channel] : saved_terrain_weight;
      active_high_risk_weight_ =
          channel < coarse_high_risk_weights.size()
              ? coarse_high_risk_weights[channel] : 0.0;
      active_pointwise_risk_weight_ = 0.0;
    }
    const bool adaptive_safe_fine_channel =
        channel < coarse_roles.size() &&
        coarse_roles[channel] == "adaptive_attitude_safe_anchor";
    const bool safety_focused_fine_channel =
        adaptive_safe_fine_channel || channel == safest_coarse_channel ||
        (channel < coarse_roles.size() &&
         coarse_roles[channel] == "pareto_safest_label");
    if (pareto_terrain_mode_ && safety_focused_fine_channel) {
      terrain_cost_weight_ *= adaptive_safe_risk_scale_;
      active_high_risk_weight_ *= adaptive_safe_risk_scale_;
      active_pointwise_risk_weight_ *= adaptive_safe_risk_scale_;
      active_attitude_risk_scale_ = adaptive_safe_attitude_scale_;
    } else {
      active_attitude_risk_scale_ = 1.0;
    }
    // In Pareto mode the coarse line identifies a connected passage rather
    // than a reference trajectory. A weak normalized-distance term only
    // breaks ties where ordinary corridor masks overlap. The adaptive safety
    // channel (including the safest label selected from the shared search) is
    // different: retain that safety hypothesis with a stronger *soft*
    // topology term and risk response until Pareto comparison. Obstacles and
    // the decision detour remain the only hard constraints.
    active_corridor_center_weight_ = pareto_terrain_mode_
        ? (coarse_corridors.size() > 1
               ? (safety_focused_fine_channel
                      ? adaptive_safe_topology_preservation_weight_
                      : fine_topology_preservation_weight_)
               : 0.0)
        : (terrain_cost_weight_ > 1e-9 ? corridor_center_weight_ : 0.0);

    const PathMetrics coarse_reference =
        channel < coarse_reference_metrics.size()
            ? coarse_reference_metrics[channel]
            : PathMetrics{};

    // Reverse 2-D distance inside this exact corridor is an admissible lower
    // bound for the non-holonomic fine search. It guides nodes around blocked
    // cells without adding terrain preferences or changing the Pareto
    // objective, so fewer states are expanded for the same optimality target.
    buildHolonomicHeuristicField(gx, gy);
    if (!holonomicFieldReaches(search_sx, search_sy)) {
      coarse_audit.fine_channel_status[channel] =
          "corridor_disconnected_2d_precheck";
      ROS_WARN("Fine search in coarse channel %zu skipped: corridor has no "
               "clearance-connected 2-D path from start to terminal target",
               channel + 1);
      continue;
    }
    std::vector<geometry_msgs::PoseStamped> candidate;
    const double saved_search_length_bound =
        active_search_max_path_length_;
    if (pareto_terrain_mode_ && !fine_metrics.empty()) {
      const auto shortest_fine = std::min_element(
          fine_metrics.begin(), fine_metrics.end(),
          [](const PathMetrics& lhs, const PathMetrics& rhs) {
            return lhs.length < rhs.length;
          });
      const double decision_total_length_limit = shortest_fine->length *
          (1.0 + pareto_decision_detour_ratio_);
      // As in coarse search, subtract only the guaranteed common start prefix.
      // The unaccounted terminal transition leaves slack, so every route that
      // could pass the final 20% Pareto decision bound remains reachable.
      active_search_max_path_length_ = std::max(
          heuristic(search_sx, search_sy, gx, gy),
          decision_total_length_limit - straight_distance);
      if (adaptive_safe_fine_channel) {
        coarse_audit.adaptive_safe_fine_length_bound_m =
            active_search_max_path_length_;
      }
      ROS_DEBUG("Fine channel %zu decision-length bound: %.2fm "
                "(best executable %.2fm, detour %.0f%%)",
                channel + 1, active_search_max_path_length_,
                shortest_fine->length,
                100.0 * pareto_decision_detour_ratio_);
    }
    const std::uint64_t pruned_before_fine_search =
        benchmark_length_bound_pruned_nodes_;
    bool curvature_regularizer_fallback = false;
    bool fine_search_ok = runSearch(
        search_sx, search_sy, st, gx, gy, gt, frame, stamp,
        candidate, -1, -1, false);
    const bool essential_fine_channel =
        channel == shortest_coarse_channel || safety_focused_fine_channel;
    // Removing a curvature-quality term can help only when the search already
    // reached the terminal neighbourhood but failed to settle on an acceptable
    // terminal state.  If it never came close, the limiting factor is corridor
    // geometry/clearance or reachability; rerunning the identical 300k-state
    // search without curvature cost merely doubles runtime (nav_34: 2.24 m and
    // 2.34 m closest distances, zero terminal visits in both attempts).
    const double curvature_retry_radius =
        std::max(goal_xy_tol_, 1.5 * step_size_);
    const bool curvature_retry_has_evidence =
        last_search_terminal_position_visits_ > 0 ||
        last_search_closest_goal_distance_ <= curvature_retry_radius;
    bool curvature_retry_skipped_unreachable = false;
    if (!fine_search_ok && active_curvature_change_weight_ > 0.0 &&
        essential_fine_channel && curvature_retry_has_evidence) {
      const double saved_curvature_change_weight =
          active_curvature_change_weight_;
      active_curvature_change_weight_ = 0.0;
      candidate.clear();
      ROS_WARN("Fine search in coarse channel %zu exhausted with curvature "
               "regularization; retrying the same corridor without that "
               "quality term", channel + 1);
      fine_search_ok = runSearch(
          search_sx, search_sy, st, gx, gy, gt, frame, stamp,
          candidate, -1, -1, false);
      active_curvature_change_weight_ = saved_curvature_change_weight;
      curvature_regularizer_fallback = fine_search_ok;
    } else if (!fine_search_ok && active_curvature_change_weight_ > 0.0 &&
               essential_fine_channel) {
      curvature_retry_skipped_unreachable = true;
      ROS_WARN("Fine search in coarse channel %zu did not reach the terminal "
               "neighbourhood (closest=%.2fm, visits=%d, retry_radius=%.2fm); "
               "skipping curvature-disabled retry",
               channel + 1, last_search_closest_goal_distance_,
               last_search_terminal_position_visits_,
               curvature_retry_radius);
    }
    if (adaptive_safe_fine_channel) {
      coarse_audit.adaptive_safe_fine_length_pruned =
          benchmark_length_bound_pruned_nodes_ -
          pruned_before_fine_search;
    }
    active_search_max_path_length_ = saved_search_length_bound;
    if (!fine_search_ok) {
      coarse_audit.fine_channel_status[channel] =
          curvature_retry_skipped_unreachable
              ? "search_failed_terminal_unreachable_no_retry"
              : "search_failed";
      ROS_WARN("Fine search in coarse channel %zu failed; "
               "channel discarded", channel + 1);
      continue;
    }
    if (!append_terminal_approach(candidate)) {
      coarse_audit.fine_channel_status[channel] =
          terminal_transition_failure_status;
      ROS_WARN("Fine search in coarse channel %zu rejected: terminal "
               "transition infeasible", channel + 1);
      continue;
    }
    prepend_start_prefix(candidate);
    // Preserve the collision-, curvature- and corridor-checked raw path.
    // Spline smoothing is allowed to improve geometry, but must never turn a
    // feasible Hybrid-A* result into a navigation failure by crossing a local
    // pitch/roll hard-limit cell.
    std::vector<geometry_msgs::PoseStamped> raw_candidate = candidate;
    const PathMetrics raw_metrics = evaluatePathMetrics(raw_candidate);
    // Smooth the complete geometry, including the reserved terminal segment.
    // Smoothing only inside runSearch cannot remove the entry corner because
    // that segment has not been appended yet.
    bool smoothing_applied = smoothPath(candidate, frame, stamp);
    PathMetrics metrics = smoothing_applied
                              ? evaluatePathMetrics(candidate) : raw_metrics;
    bool attitude_fallback_applied = false;
    if (!metrics.attitude_feasible && smoothing_applied &&
        raw_metrics.attitude_feasible) {
      candidate.swap(raw_candidate);
      metrics = raw_metrics;
      smoothing_applied = false;
      attitude_fallback_applied = true;
      ROS_WARN("Fine search in coarse channel %zu: smoothing crossed an "
               "attitude hard limit; using feasible raw path "
               "(pitch=%.1fdeg roll=%.1fdeg)",
               channel + 1, metrics.max_abs_pitch_deg,
               metrics.max_abs_roll_deg);
    }
    if (!metrics.attitude_feasible) {
      coarse_audit.fine_channel_status[channel] =
          smoothing_applied ? "attitude_limit_after_smoothing"
                            : "attitude_limit_raw_path";
      ROS_WARN("Fine search in coarse channel %zu rejected: %s path "
               "exceeds pitch/roll physical limit "
               "(pitch=%.1fdeg roll=%.1fdeg)",
               channel + 1, smoothing_applied ? "smoothed" : "raw",
               metrics.max_abs_pitch_deg, metrics.max_abs_roll_deg);
      continue;
    }
    std::vector<size_t> similar_indices;
    for (size_t accepted_index = 0;
         accepted_index < fine_candidates.size(); ++accepted_index) {
      if (areFinePathsGeometricallySimilar(
              candidate, fine_candidates[accepted_index])) {
        similar_indices.push_back(accepted_index);
        ++coarse_audit.fine_geometric_similarity_matches;
      }
    }

    bool discard_similar_candidate = false;
    bool equivalent_to_existing = false;
    size_t rejecting_index = 0;
    for (size_t accepted_index : similar_indices) {
      if (fine_metrics_equivalent(metrics, fine_metrics[accepted_index])) {
        discard_similar_candidate = true;
        equivalent_to_existing = true;
        rejecting_index = accepted_index;
        break;
      }
      if (fine_metrics_dominate(fine_metrics[accepted_index], metrics)) {
        discard_similar_candidate = true;
        rejecting_index = accepted_index;
        break;
      }
    }
    if (discard_similar_candidate) {
      coarse_audit.fine_channel_status[channel] =
          equivalent_to_existing
              ? "duplicate_objective_equivalent"
              : "duplicate_dominated_same_topology";
      if (equivalent_to_existing) {
        ++coarse_audit.fine_objective_equivalent_rejections;
      } else {
        ++coarse_audit.fine_same_topology_dominated_rejections;
      }
      ROS_INFO("Fine search in coarse channel %zu geometrically matches "
               "candidate %zu and is %s; discarded "
               "(L=%.2fm general=%.3f pitch=%.3f roll=%.3f)",
               channel + 1, rejecting_index + 1,
               equivalent_to_existing ? "objective-equivalent" : "dominated",
               metrics.length, metrics.combined_risk_objective,
               metrics.pitch_objective, metrics.roll_objective);
      continue;
    }

    std::vector<size_t> replace_indices;
    bool retains_same_topology_tradeoff = false;
    for (size_t accepted_index : similar_indices) {
      if (fine_metrics_dominate(metrics, fine_metrics[accepted_index])) {
        replace_indices.push_back(accepted_index);
      } else {
        // Equivalence and accepted-dominates cases returned above. Therefore
        // neither route dominates the other: this is a real same-passage
        // length-risk/attitude tradeoff and must reach Pareto comparison.
        retains_same_topology_tradeoff = true;
      }
    }
    for (auto index_it = replace_indices.rbegin();
         index_it != replace_indices.rend(); ++index_it) {
      const size_t accepted_index = *index_it;
      const int replaced_channel = fine_source_channels[accepted_index];
      if (replaced_channel > 0 &&
          static_cast<size_t>(replaced_channel) <=
              coarse_audit.fine_channel_status.size()) {
        coarse_audit.fine_channel_status[replaced_channel - 1] =
            "replaced_by_better_same_topology_candidate";
      }
      fine_candidates.erase(fine_candidates.begin() + accepted_index);
      fine_metrics.erase(fine_metrics.begin() + accepted_index);
      fine_source_channels.erase(
          fine_source_channels.begin() + accepted_index);
      fine_candidate_roles.erase(
          fine_candidate_roles.begin() + accepted_index);
      ++coarse_audit.fine_same_topology_replacements;
    }
    if (retains_same_topology_tradeoff) {
      ++coarse_audit.fine_same_topology_tradeoffs;
    }

    if (smoothing_applied) {
      ++coarse_audit.smoothed_fine_candidates;
    } else {
      // Includes geometric smoothing rejection and the attitude-safe fallback.
      ++coarse_audit.raw_fine_fallbacks;
      if (attitude_fallback_applied)
        ++coarse_audit.attitude_fine_fallbacks;
    }
    fine_candidates.push_back(std::move(candidate));
    fine_metrics.push_back(metrics);
    fine_source_channels.push_back(static_cast<int>(channel + 1));
    fine_candidate_roles.push_back(
        channel < coarse_roles.size() ? coarse_roles[channel] : "unknown");
    const std::string acceptance_suffix =
        retains_same_topology_tradeoff
            ? "_same_topology_tradeoff"
            : (!replace_indices.empty() ? "_replaced_same_topology" : "");
    std::string fine_acceptance_status =
        smoothing_applied
            ? "accepted_smoothed"
            : (attitude_fallback_applied
                   ? "accepted_raw_attitude_fallback"
                   : "accepted_raw_fallback");
    if (curvature_regularizer_fallback)
      fine_acceptance_status += "_curvature_regularizer_fallback";
    fine_acceptance_status += acceptance_suffix;
    coarse_audit.fine_channel_status[channel] = fine_acceptance_status;
    ROS_INFO("Fine candidate %zu (channel %zu, role=%s): length=%.2fm "
             "mean_risk=%.3f peak_risk=%.3f tail_risk=%.3f "
             "high_risk=%.1f%% pitch_p95=%.1fdeg roll_p95=%.1fdeg "
             "| coarse->fine delta: mean=%+.3f "
             "tail=%+.3f exposure=%+.1fpp",
             fine_candidates.size(), channel + 1,
             fine_candidate_roles.back().c_str(), metrics.length,
             metrics.mean_risk, metrics.peak_risk, metrics.tail_risk,
             100.0 * metrics.high_risk_fraction,
             metrics.p95_abs_pitch_deg, metrics.p95_abs_roll_deg,
             metrics.mean_risk - coarse_reference.mean_risk,
             metrics.tail_risk - coarse_reference.tail_risk,
             100.0 * (metrics.high_risk_fraction -
                      coarse_reference.high_risk_fraction));
  }

  // Preserve every valid realised fine route for reproducible auditing.  The
  // Pareto selector below already excludes dominated candidates from its
  // front, so deleting them here cannot change the chosen path; it only hides
  // why a corridor lost and made many benchmark runs appear to have a single
  // candidate.  Invalid/colliding routes are still rejected before this point.

  publishPathMarkers(fine_candidates_pub_, fine_candidates,
                     "fine_candidates", frame, stamp, 0.14, 1.0);

  bool ok = false;
  int selected = -1;
  std::vector<bool> on_pareto_front(fine_metrics.size(), false);
  std::vector<double> ideal_distances(
      fine_metrics.size(), std::numeric_limits<double>::infinity());
  std::string decision_mode =
      pareto_terrain_mode_ ? "pareto" : "weighted";
  std::string decision_reason = "no_fine_candidate";
  if (!fine_candidates.empty()) {
    if (pareto_terrain_mode_) {
      selected = selectParetoCandidate(
          fine_metrics, &on_pareto_front, &ideal_distances);
      const size_t front_size = static_cast<size_t>(std::count(
          on_pareto_front.begin(), on_pareto_front.end(), true));
      coarse_audit.dominated_fine_rejections = static_cast<int>(
          fine_metrics.size() - front_size);
      for (size_t index = 0; index < on_pareto_front.size(); ++index) {
        if (on_pareto_front[index] ||
            index >= fine_source_channels.size()) {
          continue;
        }
        const int source_channel = fine_source_channels[index];
        if (source_channel <= 0 ||
            static_cast<size_t>(source_channel) >
                coarse_audit.fine_channel_status.size()) {
          continue;
        }
        std::string& status =
            coarse_audit.fine_channel_status[source_channel - 1];
        if (status.find("_dominated") == std::string::npos)
          status += "_dominated";
      }
      decision_reason =
          front_size == 1
              ? "only_non_dominated_candidate"
              : "pareto_bounded_minimax_near_tie_risk_priority";
      ROS_INFO("Pareto decision compared all %zu candidates: front=%zu, "
               "terrain risk is a soft objective, generation detour=%.1f%% "
               "decision bound=%.1f%% and near-tie margin=%.3f",
               fine_metrics.size(), front_size,
               100.0 * coarse_max_detour_ratio_,
               100.0 * pareto_decision_detour_ratio_,
               pareto_score_near_tie_margin_);
    } else {
      selected = 0;
      decision_reason = "first_candidate_in_weighted_mode";
    }
    if (selected >= 0 &&
        selected < static_cast<int>(fine_candidates.size())) {
      // Keep the diagnostic candidate intact. The evaluator compares this
      // exact candidate with /hybrid_astar/plan using the shared timestamp.
      plan = fine_candidates[selected];
      ok = true;
      ROS_INFO("Selected fine candidate %d/%zu: length=%.2fm "
               "mean_risk=%.3f peak_risk=%.3f tail_risk=%.3f "
               "high_risk=%.1f%%",
               selected + 1, fine_metrics.size(),
               fine_metrics[selected].length,
               fine_metrics[selected].mean_risk,
               fine_metrics[selected].peak_risk,
               fine_metrics[selected].tail_risk,
               100.0 * fine_metrics[selected].high_risk_fraction);
    }
  }

  // One full-map fallback preserves reachability without collapsing every
  // failed corridor into another copy of the same global search.
  if (!ok) {
    use_corridor_ = false;
    terrain_cost_weight_ = saved_terrain_weight;
    active_high_risk_weight_ = 0.0;
    active_pointwise_risk_weight_ = 0.0;
    buildHolonomicHeuristicField(gx, gy);
    ROS_WARN("No independent corridor produced a fine path; "
             "retrying once on the full fine map");
    ok = runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                   plan, -1, -1, false);
    if (!ok && active_curvature_change_weight_ > 0.0) {
      const double saved_curvature_change_weight =
          active_curvature_change_weight_;
      active_curvature_change_weight_ = 0.0;
      plan.clear();
      ROS_WARN("Full-map fine search exhausted with curvature "
               "regularization; retrying without that quality term");
      ok = runSearch(search_sx, search_sy, st, gx, gy, gt, frame, stamp,
                     plan, -1, -1, false);
      active_curvature_change_weight_ = saved_curvature_change_weight;
      if (ok) decision_reason = "full_map_curvature_regularizer_fallback";
    }
    if (ok && append_terminal_approach(plan)) {
      prepend_start_prefix(plan);
      smoothPath(plan, frame, stamp);
      if (!evaluatePathMetrics(plan).attitude_feasible) {
        ok = false;
        ROS_WARN("Full-map path rejected: smoothed path exceeds pitch/roll "
                 "physical limit");
      }
    } else if (ok) {
      ok = false;
      ROS_WARN("Full-map path rejected: terminal transition infeasible");
    }
    decision_mode = "full_map_fallback";
    if (!ok) {
      decision_reason = "full_map_search_failed";
    } else if (decision_reason !=
               "full_map_curvature_regularizer_fallback") {
      decision_reason = "no_corridor_candidate_reached_goal";
    }
    selected = -1;
  }
  terrain_cost_weight_ = saved_terrain_weight;
  active_high_risk_weight_ = 0.0;
  active_pointwise_risk_weight_ = 0.0;
  active_attitude_risk_scale_ = 1.0;
  active_curvature_change_weight_ = 0.0;
  active_corridor_center_weight_ = 0.0;
  active_search_max_path_length_ =
      std::numeric_limits<double>::infinity();
  use_corridor_ = false;
  holonomic_heuristic_field_.clear();

  for (size_t index = 0; index < fine_metrics.size(); ++index) {
    const bool on_front =
        index < on_pareto_front.size() && on_pareto_front[index];
    const double score =
        index < ideal_distances.size() &&
                std::isfinite(ideal_distances[index])
            ? ideal_distances[index] : -1.0;
    const bool zero_high_risk_exposure =
        fine_metrics[index].high_risk_fraction <= 1e-9;
    const bool safety_feasible = isParetoSafetyFeasible(fine_metrics[index]);
    const double safety_violation =
        computeParetoSafetyViolation(fine_metrics[index]);
    ROS_INFO("Pareto audit candidate %zu: channel=%d role=%s length=%.2fm "
             "mean_risk=%.3f peak_risk=%.3f tail_risk=%.3f "
             "high_risk=%.1f%% pitch_p95=%.1fdeg roll_p95=%.1fdeg "
             "zero_exposure=%s safety_feasible=%s "
             "safety_violation=%.3f front=%s compromise_score=%.3f "
             "selected=%s",
             index + 1,
             index < fine_source_channels.size()
                 ? fine_source_channels[index] : -1,
             index < fine_candidate_roles.size()
                 ? fine_candidate_roles[index].c_str() : "unknown",
             fine_metrics[index].length, fine_metrics[index].mean_risk,
             fine_metrics[index].peak_risk,
             fine_metrics[index].tail_risk,
             100.0 * fine_metrics[index].high_risk_fraction,
             fine_metrics[index].p95_abs_pitch_deg,
             fine_metrics[index].p95_abs_roll_deg,
             zero_high_risk_exposure ? "yes" : "no",
             safety_feasible ? "yes" : "no",
             safety_violation, on_front ? "yes" : "no", score,
             static_cast<int>(index) == selected ? "yes" : "no");
  }
  ROS_INFO("Pareto decision: mode=%s candidates=%zu selected=%d reason=%s",
           decision_mode.c_str(), fine_metrics.size(), selected + 1,
           decision_reason.c_str());
  publishParetoDecision(
      fine_candidates, fine_metrics, fine_source_channels,
      fine_candidate_roles,
      on_pareto_front, ideal_distances, coarse_audit, selected,
      decision_mode, decision_reason, frame, stamp);

  if (ok) {
    nav_msgs::Path msg;
    msg.header.frame_id = frame;
    msg.header.stamp = stamp;
    msg.poses = plan;
    plan_pub_.publish(msg);
  } else {
    ROS_WARN("Hybrid A*: no path found");
  }
  publishPlannerStatistics(
      start, goal, plan, ok,
      ok ? "success" : decision_reason,
      coarse_audit.accepted_channels,
      static_cast<int>(fine_candidates.size()),
      selected >= 0 ? selected + 1 : 0,
      coarse_audit.elapsed_s);
  return ok;
}

}  // namespace my_global_planner
