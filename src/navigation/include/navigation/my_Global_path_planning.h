#ifndef MY_GLOBAL_PATH_PLANNING_H
#define MY_GLOBAL_PATH_PLANNING_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <cstdint>
#include <limits>
#include <vector>
#include <queue>
#include <unordered_map>
#include <string>

namespace my_global_planner {

struct TerrainMap {
  std::vector<float> dem, grad_x, grad_y, cost_total;
  int rows, cols;
  float res, origin_x, origin_y;
  float friction_coeff, track_width, cg_height;
  bool loaded;
  TerrainMap() : rows(0), cols(0), res(0), origin_x(0), origin_y(0),
                 friction_coeff(0.4f), track_width(1.000f), cg_height(0.5f),
                 loaded(false) {}
};

class HybridAStarPlanner : public nav_core::BaseGlobalPlanner {
public:
  HybridAStarPlanner();
  HybridAStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  ~HybridAStarPlanner() override = default;

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

protected:
  // Baseline mode preserves the same search implementation while disabling
  // only terrain-risk scoring and Pareto candidate selection.
  explicit HybridAStarPlanner(bool basic_mode);

private:
  struct Node3D {
    double x, y, theta;
    int ix, iy, itheta;
    double g, h;
    // Monotone path-label objectives.  `g` remains the scalar motion-quality
    // cost used by ordinary fine search; the coarse Pareto search retains
    // several non-dominated labels at the same (x,y,theta) state using these
    // separate accumulators instead of collapsing them into one weight.
    double path_length = 0.0;
    double risk_integral = 0.0;
    double pitch_risk_integral = 0.0;
    double roll_risk_integral = 0.0;
    double high_risk_length = 0.0;
    double pointwise_risk_integral = 0.0;
    double peak_risk = 0.0;
    // Bounded spatial-channel signature used only by the shared coarse
    // multi-label search. Two progress gates record the quantized lateral
    // side from which the label crossed the start-goal corridor. Labels with
    // different signatures cannot dominate one another before reaching the
    // goal, preserving useful topologies without adding a new lattice axis.
    std::uint16_t topology_signature = 0;
    std::uint8_t topology_gate_mask = 0;
    // Signed primitive used to reach this node.  Keeping the actual motion
    // primitive lets reconstruction sample the same bounded-curvature arc
    // used during collision checking instead of joining sparse endpoints by
    // straight chords (which creates artificial curvature impulses).
    double motion_curvature = 0.0;
    // Index of the steering primitive used to enter this node.  It is retained
    // for the local curvature-change edge cost, but deliberately not added to
    // the global lattice key: doing so multiplies every (x,y,theta) state by
    // seven and made long-distance fine search exhaust 300k iterations.
    int steering_index = -1;
    double motion_length = 0.0;
    int parent;
    bool reverse;
    double f() const { return g + h; }
  };

  struct CompareNode {
    bool operator()(const std::pair<double, int>& a,
                    const std::pair<double, int>& b) const {
      return a.first > b.first;
    }
  };

  double min_turning_radius_;
  double step_size_;
  int heading_bins_;
  int max_iterations_;
  double goal_xy_tol_;
  double goal_yaw_tol_;
  bool enable_dubins_shot_;
  double dubins_shot_max_distance_;
  bool allow_goal_yaw_relaxation_;
  double goal_yaw_relaxation_penalty_;
  bool enforce_terminal_goal_yaw_;
  double terminal_approach_distance_;
  double terminal_heading_blend_distance_;
  double terminal_heading_soft_weight_;
  int terminal_candidate_settle_iterations_;
  double terminal_transition_tangent_scale_;
  double terminal_transition_max_backtrack_;
  int analytic_expansion_skip_;
  double analytic_min_prefix_distance_;
  double analytic_max_mean_terrain_cost_;
  double analytic_max_peak_terrain_cost_;
  bool allow_reverse_;
  bool allow_start_in_place_rotation_;
  double start_straight_distance_;
  double reverse_penalty_;
  double direction_change_penalty_;
  int num_steering_angles_;
  double obstacle_penalty_;
  double curvature_penalty_;
  double fine_curvature_change_weight_;
  double min_clearance_;

  std::string terrain_cost_file_;
  std::string terrain_cost_coarse_file_;
  double terrain_cost_weight_;
  bool pareto_terrain_mode_;
  double pareto_high_risk_quantile_;
  double pareto_high_risk_threshold_;
  double pareto_anchor_strength_;
  double fine_risk_strength_;
  double fine_heuristic_weight_;
  int fine_stagnation_min_iterations_;
  int fine_stagnation_window_iterations_;
  double fine_stagnation_progress_m_;
  double primitive_collision_check_fraction_;
  double pointwise_risk_exponent_;
  double pareto_tail_fraction_;
  bool attitude_risk_enabled_;
  double pitch_soft_start_rad_;
  double roll_soft_start_rad_;
  double pitch_hard_limit_rad_;
  double roll_hard_limit_rad_;
  double pitch_risk_weight_;
  double roll_risk_weight_;
  double attitude_penalty_exponent_;
  bool adaptive_safe_candidate_enabled_;
  double adaptive_safe_peak_risk_trigger_;
  double adaptive_safe_peak_quantile_margin_;
  double adaptive_safe_pitch_trigger_rad_;
  double adaptive_safe_roll_trigger_rad_;
  double adaptive_safe_risk_scale_;
  double adaptive_safe_attitude_scale_;
  int pareto_candidate_count_;
  bool pareto_safety_filter_enabled_;
  double pareto_max_high_risk_fraction_;
  double pareto_peak_risk_margin_;
  double pareto_tail_risk_margin_;
  double corridor_width_;
  double corridor_min_width_;
  double corridor_risk_margin_;
  double corridor_center_weight_;
  double fine_topology_preservation_weight_;
  double adaptive_safe_topology_preservation_weight_;
  double fine_min_hausdorff_;
  double fine_max_overlap_;
  bool use_multi_heuristic_search_;
  double mha_anchor_weight_;
  double mha_risk_weight_;
  double mha_heading_weight_;
  double mha_heading_gain_;
  double mha_queue_bound_;
  int max_coarse_channels_;
  int max_fine_channels_;
  int coarse_pool_extra_candidates_;
  int max_duplicate_attempts_;
  double coarse_search_time_budget_;
  double coarse_search_time_per_meter_;
  double coarse_search_time_max_;
  double coarse_max_detour_ratio_;
  double pareto_decision_detour_ratio_;
  double pareto_score_near_tie_margin_;
  double coarse_diversity_radius_;
  double coarse_diversity_weight_;
  double coarse_min_hausdorff_;
  double coarse_min_separated_length_;
  bool coarse_multi_label_enabled_;
  int coarse_labels_per_state_;
  double coarse_label_epsilon_length_;
  double coarse_label_epsilon_risk_;
  bool coarse_topology_signature_enabled_;
  double coarse_topology_gate_first_ratio_;
  double coarse_topology_gate_second_ratio_;
  double coarse_topology_lateral_bin_width_;
  int coarse_goal_label_limit_;
  int coarse_goal_settle_iterations_;
  int coarse_min_stable_channels_;
  int coarse_min_goal_labels_for_settle_;
  int coarse_single_topology_settle_iterations_;
  double coarse_single_topology_anchor_slack_;
  double coarse_multi_label_time_limit_;

  TerrainMap fine_map_, coarse_map_;
  const TerrainMap* active_map_;

  std::vector<bool> corridor_mask_;
  std::vector<float> corridor_center_distance_field_;
  bool use_corridor_;
  double active_corridor_center_weight_;
  std::vector<float> diversity_penalty_field_;
  double active_diversity_weight_;
  double active_high_risk_weight_;
  double active_pointwise_risk_weight_;
  double active_attitude_risk_scale_;
  // Zero for coarse/topology search; enabled only for fine trajectory search.
  // It is a bounded local regularizer rather than another lattice dimension.
  double active_curvature_change_weight_;
  // Optional branch-and-bound limit for one search invocation.  It constrains
  // only geometric path length; the scalar terrain objective is unchanged.
  double active_search_max_path_length_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  bool initialized_;
  bool basic_mode_;
  double theta_res_;

  ros::Publisher plan_pub_;
  ros::Publisher coarse_paths_pub_;
  ros::Publisher corridor_paths_pub_;
  ros::Publisher fine_candidates_pub_;
  ros::Publisher pareto_decision_pub_;
  ros::Publisher planner_statistics_pub_;

  // Aggregate counters cover every coarse/fine/fallback search executed for
  // one goal. They make the complete proposed method directly comparable to
  // the two independent single-search baselines.
  std::uint64_t benchmark_search_calls_ = 0;
  std::uint64_t benchmark_iterations_ = 0;
  std::uint64_t benchmark_expanded_nodes_ = 0;
  std::uint64_t benchmark_generated_nodes_ = 0;
  std::uint64_t benchmark_max_stored_nodes_ = 0;
  std::uint64_t benchmark_coarse_search_calls_ = 0;
  std::uint64_t benchmark_coarse_iterations_ = 0;
  std::uint64_t benchmark_coarse_expanded_nodes_ = 0;
  std::uint64_t benchmark_coarse_generated_nodes_ = 0;
  mutable std::uint64_t benchmark_primitive_attempts_ = 0;
  mutable std::uint64_t benchmark_collision_checks_ = 0;
  mutable std::uint64_t benchmark_collision_rejections_ = 0;
  std::uint64_t benchmark_length_bound_pruned_nodes_ = 0;
  ros::WallTime benchmark_plan_start_;

  // Diagnostics from the most recent lattice search.  These are intentionally
  // separate from the accumulated benchmark counters: fine-channel fallback
  // policy needs to know whether *that search* actually reached the terminal
  // neighbourhood before deciding that curvature regularisation may be the
  // reason for failure.
  double last_search_closest_goal_distance_ =
      std::numeric_limits<double>::infinity();
  int last_search_terminal_position_visits_ = 0;
  int last_search_processed_iterations_ = 0;
  bool last_search_anchor_queue_exhausted_ = false;

  std::vector<bool> clearance_map_;
  unsigned int cm_width_, cm_height_;
  std::vector<double> risk_heuristic_field_;
  // Admissible obstacle/corridor-aware distance-to-go used by fine search.
  // Unlike risk_heuristic_field_, this field contains distance only, so it
  // can strengthen the anchor heuristic without changing the objective.
  std::vector<double> holonomic_heuristic_field_;

  bool loadTerrainMap(const std::string& path, TerrainMap& map);
  float bilinearQuery(const TerrainMap& map, const std::vector<float>& layer,
                      double x, double y) const;
  float getDEMZ(double x, double y) const;
  double queryTerrainCost(double x, double y, double theta) const;
  struct TerrainAttitude {
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
    double pitch_risk = 0.0;
    double roll_risk = 0.0;
    bool feasible = true;
  };
  struct TerrainSample {
    double risk = 0.0;
    TerrainAttitude attitude;
  };
  TerrainSample queryTerrainSample(
      double x, double y, double theta) const;
  TerrainAttitude queryTerrainAttitude(
      double x, double y, double theta) const;
  double attitudeSoftRisk(double absolute_angle, double soft_start,
                          double hard_limit) const;
  double computeHighRiskThreshold(const TerrainMap& map) const;

  struct PathMetrics {
    double length = 0.0;
    double mean_risk = 0.0;
    double peak_risk = 0.0;
    double high_risk_fraction = 0.0;
    double tail_risk = 0.0;
    double combined_risk_objective = 0.0;
    double mean_abs_pitch_deg = 0.0;
    double p95_abs_pitch_deg = 0.0;
    double max_abs_pitch_deg = 0.0;
    double pitch_exposure_fraction = 0.0;
    double pitch_objective = 0.0;
    double mean_abs_roll_deg = 0.0;
    double p95_abs_roll_deg = 0.0;
    double max_abs_roll_deg = 0.0;
    double roll_exposure_fraction = 0.0;
    double roll_objective = 0.0;
    bool attitude_feasible = true;
  };
  struct CoarseSearchAudit {
    int accepted_channels = 0;
    int risk_anchor_attempts = 0;
    int exposure_anchor_attempts = 0;
    int additional_attempts = 0;
    int failed_attempts = 0;
    int duplicate_rejections = 0;
    int detour_rejections = 0;
    int generated_pool_count = 0;
    int pruned_pool_count = 0;
    int label_generated = 0;
    int label_dominance_pruned = 0;
    int label_cap_pruned = 0;
    int goal_labels = 0;
    int goal_topology_count = 0;
    int goal_signature_count = 0;
    int fine_refinement_limit = 0;
    int fine_refinement_selected = 0;
    int fine_budget_pruned = 0;
    std::uint64_t topology_signature_labels_admitted = 0;
    std::uint64_t topology_signature_cap_protected = 0;
    int dominated_fine_rejections = 0;
    int fine_geometric_similarity_matches = 0;
    int fine_objective_equivalent_rejections = 0;
    int fine_same_topology_dominated_rejections = 0;
    int fine_same_topology_tradeoffs = 0;
    int fine_same_topology_replacements = 0;
    int smoothed_fine_candidates = 0;
    int raw_fine_fallbacks = 0;
    int attitude_fine_fallbacks = 0;
    int adaptive_safe_attempts = 0;
    int adaptive_safe_candidates = 0;
    int coarse_stop_iteration = 0;
    double time_budget_s = 0.0;
    double elapsed_s = 0.0;
    double fine_mean_risk_weight = 0.0;
    double fine_high_risk_weight = 0.0;
    double fine_pointwise_risk_weight = 0.0;
    double adaptive_safe_effective_peak_trigger = 0.0;
    double adaptive_safe_generation_detour_limit = 0.0;
    double adaptive_safe_reference_length_m = 0.0;
    double adaptive_safe_reference_mean_risk = 0.0;
    double adaptive_safe_reference_combined_risk = 0.0;
    double adaptive_safe_reference_peak_risk = 0.0;
    double adaptive_safe_reference_pitch_objective = 0.0;
    double adaptive_safe_reference_roll_objective = 0.0;
    double adaptive_safe_reference_max_pitch_deg = 0.0;
    double adaptive_safe_reference_max_roll_deg = 0.0;
    double adaptive_safe_candidate_length_m = 0.0;
    double adaptive_safe_candidate_detour_ratio = 0.0;
    double adaptive_safe_candidate_mean_risk = 0.0;
    double adaptive_safe_candidate_combined_risk = 0.0;
    double adaptive_safe_candidate_peak_risk = 0.0;
    double adaptive_safe_candidate_pitch_objective = 0.0;
    double adaptive_safe_candidate_roll_objective = 0.0;
    double adaptive_safe_candidate_max_pitch_deg = 0.0;
    double adaptive_safe_candidate_max_roll_deg = 0.0;
    double adaptive_safe_coarse_length_bound_m = 0.0;
    double adaptive_safe_fine_length_bound_m = 0.0;
    std::uint64_t adaptive_safe_coarse_length_pruned = 0;
    std::uint64_t adaptive_safe_fine_length_pruned = 0;
    std::string risk_anchor_status = "not_attempted";
    std::string exposure_anchor_status = "not_attempted";
    std::string adaptive_safe_status = "not_triggered";
    std::string adaptive_safe_trigger = "none";
    std::string coarse_stop_reason = "not_started";
    std::vector<std::string> fine_channel_roles;
    std::vector<std::string> fine_channel_status;
  };
  PathMetrics evaluatePathMetrics(
      const std::vector<geometry_msgs::PoseStamped>& path) const;
  bool isParetoSafetyFeasible(const PathMetrics& metrics) const;
  double computeParetoSafetyViolation(const PathMetrics& metrics) const;
  int selectParetoCandidate(
      const std::vector<PathMetrics>& metrics,
      std::vector<bool>* on_pareto_front = nullptr,
      std::vector<double>* ideal_distances = nullptr) const;
  void publishParetoDecision(
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
      const ros::Time& stamp) const;

  void buildCorridorMask(const std::vector<geometry_msgs::PoseStamped>& path);
  bool isInCorridor(double x, double y) const;
  double queryCorridorCenterDistance(double x, double y) const;
  void clearDiversityPenalty();
  void addDiversityPenalty(
      const std::vector<geometry_msgs::PoseStamped>& path);
  double queryDiversityPenalty(double x, double y) const;
  double pathHausdorffDistance(
      const std::vector<geometry_msgs::PoseStamped>& first,
      const std::vector<geometry_msgs::PoseStamped>& second) const;
  double pathOverlapRatio(
      const std::vector<geometry_msgs::PoseStamped>& first,
      const std::vector<geometry_msgs::PoseStamped>& second) const;
  double pathLongestSeparatedLength(
      const std::vector<geometry_msgs::PoseStamped>& first,
      const std::vector<geometry_msgs::PoseStamped>& second,
      double separation_threshold) const;
  bool isDistinctPath(
      const std::vector<geometry_msgs::PoseStamped>& candidate,
      const std::vector<std::vector<geometry_msgs::PoseStamped>>&
          accepted_paths) const;
  bool isDistinctFinePath(
      const std::vector<geometry_msgs::PoseStamped>& candidate,
      const std::vector<std::vector<geometry_msgs::PoseStamped>>&
          accepted_paths) const;
  bool areFinePathsGeometricallySimilar(
      const std::vector<geometry_msgs::PoseStamped>& first,
      const std::vector<geometry_msgs::PoseStamped>& second) const;

  void publishPathMarkers(
      const ros::Publisher& publisher,
      const std::vector<std::vector<geometry_msgs::PoseStamped>>& paths,
      const std::string& marker_namespace,
      const std::string& frame,
      const ros::Time& stamp,
      double line_width,
      double alpha) const;
  void publishCorridorMarkers(
      const std::vector<std::vector<bool>>& corridor_masks,
      const std::string& frame,
      const ros::Time& stamp) const;
  void publishPlannerStatistics(
      const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal,
      const std::vector<geometry_msgs::PoseStamped>& plan,
      bool success, const std::string& failure_reason,
      int coarse_channels, int fine_candidates, int selected_candidate,
      double coarse_time_s) const;

  bool smoothPath(std::vector<geometry_msgs::PoseStamped>& plan,
                  const std::string& frame, const ros::Time& stamp) const;

  void buildClearanceMap();
  bool isValid(double x, double y) const;
  bool isTraversable(double x, double y) const;
  bool isPathCollisionFree(double x0, double y0, double x1, double y1) const;
  double getCellCost(double x, double y) const;
  double heuristic(double x, double y, double gx, double gy) const;
  void buildRiskHeuristicField(double gx, double gy);
  void buildHolonomicHeuristicField(double gx, double gy);
  bool holonomicFieldReaches(double x, double y) const;
  double holonomicHeuristic(double x, double y,
                            double gx, double gy) const;
  double riskHeuristic(double x, double y, double theta,
                       double gx, double gy) const;
  double headingHeuristic(double x, double y, double theta,
                          double gx, double gy, double gt) const;
  int toKey(int ix, int iy, int itheta, int steering_index = -1) const;
  void expand(const Node3D& node, std::vector<Node3D>& successors) const;

  bool dubinsShot(const Node3D& from, double gx, double gy, double gt,
                  std::vector<geometry_msgs::PoseStamped>& path,
                  const std::string& frame, const ros::Time& stamp) const;
  double evaluateAnalyticPathCost(
      const std::vector<geometry_msgs::PoseStamped>& path) const;
  bool isAnalyticPathTerrainSafe(
      const std::vector<geometry_msgs::PoseStamped>& path) const;
  void buildPath(const std::vector<Node3D>& nodes, int goal_idx,
                 const std::string& frame, const ros::Time& stamp,
                 std::vector<geometry_msgs::PoseStamped>& plan) const;

  bool runSearch(double sx, double sy, double st,
                 double gx, double gy, double gt,
                 const std::string& frame, const ros::Time& stamp,
                 std::vector<geometry_msgs::PoseStamped>& plan,
                 double step_override, int max_iter_override,
                 bool use_multi_heuristic);
  bool runParetoCoarseSearch(
      double sx, double sy, double st,
      double gx, double gy, double gt,
      const std::string& frame, const ros::Time& stamp,
      std::vector<std::vector<geometry_msgs::PoseStamped>>& plans,
      double step_override, int max_iter_override,
      CoarseSearchAudit* audit);
};

class BasicHybridAStarPlanner : public HybridAStarPlanner {
public:
  BasicHybridAStarPlanner();

  void initialize(std::string name,
                  costmap_2d::Costmap2DROS* costmap_ros) override;
};

}  // namespace my_global_planner

#endif
