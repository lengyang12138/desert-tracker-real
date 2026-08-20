#pragma once

#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <mutex>
#include <deque>
#include <vector>
#include <string>

namespace mpc_local_planner {

struct TerrainMap {
    std::vector<float> dem, grad_x, grad_y, cost_total;
    int rows, cols;
    float res, origin_x, origin_y;
    float friction_coeff, track_width, cg_height;
    bool loaded;
    TerrainMap() : rows(0), cols(0), res(0), origin_x(0), origin_y(0),
                   friction_coeff(0.4f), track_width(1.277f), cg_height(0.5f),
                   loaded(false) {}
};

class MpcLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    MpcLocalPlanner();
    ~MpcLocalPlanner() override = default;

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    struct State {
        double x, y, theta;
        double v, omega;
        // Body-frame lateral velocity not represented by the nominal
        // nonholonomic model. Positive is to the vehicle's left.
        double lateral_disturbance;
    };

    struct PathProjection {
        int segment_index;
        double fraction;
        double x, y;
        double signed_cte;
        double distance;
        bool valid;
    };

    struct CommandSample {
        ros::Time stamp;
        double v;
        double omega;
    };

    struct Params {
        double tau_v, tau_omega;
        double linear_tracking_gain;
        bool enable_linear_gain_adaptation;
        double linear_gain_min, linear_gain_max;
        double linear_gain_filter_alpha;
        double linear_gain_min_command;
        double linear_gain_stable_delta;
        int linear_gain_stable_cycles;
        double omega_tracking_gain;
        bool enable_omega_gain_adaptation;
        double omega_gain_min, omega_gain_max;
        double omega_gain_filter_alpha;
        double omega_gain_rise_alpha;
        double omega_gain_min_command;
        double omega_gain_stable_delta;
        int omega_gain_stable_cycles;
        bool enable_time_constant_adaptation;
        double tau_v_min, tau_v_max;
        double tau_omega_min, tau_omega_max;
        double tau_filter_alpha;
        double tau_min_response_delta;
        double command_alignment_delay;
        double command_alignment_max_age;
        double angular_deadband;
        double state_timeout;
        double tf_timeout;
        double max_position_covariance;
        double max_yaw_covariance;
        double max_control_cycle_interval;
        double solver_time_limit;
        int max_consecutive_solver_timeouts;
        bool require_localization_quality;
        double localization_quality_timeout;
        int projection_search_back_segments;
        int projection_search_forward_segments;
        double projection_global_fallback_distance;
        double max_vel_x, max_vel_x_backwards, max_vel_theta;
        double acc_lim_x, acc_lim_theta;
        double curve_acc_lim_theta, curve_threshold;
        double prediction_dt;
        int prediction_steps;
        int num_omega_samples;
        int num_v_samples;
        int control_hold_steps;
        int num_second_omega_samples;
        double weight_cte, weight_heading, weight_omega, weight_omega_rate;
        double weight_speed, weight_v_rate;
        double weight_curvature_feedforward;
        double weight_obstacle;
        double goal_tolerance_xy, goal_tolerance_yaw;
        double curvature_decel, goal_decel_dist, min_vel_x;
        double terminal_recovery_speed;
        double terminal_alignment_distance;
        double terminal_bearing_tolerance;
        double terminal_bearing_gain;

        double heading_lookahead;
        double cte_heading_gain;
        double cte_heading_soft_speed;
        double max_cte_heading_correction;
        bool enable_turn_authority_assist;
        double turn_assist_cte_threshold;
        double turn_assist_release_cte_threshold;
        double turn_assist_heading_threshold;
        double turn_assist_time_constant;
        double turn_assist_cte_rate_alpha;
        double turn_assist_cte_rate_threshold;
        double turn_assist_cte_rate_gain;
        double turn_assist_rate_heading_alpha;
        double turn_assist_max_rate_heading;
        double turn_assist_release_time;
        bool enable_lateral_disturbance_observer;
        double lateral_disturbance_time_constant;
        double lateral_disturbance_release_time_constant;
        double lateral_disturbance_min_speed;
        double lateral_disturbance_deadband;
        double max_lateral_disturbance;
        double max_lateral_disturbance_heading;
        double terrain_decel;
        double slip_compensation_gain;
    };

    State predictStep(const State& s, double v_cmd, double omega_cmd) const;
    PathProjection projectOntoPath(double x, double y,
                                   int segment_hint = -1) const;
    double projectionHeading(const PathProjection& projection) const;
    double projectionCurvature(const PathProjection& projection) const;
    double pathHeading(int idx) const;
    double pathCurvature(int idx) const;
    double desiredSpeed(int idx, double dist_to_goal, double x, double y) const;
    double evaluateOmegaSequence(const State& state, double v_cmd,
                                 double v_reference,
                                 double omega_first, double omega_second,
                                 std::vector<State>* trajectory) const;
    void updateResponseModel(double actual_v, double actual_omega,
                             const ros::Time& measurement_stamp);
    void updateOmegaTrackingGain(double actual_omega,
                                 double aligned_omega_command);
    void recordIssuedCommand(double v, double omega);
    bool commandAtMeasurement(const ros::Time& measurement_stamp,
                              double& v, double& omega,
                              double& alignment_age) const;
    double effectiveLinearTrackingGain() const;
    double effectiveOmegaTrackingGain() const;
    double effectiveTauV() const;
    double effectiveTauOmega() const;
    void updateLateralDisturbanceEstimate(double heading_error,
                                          double cte_prediction_residual,
                                          double prediction_dt,
                                          bool measurement_valid);
    double lateralDisturbanceHeading(double forward_v,
                                     double lateral_disturbance) const;
    bool updatePlanInCostmapFrame();
    bool controlInputsHealthy(const geometry_msgs::PoseStamped& robot_pose,
                              const ros::Time& measurement_stamp,
                              double position_covariance,
                              double yaw_covariance,
                              double& state_age,
                              double& tf_age) const;
    void issueSafeStop(geometry_msgs::Twist& cmd_vel,
                       const char* reason);

    bool loadTerrainMap(const std::string& path, TerrainMap& map);
    float bilinearQuery(const TerrainMap& map, const std::vector<float>& layer,
                        double x, double y) const;
    bool controlToTerrainFrame(double x, double y, double theta,
                               double& terrain_x, double& terrain_y,
                               double& terrain_theta) const;
    float queryTerrainCost(double x, double y) const;
    double querySlipOmega(double x, double y, double theta, double v) const;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void localizationQualityCallback(const std_msgs::Bool::ConstPtr& msg);

    Params params_;
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    TerrainMap terrain_map_;
    std::string terrain_cost_file_;

    // move_base supplies the global plan in the planner frame (map), while
    // the rolling local costmap normally operates in odom. Keep the source
    // plan untouched and rebuild this transformed working copy each cycle.
    std::vector<geometry_msgs::PoseStamped> source_plan_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    // Latest transform from the terrain/plan frame into the rolling local
    // costmap frame. Terrain arrays are stored in the plan frame, while MPC
    // states normally live in odom, so every terrain query must apply the
    // inverse of this transform first.
    double plan_to_control_x_, plan_to_control_y_, plan_to_control_yaw_;
    bool terrain_query_transform_valid_;
    bool initialized_, goal_reached_;
    double prev_v_cmd_, prev_omega_cmd_;
    double current_v_, current_lateral_v_, current_omega_;
    ros::Time current_state_measurement_stamp_;
    double current_position_covariance_;
    double current_yaw_covariance_;
    double observed_lateral_disturbance_;
    double estimated_lateral_disturbance_;
    ros::Time last_lateral_disturbance_update_time_;
    double identified_linear_gain_;
    double identified_omega_gain_;
    double identified_tau_v_, identified_tau_omega_;
    double previous_measured_v_, previous_measured_omega_;
    ros::Time last_response_update_time_;
    ros::Time last_response_measurement_stamp_;
    double aligned_response_v_cmd_, aligned_response_omega_cmd_;
    double response_command_alignment_age_;
    bool response_model_initialized_;
    double linear_gain_probe_cmd_;
    int linear_gain_stable_cycles_;
    double gain_probe_cmd_;
    int gain_stable_cycles_;
    double previous_signed_cte_;
    double filtered_cte_rate_;
    double held_rate_heading_correction_;
    double turn_assist_improving_time_;
    ros::Time last_cte_update_time_;
    bool turn_assist_latched_;
    bool one_step_cte_prediction_valid_;
    double one_step_predicted_cte_;
    double one_step_prediction_residual_;
    ros::Time one_step_prediction_created_time_;
    int projection_segment_cache_;
    ros::WallTime last_control_cycle_start_;
    double last_control_cycle_interval_;
    double last_solver_duration_;
    bool last_solver_timed_out_;
    int last_evaluated_candidates_;
    int consecutive_solver_timeouts_;
    bool localization_quality_ok_;
    ros::Time last_localization_quality_time_;
    std::deque<CommandSample> command_history_;
    std::mutex odom_mutex_;
    mutable std::mutex localization_quality_mutex_;
    mutable std::mutex command_history_mutex_;

    ros::Subscriber odom_sub_;
    ros::Subscriber localization_quality_sub_;
    ros::Publisher local_plan_pub_;
    // Flattened rows [t, x, y, yaw, v, omega, cte, heading_error] for each
    // point of the selected prediction horizon. This is diagnostics-only.
    ros::Publisher prediction_diagnostics_pub_;
    // Read-only controller diagnostics for experiment evaluation. Publishing
    // these values must not feed back into the control law.
    ros::Publisher diagnostics_pub_;
};

}  // namespace mpc_local_planner
