#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

double normalizeAngle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

Eigen::Matrix4f transformToMatrix(const geometry_msgs::Transform& t)
{
  Eigen::Quaternionf q(static_cast<float>(t.rotation.w),
                       static_cast<float>(t.rotation.x),
                       static_cast<float>(t.rotation.y),
                       static_cast<float>(t.rotation.z));
  q.normalize();
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  out.block<3, 3>(0, 0) = q.toRotationMatrix();
  out(0, 3) = static_cast<float>(t.translation.x);
  out(1, 3) = static_cast<float>(t.translation.y);
  out(2, 3) = static_cast<float>(t.translation.z);
  return out;
}

Eigen::Matrix4f makeSe2(double x, double y, double yaw)
{
  Eigen::Matrix4f out = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(yaw));
  const float s = static_cast<float>(std::sin(yaw));
  out(0, 0) = c;
  out(0, 1) = -s;
  out(1, 0) = s;
  out(1, 1) = c;
  out(0, 3) = static_cast<float>(x);
  out(1, 3) = static_cast<float>(y);
  return out;
}

double yawOf(const Eigen::Matrix4f& t)
{
  return std::atan2(static_cast<double>(t(1, 0)), static_cast<double>(t(0, 0)));
}

Eigen::Matrix4f se2Part(const Eigen::Matrix4f& t)
{
  return makeSe2(t(0, 3), t(1, 3), yawOf(t));
}

geometry_msgs::Transform matrixToTransform(const Eigen::Matrix4f& t)
{
  geometry_msgs::Transform out;
  out.translation.x = t(0, 3);
  out.translation.y = t(1, 3);
  out.translation.z = t(2, 3);
  Eigen::Quaternionf q(t.block<3, 3>(0, 0));
  q.normalize();
  out.rotation.x = q.x();
  out.rotation.y = q.y();
  out.rotation.z = q.z();
  out.rotation.w = q.w();
  return out;
}

}  // namespace

class FixedMapLocalizer
{
public:
  FixedMapLocalizer()
    : pnh_("~"), tf_listener_(tf_buffer_), map_(new CloudT), map_ds_(new CloudT),
      local_map_(new CloudT), map_to_base_(Eigen::Matrix4f::Identity())
  {
    pnh_.param<std::string>("map_file", map_file_, std::string());
    pnh_.param<std::string>("scan_topic", scan_topic_, "/lio_sam/deskew/cloud_deskewed");
    pnh_.param<std::string>("motion_odom_topic", motion_odom_topic_,
                            "/odometry/imu_incremental");
    pnh_.param<std::string>("initial_pose_topic", initial_pose_topic_,
                            "/localization/qianxun_seed");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<double>("map_leaf_size", map_leaf_size_, 0.35);
    pnh_.param<double>("scan_leaf_size", scan_leaf_size_, 0.25);
    pnh_.param<double>("local_map_radius", local_map_radius_, 22.0);
    pnh_.param<double>("ndt_resolution", ndt_resolution_, 0.8);
    pnh_.param<double>("ndt_step_size", ndt_step_size_, 0.15);
    pnh_.param<double>("transformation_epsilon", transformation_epsilon_, 0.01);
    pnh_.param<int>("maximum_iterations", maximum_iterations_, 35);
    pnh_.param<double>("max_fitness_score", max_fitness_score_, 0.8);
    pnh_.param<double>("max_innovation_xy", max_innovation_xy_, 1.5);
    pnh_.param<double>("max_innovation_yaw_deg", max_innovation_yaw_deg_, 8.0);
    pnh_.param<double>("correction_gain", correction_gain_, 0.20);
    pnh_.param<double>("max_correction_step", max_correction_step_, 0.04);
    pnh_.param<double>("max_yaw_step_deg", max_yaw_step_deg_, 0.5);
    pnh_.param<double>("tf_publish_rate", tf_publish_rate_, 30.0);
    pnh_.param<double>("correction_update_deadband_xy",
                       correction_update_deadband_xy_, 0.025);
    pnh_.param<double>("correction_update_deadband_yaw_deg",
                       correction_update_deadband_yaw_deg_, 0.15);
    pnh_.param<double>("motion_velocity_alpha", motion_velocity_alpha_, 0.15);
    pnh_.param<double>("forward_velocity_scale", forward_velocity_scale_, 0.97);
    pnh_.param<double>("lateral_velocity_scale", lateral_velocity_scale_, 0.0);
    pnh_.param<double>("max_motion_dt", max_motion_dt_, 0.05);
    pnh_.param<double>("max_forward_speed", max_forward_speed_, 0.65);
    pnh_.param<double>("max_lateral_speed", max_lateral_speed_, 0.25);
    pnh_.param<double>("max_yaw_rate", max_yaw_rate_, 0.60);
    pnh_.param<int>("process_every_n_scans", process_every_n_scans_, 5);
    pnh_.param<int>("required_consecutive_matches", required_consecutive_matches_, 2);
    pnh_.param<int>("minimum_scan_points", minimum_scan_points_, 300);
    pnh_.param<int>("minimum_map_points", minimum_map_points_, 1000);
    pnh_.param<std::string>("quality_topic", quality_topic_,
                            "/localization/quality_ok");
    pnh_.param<double>("quality_timeout", quality_timeout_, 1.0);
    pnh_.param<bool>("require_qianxun_initialization",
                     require_qianxun_initialization_, true);
    pnh_.param<double>("initial_pose_timeout", initial_pose_timeout_, 1.0);
    pnh_.param<double>("max_initial_pose_covariance",
                       max_initial_pose_covariance_, 1.0);
    quality_timeout_ = std::max(0.1, quality_timeout_);
    initial_pose_timeout_ = std::max(0.1, initial_pose_timeout_);

    if (map_file_.empty())
      throw std::runtime_error("~map_file is required");
    if (pcl::io::loadPCDFile<PointT>(map_file_, *map_) < 0 || map_->empty())
      throw std::runtime_error("failed to load fixed PCD map: " + map_file_);

    pcl::VoxelGrid<PointT> map_filter;
    map_filter.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    map_filter.setInputCloud(map_);
    map_filter.filter(*map_ds_);
    map_.reset();
    map_tree_.reset(new pcl::search::KdTree<PointT>);
    map_tree_->setInputCloud(map_ds_);

    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(transformation_epsilon_);
    ndt_.setMaximumIterations(maximum_iterations_);

    scan_sub_ = nh_.subscribe(scan_topic_, 1, &FixedMapLocalizer::scanCallback, this,
                              ros::TransportHints().tcpNoDelay());
    motion_odom_sub_ = nh_.subscribe(
        motion_odom_topic_, 2000, &FixedMapLocalizer::motionOdometryCallback,
        this, ros::TransportHints().tcpNoDelay());
    initial_pose_sub_ = nh_.subscribe(
        initial_pose_topic_, 5, &FixedMapLocalizer::initialPoseCallback, this,
        ros::TransportHints().tcpNoDelay());
    odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("odometry", 5);
    target_odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("target_odometry", 5);
    local_map_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("local_map", 1, true);
    quality_pub_ = nh_.advertise<std_msgs::Bool>(quality_topic_, 1, true);
    tf_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(tf_publish_rate_, 1.0)),
        &FixedMapLocalizer::tfTimerCallback, this);

    ROS_INFO_STREAM("Fixed-map localizer ready: " << map_ds_->size() << " map points, scan="
                    << scan_topic_ << ", map=" << map_file_
                    << ", initial_pose=" << initial_pose_topic_
                    << ", require_qianxun=" << require_qianxun_initialization_);
  }

private:
  bool lookup(const std::string& target, const std::string& source, const ros::Time& stamp,
              Eigen::Matrix4f& out)
  {
    try
    {
      const geometry_msgs::TransformStamped tf =
          tf_buffer_.lookupTransform(target, source, stamp, ros::Duration(0.08));
      out = transformToMatrix(tf.transform);
      return true;
    }
    catch (const tf2::TransformException& e)
    {
      ROS_WARN_THROTTLE(2.0, "Fixed-map TF lookup failed (%s <- %s): %s",
                        target.c_str(), source.c_str(), e.what());
      return false;
    }
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (msg->header.frame_id != map_frame_)
    {
      ROS_WARN_THROTTLE(2.0, "Qianxun seed rejected: frame '%s' is not '%s'",
                        msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y +
                                    q.z * q.z + q.w * q.w);
    const double covariance_xy = std::max(msg->pose.covariance[0],
                                          msg->pose.covariance[7]);
    const double covariance_yaw = msg->pose.covariance[35];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(q_norm) || q_norm < 1e-6 ||
        !std::isfinite(covariance_xy) || !std::isfinite(covariance_yaw) ||
        covariance_xy < 0.0 || covariance_yaw < 0.0 ||
        covariance_xy > max_initial_pose_covariance_ ||
        covariance_yaw > max_initial_pose_covariance_)
    {
      ROS_WARN_THROTTLE(2.0, "Qianxun seed rejected: invalid pose or covariance");
      return;
    }

    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    std::lock_guard<std::mutex> lock(initial_pose_mutex_);
    initial_map_to_base_ = makeSe2(p.x, p.y, yaw);
    initial_pose_received_time_ = ros::Time::now();
    initial_pose_available_ = true;
  }

  bool initialPoseSnapshot(const ros::Time& now, Eigen::Matrix4f& pose)
  {
    std::lock_guard<std::mutex> lock(initial_pose_mutex_);
    if (!initial_pose_available_ ||
        (now - initial_pose_received_time_).toSec() > initial_pose_timeout_)
      return false;
    pose = initial_map_to_base_;
    return true;
  }

  bool initializeState(const Eigen::Matrix4f& odom_to_base,
                       const ros::Time& stamp)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_initialized_)
        return false;
    }

    Eigen::Matrix4f initial_pose;
    const bool have_initial_pose = initialPoseSnapshot(ros::Time::now(), initial_pose);
    if (require_qianxun_initialization_ && !have_initial_pose)
    {
      ROS_WARN_THROTTLE(2.0, "Fixed-map localization waiting for a healthy Qianxun seed");
      return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_initialized_)
      return false;

    // Qianxun supplies the absolute map pose once. From this point on, pose
    // propagation uses LIO velocity and NDT corrections; GNSS does not
    // continuously overwrite the independent fixed-map localization result.
    map_to_base_ = have_initial_pose
                       ? initial_pose
                       : makeSe2(odom_to_base(0, 3), odom_to_base(1, 3),
                                 yawOf(odom_to_base));
    last_motion_stamp_ = stamp;
    state_initialized_ = true;
    ROS_INFO("Fixed-map localization initialized from %s at x=%.3f y=%.3f yaw=%.2fdeg",
             have_initial_pose ? "Qianxun" : "local odometry",
             map_to_base_(0, 3), map_to_base_(1, 3),
             yawOf(map_to_base_) * 180.0 / M_PI);
    return true;
  }

  bool stateSnapshot(Eigen::Matrix4f& state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_initialized_)
      return false;
    state = map_to_base_;
    return true;
  }

  void motionOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_initialized_)
    {
      last_motion_stamp_ = msg->header.stamp;
      return;
    }

    const double dt = (msg->header.stamp - last_motion_stamp_).toSec();
    last_motion_stamp_ = msg->header.stamp;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_motion_dt_)
      return;

    const double raw_yaw = std::atan2(
        2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
               msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
        1.0 - 2.0 * (msg->pose.pose.orientation.y *
                         msg->pose.pose.orientation.y +
                     msg->pose.pose.orientation.z *
                         msg->pose.pose.orientation.z));
    const double velocity_x = msg->twist.twist.linear.x;
    const double velocity_y = msg->twist.twist.linear.y;
    double forward_velocity =
        std::cos(raw_yaw) * velocity_x + std::sin(raw_yaw) * velocity_y;
    double lateral_velocity =
        -std::sin(raw_yaw) * velocity_x + std::cos(raw_yaw) * velocity_y;
    double yaw_rate = msg->twist.twist.angular.z;
    if (!std::isfinite(forward_velocity) ||
        !std::isfinite(lateral_velocity) || !std::isfinite(yaw_rate))
      return;

    // The IMU-preintegrated world velocity has an accurate forward component
    // in this setup, but its body-lateral component is dominated by tilt and
    // bias (nav138: correlation with GT only 0.21). A tracked vehicle is
    // locally non-holonomic, so leave real lateral displacement to the NDT
    // correction instead of integrating that noise indefinitely.
    forward_velocity *= forward_velocity_scale_;
    lateral_velocity *= lateral_velocity_scale_;
    forward_velocity = std::max(
        -max_forward_speed_, std::min(max_forward_speed_, forward_velocity));
    lateral_velocity = std::max(
        -max_lateral_speed_, std::min(max_lateral_speed_, lateral_velocity));
    yaw_rate =
        std::max(-max_yaw_rate_, std::min(max_yaw_rate_, yaw_rate));

    const double alpha =
        std::max(0.0, std::min(1.0, motion_velocity_alpha_));
    filtered_forward_velocity_ +=
        alpha * (forward_velocity - filtered_forward_velocity_);
    filtered_lateral_velocity_ +=
        alpha * (lateral_velocity - filtered_lateral_velocity_);
    filtered_yaw_rate_ += alpha * (yaw_rate - filtered_yaw_rate_);

    const double yaw = yawOf(map_to_base_);
    const double dx =
        (std::cos(yaw) * filtered_forward_velocity_ -
         std::sin(yaw) * filtered_lateral_velocity_) *
        dt;
    const double dy =
        (std::sin(yaw) * filtered_forward_velocity_ +
         std::cos(yaw) * filtered_lateral_velocity_) *
        dt;
    map_to_base_ = makeSe2(
        map_to_base_(0, 3) + dx, map_to_base_(1, 3) + dy,
        normalizeAngle(yaw + filtered_yaw_rate_ * dt));
  }

  void tfTimerCallback(const ros::TimerEvent&)
  {
    std_msgs::Bool quality;
    {
      std::lock_guard<std::mutex> lock(quality_mutex_);
      quality.data = !last_trusted_match_time_.isZero() &&
                     (ros::Time::now() - last_trusted_match_time_).toSec() <=
                         quality_timeout_;
    }
    quality_pub_.publish(quality);

    Eigen::Matrix4f odom_to_base;
    const ros::Time stamp = ros::Time::now();
    if (!lookup(odom_frame_, base_frame_, ros::Time(0), odom_to_base))
      return;
    initializeState(odom_to_base, stamp);

    Eigen::Matrix4f map_to_base;
    if (!stateSnapshot(map_to_base))
      return;
    // map_to_base_ is a planar localization correction.  Do not cancel the
    // vehicle's terrain roll, pitch and height through map->odom: doing so
    // makes the published base pose artificially flat and changes its XY
    // projection whenever the vehicle enters or leaves a slope.
    const Eigen::Matrix4f odom_to_base_se2 = se2Part(odom_to_base);
    const Eigen::Matrix4f map_to_odom =
        map_to_base * odom_to_base_se2.inverse();
    const Eigen::Matrix4f map_to_base_full =
        map_to_odom * odom_to_base;

    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform = matrixToTransform(map_to_odom);
    tf_broadcaster_.sendTransform(tf);
    publishOdometry(stamp, map_to_base_full, odom_pub_);
  }

  void publishOdometry(const ros::Time& stamp, const Eigen::Matrix4f& map_to_base,
                       ros::Publisher& publisher)
  {
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = map_to_base(0, 3);
    msg.pose.pose.position.y = map_to_base(1, 3);
    msg.pose.pose.position.z = map_to_base(2, 3);
    const geometry_msgs::Transform t = matrixToTransform(map_to_base);
    msg.pose.pose.orientation = t.rotation;
    publisher.publish(msg);
  }

  bool buildLocalMap(const Eigen::Matrix4f& initial_guess)
  {
    PointT centre;
    centre.x = initial_guess(0, 3);
    centre.y = initial_guess(1, 3);
    centre.z = initial_guess(2, 3);
    centre.intensity = 0.0f;
    std::vector<int> indices;
    std::vector<float> sq_distances;
    map_tree_->radiusSearch(centre, local_map_radius_, indices, sq_distances);
    local_map_->clear();
    local_map_->reserve(indices.size());
    for (const int index : indices)
      local_map_->push_back((*map_ds_)[index]);
    local_map_->width = local_map_->size();
    local_map_->height = 1;
    local_map_->is_dense = true;
    return static_cast<int>(local_map_->size()) >= minimum_map_points_;
  }

  void updateStateFromMeasurement(const Eigen::Matrix4f& predicted_at_scan,
                                  const Eigen::Matrix4f& measured_at_scan)
  {
    // NDT finishes after the motion observer has usually propagated several
    // IMU samples. Apply the residual measured at scan time to the *current*
    // state instead of replacing it with an old pose.
    const double raw_dx =
        measured_at_scan(0, 3) - predicted_at_scan(0, 3);
    const double raw_dy =
        measured_at_scan(1, 3) - predicted_at_scan(1, 3);
    const double raw_distance = std::hypot(raw_dx, raw_dy);
    double dx = raw_distance >= correction_update_deadband_xy_
                    ? correction_gain_ * raw_dx
                    : 0.0;
    double dy = raw_distance >= correction_update_deadband_xy_
                    ? correction_gain_ * raw_dy
                    : 0.0;
    const double length = std::hypot(dx, dy);
    if (length > max_correction_step_ && length > 1e-9)
    {
      const double scale = max_correction_step_ / length;
      dx *= scale;
      dy *= scale;
    }
    const double max_yaw_step = max_yaw_step_deg_ * M_PI / 180.0;
    const double raw_yaw_error =
        normalizeAngle(yawOf(measured_at_scan) -
                       yawOf(predicted_at_scan));
    const double yaw_deadband =
        correction_update_deadband_yaw_deg_ * M_PI / 180.0;
    double dyaw = std::fabs(raw_yaw_error) >= yaw_deadband
                      ? correction_gain_ * raw_yaw_error
                      : 0.0;
    dyaw = std::max(-max_yaw_step, std::min(max_yaw_step, dyaw));

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_initialized_)
      return;
    map_to_base_ = makeSe2(
        map_to_base_(0, 3) + dx,
        map_to_base_(1, 3) + dy,
        normalizeAngle(yawOf(map_to_base_) + dyaw));
  }

  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    Eigen::Matrix4f odom_to_sensor;
    Eigen::Matrix4f odom_to_base;
    if (!lookup(odom_frame_, msg->header.frame_id, msg->header.stamp, odom_to_sensor) ||
        !lookup(odom_frame_, base_frame_, msg->header.stamp, odom_to_base))
      return;
    initializeState(odom_to_base, msg->header.stamp);

    ++scan_count_;
    if (process_every_n_scans_ > 1 && scan_count_ % process_every_n_scans_ != 0)
      return;

    CloudT::Ptr scan(new CloudT);
    CloudT::Ptr scan_ds(new CloudT);
    pcl::fromROSMsg(*msg, *scan);
    pcl::VoxelGrid<PointT> scan_filter;
    scan_filter.setLeafSize(scan_leaf_size_, scan_leaf_size_, scan_leaf_size_);
    scan_filter.setInputCloud(scan);
    scan_filter.filter(*scan_ds);
    if (static_cast<int>(scan_ds->size()) < minimum_scan_points_)
    {
      ROS_WARN_THROTTLE(2.0, "Fixed-map scan rejected: only %zu points", scan_ds->size());
      return;
    }

    Eigen::Matrix4f predicted_map_to_base;
    if (!stateSnapshot(predicted_map_to_base))
      return;
    const Eigen::Matrix4f base_to_sensor =
        odom_to_base.inverse() * odom_to_sensor;
    // Apply only the map/odom planar correction to the full sensor pose.
    // This preserves the current terrain Z/roll/pitch in the NDT initial
    // guess instead of forcing every scan to start from a level pose.
    const Eigen::Matrix4f odom_to_base_se2 = se2Part(odom_to_base);
    const Eigen::Matrix4f predicted_map_to_odom =
        predicted_map_to_base * odom_to_base_se2.inverse();
    const Eigen::Matrix4f initial_guess =
        predicted_map_to_odom * odom_to_sensor;
    if (!buildLocalMap(initial_guess))
    {
      ROS_WARN_THROTTLE(2.0, "Fixed-map local target too small: %zu points", local_map_->size());
      return;
    }

    ndt_.setInputSource(scan_ds);
    ndt_.setInputTarget(local_map_);
    CloudT aligned;
    ndt_.align(aligned, initial_guess);
    const double fitness = ndt_.getFitnessScore(2.0);
    const Eigen::Matrix4f fitted_map_to_sensor = ndt_.getFinalTransformation();
    const Eigen::Matrix4f measured_full =
        fitted_map_to_sensor * base_to_sensor.inverse();
    const Eigen::Matrix4f measured_map_to_base =
        makeSe2(measured_full(0, 3), measured_full(1, 3),
                yawOf(measured_full));
    const Eigen::Matrix4f innovation =
        predicted_map_to_base.inverse() * measured_map_to_base;
    const double innovation_xy = std::hypot(innovation(0, 3), innovation(1, 3));
    const double innovation_yaw_deg = std::abs(yawOf(innovation)) * 180.0 / M_PI;

    const bool accepted = ndt_.hasConverged() && std::isfinite(fitness) &&
                          fitness <= max_fitness_score_ &&
                          innovation_xy <= max_innovation_xy_ &&
                          innovation_yaw_deg <= max_innovation_yaw_deg_;

    if (accepted)
    {
      ++consecutive_matches_;
      consecutive_rejections_ = 0;
    }
    else
    {
      consecutive_matches_ = 0;
      ++consecutive_rejections_;
    }

    if (accepted &&
        consecutive_matches_ >= required_consecutive_matches_)
    {
      updateStateFromMeasurement(predicted_map_to_base,
                                 measured_map_to_base);
      std::lock_guard<std::mutex> lock(quality_mutex_);
      last_trusted_match_time_ = ros::Time::now();
    }

    publishOdometry(msg->header.stamp, measured_full,
                    target_odom_pub_);

    Eigen::Matrix4f state_after;
    stateSnapshot(state_after);
    ROS_INFO_THROTTLE(1.0,
        "Fixed-map NDT: %s matches=%d rejects=%d "
        "fitness=%.3f innovation=%.3fm/%.2fdeg "
        "observer=(%.3f, %.3f, %.2fdeg)",
        accepted ? "accepted" : "REJECTED",
        consecutive_matches_, consecutive_rejections_,
        fitness, innovation_xy, innovation_yaw_deg,
        state_after(0, 3), state_after(1, 3),
        yawOf(state_after) * 180.0 / M_PI);

    if (local_map_pub_.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 local_msg;
      pcl::toROSMsg(*local_map_, local_msg);
      local_msg.header.stamp = msg->header.stamp;
      local_msg.header.frame_id = map_frame_;
      local_map_pub_.publish(local_msg);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber motion_odom_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher target_odom_pub_;
  ros::Publisher local_map_pub_;
  ros::Publisher quality_pub_;
  ros::Timer tf_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string map_file_;
  std::string scan_topic_;
  std::string motion_odom_topic_;
  std::string initial_pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string quality_topic_;
  double map_leaf_size_;
  double scan_leaf_size_;
  double local_map_radius_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transformation_epsilon_;
  int maximum_iterations_;
  double max_fitness_score_;
  double max_innovation_xy_;
  double max_innovation_yaw_deg_;
  double correction_gain_;
  double max_correction_step_;
  double max_yaw_step_deg_;
  double tf_publish_rate_;
  double correction_update_deadband_xy_;
  double correction_update_deadband_yaw_deg_;
  double motion_velocity_alpha_;
  double forward_velocity_scale_;
  double lateral_velocity_scale_;
  double max_motion_dt_;
  double max_forward_speed_;
  double max_lateral_speed_;
  double max_yaw_rate_;
  int process_every_n_scans_;
  int required_consecutive_matches_;
  int minimum_scan_points_;
  int minimum_map_points_;
  double quality_timeout_;
  bool require_qianxun_initialization_;
  double initial_pose_timeout_;
  double max_initial_pose_covariance_;
  int scan_count_ = 0;
  int consecutive_matches_ = 0;
  int consecutive_rejections_ = 0;

  CloudT::Ptr map_;
  CloudT::Ptr map_ds_;
  CloudT::Ptr local_map_;
  pcl::search::KdTree<PointT>::Ptr map_tree_;
  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  Eigen::Matrix4f map_to_base_;
  ros::Time last_motion_stamp_;
  double filtered_forward_velocity_ = 0.0;
  double filtered_lateral_velocity_ = 0.0;
  double filtered_yaw_rate_ = 0.0;
  bool state_initialized_ = false;
  std::mutex state_mutex_;
  Eigen::Matrix4f initial_map_to_base_ = Eigen::Matrix4f::Identity();
  ros::Time initial_pose_received_time_;
  bool initial_pose_available_ = false;
  std::mutex initial_pose_mutex_;
  ros::Time last_trusted_match_time_;
  std::mutex quality_mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_map_localizer");
  try
  {
    FixedMapLocalizer node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Fixed-map localizer failed: %s", e.what());
    return 1;
  }
  return 0;
}
