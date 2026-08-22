#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace
{
bool point_cloud_received = false;
bool imu_received = false;

bool ensureOneDirectory(const std::string& directory)
{
  if (mkdir(directory.c_str(), 0755) == 0)
    return true;
  if (errno != EEXIST)
    return false;
  struct stat status {};
  return stat(directory.c_str(), &status) == 0 && S_ISDIR(status.st_mode);
}

bool ensureDirectory(const std::string& directory)
{
  if (directory.empty())
    return true;
  std::string current;
  for (std::size_t index = 0; index < directory.size(); ++index)
  {
    const char value = directory[index];
    current.push_back(value);
    if (value != '/' || current == "/")
      continue;
    const std::string component = current.substr(0, current.size() - 1);
    if (!component.empty() && !ensureOneDirectory(component))
      return false;
  }
  return ensureOneDirectory(directory);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr&)
{
  point_cloud_received = true;
}

void imuCallback(const sensor_msgs::ImuConstPtr&)
{
  imu_received = true;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mapping_bag_recorder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double warmup_seconds = 3.0;
  double topic_timeout_seconds = 5.0;
  int split_size_mb = 4096;
  std::string output_prefix;
  std::string points_topic = "/points_raw";
  std::string imu_topic = "/imu/data";
  pnh.param("warmup_seconds", warmup_seconds, warmup_seconds);
  pnh.param("topic_timeout_seconds", topic_timeout_seconds,
            topic_timeout_seconds);
  pnh.param("split_size_mb", split_size_mb, split_size_mb);
  pnh.param("output_prefix", output_prefix, output_prefix);
  pnh.param("points_topic", points_topic, points_topic);
  pnh.param("imu_topic", imu_topic, imu_topic);

  if (output_prefix.empty())
  {
    ROS_FATAL("~output_prefix must not be empty");
    return 2;
  }
  const std::size_t separator = output_prefix.find_last_of('/');
  const std::string output_directory =
      separator == std::string::npos ? std::string() :
      output_prefix.substr(0, separator);
  if (!ensureDirectory(output_directory))
  {
    ROS_FATAL("Cannot create rosbag output directory %s: %s",
              output_directory.c_str(), std::strerror(errno));
    return 2;
  }

  ROS_INFO("Waiting %.1f s for LiDAR, IMU and point conversion startup",
           warmup_seconds);
  ros::WallDuration(warmup_seconds).sleep();

  // Subscribe only after warm-up so readiness requires fresh sensor messages.
  ros::Subscriber points_sub = nh.subscribe(
      points_topic, 1, pointCloudCallback);
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 10, imuCallback);
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(topic_timeout_seconds);

  while (ros::ok() && ros::WallTime::now() < deadline &&
         !(point_cloud_received && imu_received))
  {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }

  if (!point_cloud_received || !imu_received)
  {
    ROS_FATAL("Sensor readiness failed: %s=%s, %s=%s; rosbag was not started",
              points_topic.c_str(), point_cloud_received ? "ready" : "missing",
              imu_topic.c_str(), imu_received ? "ready" : "missing");
    return 3;
  }

  ROS_INFO("Sensor readiness passed: received fresh %s and %s messages",
           points_topic.c_str(), imu_topic.c_str());

  // Replace this gate process with rosbag so roslaunch retains normal signal
  // forwarding and rosbag can close its index cleanly on Ctrl+C.
  std::vector<std::string> arguments = {
      "rosbag", "record", "--lz4", "--split",
      "--size=" + std::to_string(split_size_mb), "-O", output_prefix,
      points_topic, imu_topic, "/fusion_location", "/tf", "/tf_static"};
  std::vector<char*> exec_arguments;
  exec_arguments.reserve(arguments.size() + 1);
  for (std::string& argument : arguments)
    exec_arguments.push_back(&argument[0]);
  exec_arguments.push_back(nullptr);

  execvp(exec_arguments[0], exec_arguments.data());
  ROS_FATAL("Failed to execute rosbag: %s", std::strerror(errno));
  return 4;
}
