#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {

bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& name)
{
  for (const auto& field : cloud.fields)
    if (field.name == name)
      return true;
  return false;
}

const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& cloud, const std::string& name)
{
  for (const auto& field : cloud.fields)
    if (field.name == name)
      return &field;
  return nullptr;
}

sensor_msgs::PointCloud2 outputTemplate(const sensor_msgs::PointCloud2& input,
                                        const std::string& frame)
{
  sensor_msgs::PointCloud2 output;
  output.header = input.header;
  output.header.frame_id = frame;
  output.height = 1;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32,
      "ring", 1, sensor_msgs::PointField::UINT16,
      "time", 1, sensor_msgs::PointField::FLOAT32);
  return output;
}

}  // namespace

class RsToLioSam
{
public:
  RsToLioSam(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("input", input_topic_, "/rslidar_points");
    pnh.param<std::string>("output", output_topic_, "/points_raw");
    pnh.param<std::string>("output_frame", output_frame_, "velodyne");
    sub_ = nh.subscribe(input_topic_, 2, &RsToLioSam::callback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    ROS_INFO("RSHELIOS XYZIRT adapter: %s -> %s, frame=%s",
             input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
  }

private:
  template <typename TimeT>
  void convert(const sensor_msgs::PointCloud2ConstPtr& input,
               const std::string& time_field, bool absolute_time)
  {
    const std::size_t count =
        static_cast<std::size_t>(input->width) * input->height;
    sensor_msgs::PointCloud2 output = outputTemplate(*input, output_frame_);
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.resize(count);

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*input, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*input, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*input, "z");
    sensor_msgs::PointCloud2ConstIterator<float> in_intensity(*input, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> in_ring(*input, "ring");
    sensor_msgs::PointCloud2ConstIterator<TimeT> in_time(*input, time_field);
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(output, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(output, "ring");
    sensor_msgs::PointCloud2Iterator<float> out_time(output, "time");

    bool have_first_time = false;
    double first_time = 0.0;
    std::size_t valid = 0;
    for (std::size_t index = 0; index < count;
         ++index, ++in_x, ++in_y, ++in_z, ++in_intensity, ++in_ring, ++in_time)
    {
      const double sample_time = static_cast<double>(*in_time);
      if (!std::isfinite(*in_x) || !std::isfinite(*in_y) ||
          !std::isfinite(*in_z) || !std::isfinite(sample_time))
        continue;
      if (!have_first_time)
      {
        first_time = sample_time;
        have_first_time = true;
      }
      const double relative_time = absolute_time
          ? sample_time - first_time : sample_time;
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_intensity = *in_intensity;
      *out_ring = *in_ring;
      *out_time = static_cast<float>(std::max(0.0, relative_time));
      ++out_x; ++out_y; ++out_z; ++out_intensity; ++out_ring; ++out_time;
      ++valid;
    }
    if (valid == 0)
      return;
    modifier.resize(valid);
    pub_.publish(output);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    for (const char* field : {"x", "y", "z", "intensity", "ring"})
    {
      if (!hasField(*input, field))
      {
        ROS_ERROR_THROTTLE(1.0,
            "输入点云缺少字段%s；驱动必须输出XYZIRT", field);
        return;
      }
    }

    const std::size_t count = static_cast<std::size_t>(input->width) * input->height;
    if (count == 0)
      return;

    const sensor_msgs::PointField* time = findField(*input, "time");
    bool absolute_time = false;
    std::string time_name = "time";
    if (time == nullptr)
    {
      time = findField(*input, "timestamp");
      time_name = "timestamp";
      absolute_time = true;
    }
    if (time == nullptr)
    {
      ROS_ERROR_THROTTLE(1.0,
          "输入点云必须包含time或timestamp字段，禁止无时间去畸变");
      return;
    }
    if (time->datatype == sensor_msgs::PointField::FLOAT32)
      convert<float>(input, time_name, absolute_time);
    else if (time->datatype == sensor_msgs::PointField::FLOAT64)
      convert<double>(input, time_name, absolute_time);
    else
      ROS_ERROR_THROTTLE(1.0,
          "点时间字段%s必须是FLOAT32或FLOAT64，当前datatype=%u",
          time_name.c_str(), time->datatype);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rs_to_lio_sam");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RsToLioSam node(nh, pnh);
  ros::spin();
  return 0;
}
