#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

struct MapPosePoint
{
    PCL_ADD_POINT4D;
    float intensity;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    MapPosePoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
    (double, time, time))

namespace
{
Eigen::Matrix4f poseMatrix(
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    return (Eigen::Translation3f(
                static_cast<float>(x),
                static_cast<float>(y),
                static_cast<float>(z)) *
            Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(static_cast<float>(roll), Eigen::Vector3f::UnitX()))
        .matrix();
}

Eigen::Vector3f matrixRpy(const Eigen::Matrix4f& transform)
{
    const float pitch = std::asin(
        std::max(-1.0f, std::min(1.0f, -transform(2, 0))));
    const float roll = std::atan2(transform(2, 1), transform(2, 2));
    const float yaw = std::atan2(transform(1, 0), transform(0, 0));
    return Eigen::Vector3f(roll, pitch, yaw);
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_frame_transform");
    ros::NodeHandle private_nh("~");

    std::string input_file;
    std::string output_file;
    std::string reference_pose_file;
    std::string transform_info_output;
    double translation_x = 0.0;
    double translation_y = 0.0;
    double translation_z = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double sensor_x = 1.0;
    double sensor_y = 0.0;
    double sensor_z = 0.35;
    double sensor_roll_deg = 0.0;
    double sensor_pitch_deg = 0.0;
    double sensor_yaw_deg = 0.0;

    private_nh.param<std::string>("input", input_file, "");
    private_nh.param<std::string>("output", output_file, "");
    private_nh.param<std::string>("reference_pose_file", reference_pose_file, "");
    private_nh.param<std::string>("transform_info_output", transform_info_output, "");
    private_nh.param("translation_x", translation_x, translation_x);
    private_nh.param("translation_y", translation_y, translation_y);
    private_nh.param("translation_z", translation_z, translation_z);
    private_nh.param("roll_deg", roll_deg, roll_deg);
    private_nh.param("pitch_deg", pitch_deg, pitch_deg);
    private_nh.param("yaw_deg", yaw_deg, yaw_deg);
    private_nh.param("sensor_x", sensor_x, sensor_x);
    private_nh.param("sensor_y", sensor_y, sensor_y);
    private_nh.param("sensor_z", sensor_z, sensor_z);
    private_nh.param("sensor_roll_deg", sensor_roll_deg, sensor_roll_deg);
    private_nh.param("sensor_pitch_deg", sensor_pitch_deg, sensor_pitch_deg);
    private_nh.param("sensor_yaw_deg", sensor_yaw_deg, sensor_yaw_deg);

    if (input_file.empty() || output_file.empty())
    {
        ROS_FATAL("Both private parameters ~input and ~output are required.");
        return 1;
    }
    if (input_file == output_file)
    {
        ROS_FATAL("Refusing to overwrite the source PCD. Use a separate output path.");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr input(
        new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *input) < 0)
    {
        ROS_FATAL("Failed to load PCD: %s", input_file.c_str());
        return 1;
    }

    const double kDegToRad = std::acos(-1.0) / 180.0;
    Eigen::Matrix4f transform;
    MapPosePoint first_pose{};
    bool derived_from_first_pose = false;

    if (!reference_pose_file.empty())
    {
        pcl::PointCloud<MapPosePoint> poses;
        if (pcl::io::loadPCDFile<MapPosePoint>(reference_pose_file, poses) < 0 ||
            poses.empty())
        {
            ROS_FATAL("Failed to load first-pose reference: %s",
                      reference_pose_file.c_str());
            return 1;
        }
        first_pose = poses.front();

        // T_map_sensor0 comes from LIO-SAM's first saved key pose.
        const Eigen::Matrix4f map_T_sensor0 =
            poseMatrix(first_pose.x, first_pose.y, first_pose.z,
                       first_pose.roll, first_pose.pitch, first_pose.yaw);

        // URDF gives the pose of the sensor in base_link: T_base_sensor.
        const Eigen::Matrix4f base_T_sensor =
            poseMatrix(sensor_x, sensor_y, sensor_z,
                       sensor_roll_deg * kDegToRad,
                       sensor_pitch_deg * kDegToRad,
                       sensor_yaw_deg * kDegToRad);

        // Re-express old map points in base_link-at-start coordinates:
        //   T_map_base0 = T_map_sensor0 * T_sensor_base
        //   p_base0     = inverse(T_map_base0) * p_map
        const Eigen::Matrix4f map_T_base0 =
            map_T_sensor0 * base_T_sensor.inverse();
        transform = map_T_base0.inverse();
        derived_from_first_pose = true;
    }
    else
    {
        transform = poseMatrix(
            translation_x, translation_y, translation_z,
            roll_deg * kDegToRad,
            pitch_deg * kDegToRad,
            yaw_deg * kDegToRad);
    }

    pcl::PointCloud<pcl::PointXYZI> output;
    pcl::transformPointCloud(*input, output, transform);
    output.header = input->header;

    if (pcl::io::savePCDFileBinary(output_file, output) < 0)
    {
        ROS_FATAL("Failed to save transformed PCD: %s", output_file.c_str());
        return 1;
    }

    ROS_INFO("Transformed %zu points: %s -> %s",
             output.size(), input_file.c_str(), output_file.c_str());
    const Eigen::Vector3f applied_rpy = matrixRpy(transform);
    ROS_INFO("Applied map->base0 translation=(%.6f, %.6f, %.6f)m, "
             "RPY=(%.6f, %.6f, %.6f)deg",
             transform(0, 3), transform(1, 3), transform(2, 3),
             applied_rpy.x() / kDegToRad,
             applied_rpy.y() / kDegToRad,
             applied_rpy.z() / kDegToRad);
    if (derived_from_first_pose)
    {
        ROS_INFO("Derived from first key pose=(%.6f, %.6f, %.6f)m, "
                 "RPY=(%.6f, %.6f, %.6f)deg and base->sensor extrinsic",
                 first_pose.x, first_pose.y, first_pose.z,
                 first_pose.roll / kDegToRad,
                 first_pose.pitch / kDegToRad,
                 first_pose.yaw / kDegToRad);
    }

    if (!transform_info_output.empty())
    {
        std::ofstream info(transform_info_output);
        if (!info)
        {
            ROS_FATAL("Failed to write transform summary: %s",
                      transform_info_output.c_str());
            return 1;
        }
        info << "reference_pose_file: " << reference_pose_file << "\n"
             << "translation: [" << transform(0, 3) << ", "
             << transform(1, 3) << ", " << transform(2, 3) << "]\n"
             << "rpy_deg: [" << applied_rpy.x() / kDegToRad << ", "
             << applied_rpy.y() / kDegToRad << ", "
             << applied_rpy.z() / kDegToRad << "]\n"
             << "matrix:\n";
        for (int row = 0; row < 4; ++row)
        {
            info << "  - [";
            for (int col = 0; col < 4; ++col)
            {
                if (col > 0)
                    info << ", ";
                info << transform(row, col);
            }
            info << "]\n";
        }
        ROS_INFO("Saved alignment summary: %s", transform_info_output.c_str());
    }
    return 0;
}
