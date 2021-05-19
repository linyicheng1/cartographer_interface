#ifndef PURE_LOCALIZATION_ROS_MSG_H
#define PURE_LOCALIZATION_ROS_MSG_H
#include <ros/ros.h>
#include "cartographer/mapping/id.h"
#include "cartographer/io/submap_painter.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/transform/transform.h"
#include "visualization_msgs/MarkerArray.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <nav_msgs/Odometry.h>
#include "localization.h"

class ros_msg
{
public:
    explicit ros_msg(cartographer_interface &lo,float resolution);

    std::unique_ptr<nav_msgs::OccupancyGrid>  DrawAndPublish();
    visualization_msgs::MarkerArray GetTrajectoryNodeList();
    std::vector<geometry_msgs::TransformStamped> TrajectoryStates(std::unique_ptr<cartographer::mapping::MapBuilderInterface> &map_builder);
    void laser_callback(sensor_msgs::MultiEchoLaserScanConstPtr msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void odometry_callback(const nav_msgs::Odometry &msg);
private:
    std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
            const cartographer::io::PaintSubmapSlicesResult& painted_slices,
            double resolution, const std::string& frame_id,
            const ros::Time& time);
    void HandleLaserScan(
            const std::string& sensor_id, const cartographer::common::Time time,
            const std::string& frame_id,
            const cartographer::sensor::PointCloudWithIntensities& points);
    void set_submaps();
    std::unique_ptr<cartographer::sensor::ImuData> ToImuData(
            const sensor_msgs::Imu::ConstPtr& msg);
    cartographer::common::Time FromRos(const ::ros::Time& time);
    ros::Time ToRos(::cartographer::common::Time time);
    Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
        return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }
    std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
    std::map<cartographer::mapping::SubmapId, cartographer::io::SubmapSlice> m_submap_slices;
    std::string m_last_frame_id;
    ros::Time m_last_timestamp;
    float m_resolution;
    int m_num_subdivisions_per_laser_scan = 10;
    std::unique_ptr<cartographer::mapping::MapBuilderInterface>& m_map_builder;
    std::map<std::string, cartographer::common::Time> m_sensor_to_previous_subdivision_time;
    cartographer_interface &m_localization;
};


#endif //PURE_LOCALIZATION_ROS_MSG_H
