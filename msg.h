#ifndef PURE_LOCALIZATION_ROS_MSG_H
#define PURE_LOCALIZATION_ROS_MSG_H

#include "cartographer/mapping/id.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include "cartographer_interface.h"
#include "sensors_data.h"

class msg
{
public:
    explicit msg(cartographer_interface &lo,float resolution);
    // 获取可视化数据 
    cv::Mat  DrawAndPublish();

    // 传感器数据处理函数
    void laser_callback(const laser_data *msg);
    void imu_callback(const imu_data *msg);
    void odometry_callback(const odom_data *msg);
private:
    cv::Mat CreateOccupancyGridMsg(
            const cartographer::io::PaintSubmapSlicesResult& painted_slices,
            double resolution, const std::string& frame_id);
    void HandleLaserScan(
            const std::string& sensor_id, cartographer::common::Time time,
            const std::string& frame_id,
            const cartographer::sensor::PointCloudWithIntensities& points);
    void set_submaps();
    std::unique_ptr<cartographer::sensor::ImuData> ToImuData(
            const imu_data *msg);

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
