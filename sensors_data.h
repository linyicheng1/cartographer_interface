#ifndef CARTOGRAPHER_INTERFACE_SENSORS_DATA_H
#define CARTOGRAPHER_INTERFACE_SENSORS_DATA_H
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// imu data
typedef struct imu_data
{
    std::chrono::steady_clock::time_point stamp;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
}imu_data;

// laser data
typedef struct laser_data
{
    std::chrono::steady_clock::time_point stamp;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
}laser_data;

// odom data
typedef struct odom_data
{
    std::chrono::steady_clock::time_point stamp;

}odom_data;


#endif //CARTOGRAPHER_INTERFACE_SENSORS_DATA_H
