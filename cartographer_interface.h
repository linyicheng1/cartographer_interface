#ifndef PURE_LOCALIZATION_LOCALIZATION_H
#define PURE_LOCALIZATION_LOCALIZATION_H
#include <string>
#include <unordered_map>

#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/make_unique.h"

// 轨迹参数结构体
struct TrajectoryOptions {
    ::cartographer::mapping::proto::TrajectoryBuilderOptions
            trajectory_builder_options;
    std::string tracking_frame;
    std::string published_frame;
    std::string odom_frame;
    bool provide_odom_frame;
    bool use_odometry;
    bool use_nav_sat;
    bool use_landmarks;
    bool publish_frame_projected_to_2d;
    int num_laser_scans;
    int num_multi_echo_laser_scans;
    int num_subdivisions_per_laser_scan;
    int num_point_clouds;
    double rangefinder_sampling_ratio;
    double odometry_sampling_ratio;
    double fixed_frame_pose_sampling_ratio;
    double imu_sampling_ratio;
    double landmarks_sampling_ratio;
};

// 节点参数结构体
struct NodeOptions {
    ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
};

NodeOptions CreateNodeOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary);
TrajectoryOptions CreateTrajectoryOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary);
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
        const std::string& configuration_directory,
        const std::string& configuration_basename);

/**
 * @brief cartographer 算法接口 
 * */
class cartographer_interface
{
public:
    // 构造和析构函数
    ~cartographer_interface();
    explicit cartographer_interface(const std::string& map,
                          float resolution,
                          const std::string& configuration_directory,
                          const std::string& configuration_basename);

    // 传感器数据处理，目前实现激光数据、里程计数据和IMU数据，其他数据添加方式类似
    void HandleLaserScanMessage(
            const std::string& sensor_id, const cartographer::common::Time time,
            const std::string& frame_id, const cartographer::sensor::TimedPointCloud& ranges);
    void HandleOdometryMessage(const std::string& sensor_id,
                               cartographer::common::Time time,
                               cartographer::transform::Rigid3d pose);
    void HandleImuMessage(const std::string& sensor_id,
                          std::unique_ptr<cartographer::sensor::ImuData> &data);

                          
    std::unique_ptr<cartographer::mapping::MapBuilderInterface>& get_map_builder(){return m_map_builder;}
    cartographer::transform::Rigid3d tracking_to_map(cartographer::common::Time now);
private:
    bool read_map(const std::string& map_path);// read map from file

    void OnLocalSlamResult2(
            const int trajectory_id, const cartographer::common::Time time,
            const cartographer::transform::Rigid3d local_pose,
            cartographer::sensor::RangeData range_data_in_local);

    cartographer::io::PaintSubmapSlicesResult *m_painted_slices;
    int m_trajectory_id;
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> m_sensor_ids;
    cartographer::mapping::TrajectoryBuilderInterface*  m_trajectory_builder;
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> m_map_builder;
    NodeOptions m_node_options;
    TrajectoryOptions m_trajectory_options;
};



/**
 * @brief 继承LogSink类，可以打印出cartographer算法运行过程中的log信息，便于调试 
 */
class ScopedLogSink : public ::google::LogSink
{
public:
    ScopedLogSink();
    ~ScopedLogSink() override;
    // 重载的log函数，对不同类型的log数据进行处理
    void send(::google::LogSeverity severity, const char* filename,
              const char* base_filename, int line, const struct std::tm* tm_time,
              const char* message, size_t message_len) override;

    void WaitTillSent() override;

private:
    bool will_die_;
};

#endif //PURE_LOCALIZATION_LOCALIZATION_H
