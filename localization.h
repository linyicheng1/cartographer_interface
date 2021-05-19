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

struct NodeOptions {
    ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
    std::string map_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
};

struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
            : rangefinder_sampler(rangefinder_sampling_ratio),
              odometry_sampler(odometry_sampling_ratio),
              fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
              imu_sampler(imu_sampling_ratio),
              landmark_sampler(landmark_sampling_ratio) {}

    cartographer::common::FixedRatioSampler rangefinder_sampler;
    cartographer::common::FixedRatioSampler odometry_sampler;
    cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    cartographer::common::FixedRatioSampler imu_sampler;
    cartographer::common::FixedRatioSampler landmark_sampler;
};

class localization
{
public:
    struct LocalSlamData {
        ::cartographer::common::Time time;
        ::cartographer::transform::Rigid3d local_pose;
        ::cartographer::sensor::RangeData range_data_in_local;
    };
    ~localization();
    explicit localization(const std::string& map,
                          float resolution,
                          const std::string& configuration_directory,
                          const std::string& configuration_basename);

    // read sensor data
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
    void OnLocalSlamResult(
            const int trajectory_id, const cartographer::common::Time time,
            const cartographer::transform::Rigid3d local_pose,
            cartographer::sensor::RangeData range_data_in_local,
            const std::unique_ptr<const ::cartographer::mapping::
            TrajectoryBuilderInterface::InsertionResult>
            insertion_result);
    void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
    void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
    void OnLocalSlamResult2(
            const int trajectory_id, const cartographer::common::Time time,
            const cartographer::transform::Rigid3d local_pose,
            cartographer::sensor::RangeData range_data_in_local);
    float m_resolution;
    std::string m_map_path_str;// map path
    cartographer::io::PaintSubmapSlicesResult *m_painted_slices;
    int m_trajectory_id;
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId> m_sensor_ids;
    cartographer::mapping::TrajectoryBuilderInterface*  m_trajectory_builder;
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> m_map_builder;
    NodeOptions m_node_options;
    TrajectoryOptions m_trajectory_options;
    std::shared_ptr<const LocalSlamData> m_local_slam_data;
    std::unordered_map<int, TrajectorySensorSamplers> m_sensor_samplers;
    std::map<int, ::cartographer::mapping::PoseExtrapolator> m_extrapolators;
};

class ScopedLogSink : public ::google::LogSink {
public:
    ScopedLogSink();
    ~ScopedLogSink() override;

    void send(::google::LogSeverity severity, const char* filename,
              const char* base_filename, int line, const struct std::tm* tm_time,
              const char* message, size_t message_len) override;

    void WaitTillSent() override;

private:
    bool will_die_;
};
#endif //PURE_LOCALIZATION_LOCALIZATION_H
