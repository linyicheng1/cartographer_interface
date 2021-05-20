#include "cartographer_interface.h"
#include <utility>


/**
 * @brief  构造函数，设置传感器数据，并添加一条轨迹，加载当前地图 
 * @param map_path                  需要加载的地图路径
 * @param resolution                地图分辨率
 * @param configuration_directory   配置参数目录
 * @param configuration_basename    配置参数文件
 */
cartographer_interface::cartographer_interface(const std::string& map_path,
                           float resolution,
                           const std::string& configuration_directory,
                           const std::string& configuration_basename)
{
    // 从配置文件中加载配置参数
    std::tie(m_node_options, m_trajectory_options) = LoadOptions(configuration_directory,configuration_basename);
    // 调用 cartographer 接口，构造 MapBuilder 类
    m_map_builder = cartographer::common::make_unique<cartographer::mapping::MapBuilder>(m_node_options.map_builder_options);
    // 加载现有的地图数据
    //read_map(map_path);
    //@TODO need to change when change the sensors
    // 目前在本项目中使用一个2d雷达 + IMU + 轮速计
    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    m_sensor_ids.insert(SensorId{SensorType::RANGE, "echoes"});// laser 2d
    m_sensor_ids.insert(SensorId{SensorType::IMU, "imu"});// IMU
    //m_sensor_ids.insert(SensorId{SensorType::ODOMETRY, "odometry"});// odometry

    // 调用接口构造一条轨迹，返回轨迹id
    // m_sensor_ids   传感器类型及id
    // trajectory_builder_options 轨迹的配置参数
    // 最后一个参数为lambda表达式，为定位结果回调调用函数
    m_trajectory_id = m_map_builder->AddTrajectoryBuilder(m_sensor_ids, m_trajectory_options.trajectory_builder_options,
                                                          [this](const int id,
                                                                 const cartographer::common::Time time,
                                                                 const cartographer::transform::Rigid3d local_pose,
                                                                 cartographer::sensor::RangeData range_data_in_local,
                                                                 const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res) {
                                                              OnLocalSlamResult2(id, time, local_pose,
                                                                                 range_data_in_local);
                                                          });
    // 获取轨迹轨迹生成类指针
    m_trajectory_builder = m_map_builder->GetTrajectoryBuilder(m_trajectory_id);
}


/**
 * @brief  回调函数
 * @param id                   轨迹id
 * @param time                 时间
 * @param local_pose           局部地图上的位置
 * @param range_data_in_local  局部地图到全局地图的转换关系
 */
void cartographer_interface::OnLocalSlamResult2(
            const int id, const cartographer::common::Time time,
            const cartographer::transform::Rigid3d local_pose,
            cartographer::sensor::RangeData range_data_in_local)
{
    cartographer::transform::Rigid3d local2global = m_map_builder->pose_graph()->GetLocalToGlobalTransform(m_trajectory_id);
    cartographer::transform::Rigid3d pose3d = local2global * local_pose;
    std::cout<<"pose: x"<<pose3d.translation().x()<<" y "<<pose3d.translation().y()<<" angle "<<pose3d.rotation().z()<<std::endl;
    return;
}

cartographer_interface::~cartographer_interface()
{
    delete m_painted_slices;
}


/**
 * @brief read map from pbstream file
 * @param map_path
 * @return
 */
bool cartographer_interface::read_map(const std::string& map_path)
{
    const std::string suffix = ".pbstream";
    CHECK_EQ(map_path.substr(
            std::max<int>(map_path.size() - suffix.size(), 0)),
             suffix)
        << "The file containing the state to be loaded must be a "
           ".pbstream file.";
    cartographer::io::ProtoStreamReader stream(map_path);
    m_map_builder->LoadState(&stream,true);
    return true;
}

/**
 * @brief add one laser scan data with time && pos
 * @param sensor_id
 * @param time
 * @param pos
 * @param points
 */
void cartographer_interface::HandleLaserScanMessage(const std::string& sensor_id, const cartographer::common::Time time,
                                          const std::string& frame_id, const cartographer::sensor::TimedPointCloud& ranges)
{
    Eigen::Vector3d trans(0.1007,0,0.0558);
    Eigen::Quaterniond rot(1,0,0,0);
    cartographer::transform::Rigid3d sensor_to_tracking(trans, rot);
    m_trajectory_builder->AddSensorData(sensor_id,
                                        cartographer::sensor::TimedPointCloudData{
        time,
        sensor_to_tracking.translation().cast<float>(),
        cartographer::sensor::TransformTimedPointCloud(ranges,sensor_to_tracking.cast<float>())});

}

/**
 * @brief
 * @param sensor_id
 * @param time
 * @param pose
 */
void cartographer_interface::HandleOdometryMessage(const std::string& sensor_id,
                                         cartographer::common::Time time,
                                         cartographer::transform::Rigid3d pose)
{
    m_trajectory_builder->AddSensorData(
            sensor_id,
            cartographer::sensor::OdometryData{time, std::move(pose)});
}

/**
 * @brief
 * @param sensor_id
 * @param data
 */
void cartographer_interface::HandleImuMessage(
        const std::string& sensor_id,
        std::unique_ptr<cartographer::sensor::ImuData> &data)
{
    m_trajectory_builder->AddSensorData(
            sensor_id,
            *data);
}

/**
 * @brief  从配置文件中加载参数 
 * @param configuration_directory  配置文件目录
 * @param configuration_basename 配置文件名称
 * @return
 */
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
        const std::string& configuration_directory,
        const std::string& configuration_basename)
{
    auto file_resolver = cartographer::common::make_unique<
            cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});

    const std::string code =
            file_resolver->GetFileContentOrDie(configuration_basename);

    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
            code, std::move(file_resolver));

    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                           CreateTrajectoryOptions(&lua_parameter_dictionary));
}

/**
 * @brief  加载节点参数 
 * @param lua_parameter_dictionary
 * @return 
 */
NodeOptions CreateNodeOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary)
{
    NodeOptions options;
    options.map_builder_options =
            ::cartographer::mapping::CreateMapBuilderOptions(
                    lua_parameter_dictionary->GetDictionary("map_builder").get());
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    options.lookup_transform_timeout_sec =
            lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    options.submap_publish_period_sec =
            lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    options.pose_publish_period_sec =
            lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    options.trajectory_publish_period_sec =
            lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
    return options;
}

/**
 * @brief  加载轨迹参数
 * @param lua_parameter_dictionary
 * @return
 */
TrajectoryOptions CreateTrajectoryOptions(
        ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
    TrajectoryOptions options;
    options.trajectory_builder_options =
            ::cartographer::mapping::CreateTrajectoryBuilderOptions(
                    lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
    options.tracking_frame =
            lua_parameter_dictionary->GetString("tracking_frame");
    options.published_frame =
            lua_parameter_dictionary->GetString("published_frame");
    options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    options.provide_odom_frame =
            lua_parameter_dictionary->GetBool("provide_odom_frame");
    options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    options.publish_frame_projected_to_2d =
            lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
    options.num_laser_scans =
            lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
    options.num_multi_echo_laser_scans =
            lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
    options.num_subdivisions_per_laser_scan =
            lua_parameter_dictionary->GetNonNegativeInt(
                    "num_subdivisions_per_laser_scan");
    options.num_point_clouds =
            lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
    options.rangefinder_sampling_ratio =
            lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    options.odometry_sampling_ratio =
            lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    options.fixed_frame_pose_sampling_ratio =
            lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    options.imu_sampling_ratio =
            lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    options.landmarks_sampling_ratio =
            lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");
    return options;
}



const char* GetBasename(const char* filepath)
{
    const char* base = std::strrchr(filepath, '/');
    return base ? (base + 1) : filepath;
}

// 都是默认使用的构造函数
ScopedLogSink::ScopedLogSink() : will_die_(false) { AddLogSink(this); }
ScopedLogSink::~ScopedLogSink() { RemoveLogSink(this); }

/**
 * @brief 重载该函数实现log日志或直接输出
 * 
 * */
void ScopedLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) 
 {
     // 获取日志数据 
    const std::string message_string = ::google::LogSink::ToString(
            severity, GetBasename(filename), line, tm_time, message, message_len);
    // 根据不同消息等级进行处理
    switch (severity) 
    {
         // 普通输出   
        case ::google::GLOG_INFO:
            std::cout<<message_string<<std::endl;
            break;
        // 警告输出
        case ::google::GLOG_WARNING:
            std::cout<<message_string<<std::endl;
            break;
        // 错误输出
        case ::google::GLOG_ERROR:
            std::cerr<<message_string<<std::endl;
            break;
        // 致命错误输出
        case ::google::GLOG_FATAL:
            std::cerr<<message_string<<std::endl;
            will_die_ = true;
            break;
    }
}

/**
 * @brief 线程等待函数
 * */
void ScopedLogSink::WaitTillSent() 
{
    if (will_die_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}