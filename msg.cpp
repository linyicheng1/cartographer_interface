#include "msg.h"
#include "cartographer_interface.h"

/**
 * @brief  构造函数，拿到接口类指针，并指定地图精度
 * @param lo  接口类指针
 * @param resolution  地图分辨率
 */
msg::msg(cartographer_interface &lo,float resolution):
    m_resolution(resolution),m_map_builder(lo.get_map_builder()),m_localization(lo)
{
}

/**
 * @brief  激光雷达数据回调函数，对激光数据进行了时间分配
 * @param msg 激光雷达数据 
 */
auto start_time = std::chrono::steady_clock::now();
void msg::laser_callback(const laser_data *msg)
{
    // 构造点云数据 
    cartographer::sensor::PointCloudWithIntensities point_cloud;
    // 计算当前时间,公式和FromRos函数一致
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(msg->stamp - start_time).count();
    cartographer::common::Time time =
            cartographer::common::FromUniversal(duration / 100); //100ns
    // 当前帧最小的角度 
    float angle = msg->angle_min;
    // 遍历所有激光数据 
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        // 获取一个激光数据 
        //const auto& echoes = msg->ranges[i];
        //if (!echoes.echoes.empty())
        {// 确保非空
            const float first_echo = msg->ranges[i];// 当前激光的距离
            if (msg->range_min <= first_echo && first_echo <= msg->range_max)
            {// 确保激光数据在范围内 
                // 绕z轴旋转角度 angle
                const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                // cartographer 接口的激光数据 
                cartographer::sensor::TimedRangefinderPoint point;
                // 计算当前点的位置
                point.position = rotation * (first_echo * Eigen::Vector3f::UnitX());
                // 计算当前点的时间 ，注意该时间是在当前帧内的时间
                point.time = i * msg->time_increment;
                // 压入到点云数据
                point_cloud.points.push_back(point);
                // 如果激光数据有光强参数
                if (!msg->intensities.empty())
                {
                    // 压入光强数据
                    auto echo_intensities = msg->intensities[i];
                    point_cloud.intensities.push_back(echo_intensities);
                }
                else
                {
                    point_cloud.intensities.push_back(0.f);
                }
            }
        }
        // 更新当前激光角度 
        angle += msg->angle_increment;
    }
    // 确保当前获取的点云数据非空
    if (!point_cloud.points.empty())
    {
        // 获取最后一个点云的时间，即当前帧总共花费的时间
        auto duration = point_cloud.points.back().time;
        // 当前系统时间  + 当前帧花费的时间 ，得到真正的时间信息 
        time += cartographer::common::FromSeconds(duration);
        // 整体时间前移，认为最后一束激光为当前时间，其他激光都是稍早之前的数据
        for (auto &point : point_cloud.points)
        {
            point.time -= duration;
        }
    }
    // 调用 HandleLaserScan 处理激光雷达数据
    HandleLaserScan("echoes",time,"id",point_cloud);
}

/**
 * @brief IMU  数据回调函数
 * @param msg IMU 数据 
 */
void msg::imu_callback(const imu_data *msg)
{
    // ROS 格式数据转换到 cartographer 接口数据 
    std::unique_ptr<cartographer::sensor::ImuData> imu_data = ToImuData(msg);
    // 数据不为空，则调用接口将数据输入到算法中
    if (imu_data != nullptr)
    {
        m_localization.HandleImuMessage("imu",imu_data);
    }
}

/**
 * @brief 轮速计数据回调函数
 * @param msg 轮速计数据
 */
void msg::odometry_callback(const odom_data *msg)
{
    //localize.HandleOdometryMessage();
}

/**
 * @brief  处理激光雷达数据，将一帧激光雷达数据拆分成10份分别进行处理 
 * @param sensor_id  传感器id 
 * @param time  传感器时间
 * @param frame_id 帧id
 * @param points 点云数据 
 */
void msg::HandleLaserScan(
        const std::string& sensor_id, const cartographer::common::Time time,
        const std::string& frame_id,
        const cartographer::sensor::PointCloudWithIntensities& points)
{
    // 检查点云时间
    CHECK_LE(points.points.back().time, 0);
    // 将一帧激光数据分为10次输入
    for (int i = 0; i != m_num_subdivisions_per_laser_scan; ++i)
    {
        // 起始id
        const size_t start_index =
                points.points.size() * i / m_num_subdivisions_per_laser_scan;
        // 结束id 
        const size_t end_index =
                points.points.size() * (i + 1) / m_num_subdivisions_per_laser_scan;
        // 构造  subdivision       
        cartographer::sensor::TimedPointCloud subdivision(
                points.points.begin() + start_index, points.points.begin() + end_index);
        if (start_index == end_index)
        {
            continue;
        }
        // subdivision 的时间 
        const float time_to_subdivision_end = subdivision.back().time;
        // `subdivision_time` is the end of the measurement so sensor::Collator will
        // send all other sensor data first.
        // 重新分配时间
        const cartographer::common::Time subdivision_time =
                time + cartographer::common::FromSeconds(time_to_subdivision_end);
        auto it = m_sensor_to_previous_subdivision_time.find(sensor_id);
        if (it != m_sensor_to_previous_subdivision_time.end() &&
            it->second >= subdivision_time)
        {
            LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                         << sensor_id << " because previous subdivision time "
                         << it->second << " is not before current subdivision time "
                         << subdivision_time;
            continue;
        }
        m_sensor_to_previous_subdivision_time[sensor_id] = subdivision_time;
        for (auto& point : subdivision)
        {
            point.time -= time_to_subdivision_end;
        }
        // 调用接口对数据进行处理
        CHECK_EQ(subdivision.back().time, 0);
        m_localization.HandleLaserScanMessage(sensor_id, subdivision_time, frame_id, subdivision);
    }
}

/**
 * @brief  将ROS格式IMU 数据转换到 cartographer 格式的数据 
 * @param msg IMU 数据
 * @return cartographer格式数据 
 */
std::unique_ptr<cartographer::sensor::ImuData> msg::ToImuData(
        const imu_data *msg)
{
    // 获取当前时间 
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(msg->stamp - start_time).count();
    cartographer::common::Time time =
            cartographer::common::FromUniversal(duration / 100); //100ns
    // 构造  cartographer格式数据  
    return cartographer::common::make_unique<cartographer::sensor::ImuData>(
            cartographer::sensor::ImuData{
                    time,
                    msg->acc,
                    msg->gyro});
}

///////////////////////// visualization /////////////////////////////////

/**
 * @brief  可视化地图数据 
 * @return 返回栅格地图数据 
 */
cv::Mat msg::DrawAndPublish()
{
    // 获取所有子地图
    set_submaps();
    // 绘制地图
    auto painted_slices = PaintSubmapSlices(m_submap_slices, m_resolution);
    // 绘制地图转换为栅格地图格式
    return CreateOccupancyGridMsg(
            painted_slices, m_resolution, m_last_frame_id);
}


cv::Mat msg::CreateOccupancyGridMsg(const cartographer::io::PaintSubmapSlicesResult &painted_slices,
                                const double resolution, const std::string &frame_id)
{
    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height = cairo_image_surface_get_height(painted_slices.surface.get());
    cv::Mat occupancy_grid(cv::Size(width,height),CV_8U);
    memset(occupancy_grid.data,255,width*height);

    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
            cairo_image_surface_get_data(painted_slices.surface.get()));
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                    observed == 0
                    ? -1
                    : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(255, value);
            occupancy_grid.at<uchar>(y,x)=value;
        }
    }
    return occupancy_grid;
}

void msg::set_submaps()
{
    auto submaps_id_pose = m_map_builder->pose_graph()->GetAllSubmapPoses();
    if(submaps_id_pose.empty())
    {
        return;
    }
    std::set<cartographer::mapping::SubmapId> submap_ids_to_delete;
    for(const auto& pair:m_submap_slices)
    {
        submap_ids_to_delete.insert(pair.first);
    }
    for (const auto& submap_id_pose : submaps_id_pose)
    {
        submap_ids_to_delete.erase(submap_id_pose.id);
        auto& submap_slice = m_submap_slices[submap_id_pose.id];
        submap_slice.pose = submap_id_pose.data.pose;
        submap_slice.metadata_version = submap_id_pose.data.version;
        if(submap_slice.surface != nullptr && submap_slice.version == submap_id_pose.data.version)
        {// need not to update map
            continue;
        }
        cartographer::mapping::proto::SubmapQuery::Response response_proto;
        m_map_builder->SubmapToProto(submap_id_pose.id,&response_proto);
        submap_slice.version = response_proto.submap_version();
        for (const auto& texture_proto : response_proto.textures())
        {
        }
        submap_slice.width = response_proto.textures().begin()->width();
        submap_slice.height = response_proto.textures().begin()->height();
        auto pose = response_proto.textures().begin()->slice_pose();
        submap_slice.slice_pose = cartographer::transform::ToRigid3(response_proto.textures().begin()->slice_pose());
        submap_slice.resolution = response_proto.textures().begin()->resolution();
        submap_slice.cairo_data.clear();

        const std::string compressed_cells(response_proto.textures().begin()->cells().begin(),response_proto.textures().begin()->cells().end());
        const auto fetched_texture = cartographer::io::UnpackTextureData(
                compressed_cells,
                submap_slice.width,
                submap_slice.height);
        submap_slice.surface = cartographer::io::DrawTexture(
                fetched_texture.intensity, fetched_texture.alpha,
                submap_slice.width, submap_slice.height,
                &submap_slice.cairo_data);
    }
    for (const auto& id : submap_ids_to_delete)
    {
        m_submap_slices.erase(id);
    }
    m_last_frame_id = "map";
}
