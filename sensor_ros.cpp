#include "sensor_ros.h"
#include <geometry_msgs/Transform.h>
#include <utility>

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id);
void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers);
geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);


/**
 * @brief  构造函数，拿到接口类指针，并指定地图精度
 * @param lo  接口类指针
 * @param resolution  地图分辨率
 */
sensor_ros::sensor_ros(cartographer_interface &lo,float resolution):
        m_resolution(resolution),m_map_builder(lo.get_map_builder()),m_localization(lo)
{
}

/**
 * @brief  激光雷达数据回调函数，对激光数据进行了时间分配
 * @param msg 激光雷达数据
 */
void sensor_ros::laser_callback(sensor_msgs::MultiEchoLaserScanConstPtr msg)
{
    // 构造点云数据
    cartographer::sensor::PointCloudWithIntensities point_cloud;
    // 计算当前时间,公式和FromRos函数一致
    cartographer::common::Time time = cartographer::common::FromUniversal(
            (msg->header.stamp.sec +
             ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
            (msg->header.stamp.nsec + 50) / 100);
    // 当前帧最小的角度
    float angle = msg->angle_min;
    // 遍历所有激光数据
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        // 获取一个激光数据
        const auto& echoes = msg->ranges[i];
        if (!echoes.echoes.empty())
        {// 确保非空
            const float first_echo = echoes.echoes[0];// 当前激光的距离
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
                    point_cloud.intensities.push_back(echo_intensities.echoes[0]);
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
    m_localization.Handle2DLaserScanMessage("echoes",time, msg->header.frame_id,point_cloud.points);
}

/**
 * @brief IMU  数据回调函数
 * @param msg IMU 数据
 */
void sensor_ros::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
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
void sensor_ros::odometry_callback(const nav_msgs::Odometry &msg)
{
    //localize.HandleOdometryMessage();
}


/**
 * @brief  将ROS格式IMU 数据转换到 cartographer 格式的数据
 * @param msg IMU 数据
 * @return cartographer格式数据
 */
std::unique_ptr<cartographer::sensor::ImuData> sensor_ros::ToImuData(
        const sensor_msgs::Imu::ConstPtr& msg)
{
    // 检查协方差参数
    CHECK_NE(msg->linear_acceleration_covariance[0], -1)
            << "Your IMU data claims to not contain linear acceleration measurements "
               "by setting linear_acceleration_covariance[0] to -1. Cartographer "
               "requires this data to work. See "
               "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
    CHECK_NE(msg->angular_velocity_covariance[0], -1)
            << "Your IMU data claims to not contain angular velocity measurements "
               "by setting angular_velocity_covariance[0] to -1. Cartographer "
               "requires this data to work. See "
               "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
    // 获取当前时间
    const cartographer::common::Time time = FromRos(msg->header.stamp);
    // 构造  cartographer格式数据
    return cartographer::common::make_unique<cartographer::sensor::ImuData>(
            cartographer::sensor::ImuData{
                    time,
                    ToEigen(msg->linear_acceleration),
                    ToEigen(msg->angular_velocity)});
}

/**
 * @brief  ROS 格式的时间数据转换到cartotographer格式时间
 * @param time ROS 格式时间
 * @return  cartotographer格式时间
 */
cartographer::common::Time sensor_ros::FromRos(const ::ros::Time& time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return ::cartographer::common::FromUniversal(
            (time.sec +
             ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
            (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

/**
 * @brief  可视化地图数据
 * @return 返回栅格地图数据
 */
std::unique_ptr<nav_msgs::OccupancyGrid>  sensor_ros::DrawAndPublish()
{
    // 获取所有子地图
    set_submaps();
    // 绘制地图
    auto painted_slices = PaintSubmapSlices(m_submap_slices, m_resolution);
    // 绘制地图转换为栅格地图格式
    return CreateOccupancyGridMsg(
            painted_slices, m_resolution, m_last_frame_id, m_last_timestamp);
}

/**
 * @brief    获取轨迹数据
 * @return   可视化轨迹数据
 */
visualization_msgs::MarkerArray sensor_ros::GetTrajectoryNodeList()
{
    // 轨迹数据
    visualization_msgs::MarkerArray trajectory_node_list;
    const auto node_poses = m_map_builder->pose_graph()->GetTrajectoryNodePoses();
    // Find the last node indices for each trajectory that have either
    // inter-submap or inter-trajectory constraints.
    std::map<int, int /* node_index */>
            trajectory_to_last_inter_submap_constrained_node;
    std::map<int, int /* node_index */>
            trajectory_to_last_inter_trajectory_constrained_node;
    for (const int trajectory_id : node_poses.trajectory_ids()) {
        trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
        trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
    }
    const auto constraints = m_map_builder->pose_graph()->constraints();
    for (const auto& constraint : constraints) {
        if (constraint.tag == 1) {
            if (constraint.node_id.trajectory_id ==
                constraint.submap_id.trajectory_id) {
                trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                        .trajectory_id] =
                        std::max(trajectory_to_last_inter_submap_constrained_node.at(
                                constraint.node_id.trajectory_id),
                                 constraint.node_id.node_index);
            } else {
                trajectory_to_last_inter_trajectory_constrained_node
                [constraint.node_id.trajectory_id] =
                        std::max(trajectory_to_last_inter_submap_constrained_node.at(
                                constraint.node_id.trajectory_id),
                                 constraint.node_id.node_index);
            }
        }
    }

    for (const int trajectory_id : node_poses.trajectory_ids())
    {
        visualization_msgs::Marker marker =
                CreateTrajectoryMarker(trajectory_id, "map");
        int last_inter_submap_constrained_node = std::max(
                node_poses.trajectory(trajectory_id).begin()->id.node_index,
                trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
        int last_inter_trajectory_constrained_node = std::max(
                node_poses.trajectory(trajectory_id).begin()->id.node_index,
                trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
        last_inter_submap_constrained_node =
                std::max(last_inter_submap_constrained_node,
                         last_inter_trajectory_constrained_node);

        if (m_map_builder->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
            last_inter_submap_constrained_node =
                    (--node_poses.trajectory(trajectory_id).end())->id.node_index;
            last_inter_trajectory_constrained_node =
                    last_inter_submap_constrained_node;
        }

        marker.color.a = 1.0;
        for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
            if (!node_id_data.data.constant_pose_data.has_value()) {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                continue;
            }
            const ::geometry_msgs::Point node_point =
                    ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
            marker.points.push_back(node_point);

            if (node_id_data.id.node_index ==
                last_inter_trajectory_constrained_node) {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                marker.points.push_back(node_point);
                marker.color.a = 0.5;
            }
            if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                marker.points.push_back(node_point);
                marker.color.a = 0.25;
            }
            // Work around the 16384 point limit in RViz by splitting the
            // trajectory into multiple markers.
            if (marker.points.size() == 16384) {
                PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
                // Push back the last point, so the two markers appear connected.
                marker.points.push_back(node_point);
            }
        }
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
        if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
            trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
        } else {
            marker.action = visualization_msgs::Marker::DELETE;
            while (static_cast<size_t>(marker.id) <=
                   trajectory_to_highest_marker_id_[trajectory_id]) {
                trajectory_node_list.markers.push_back(marker);
                ++marker.id;
            }
            trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
        }
    }
    return trajectory_node_list;
}


ros::Time sensor_ros::ToRos(::cartographer::common::Time time)
{
    int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
    int64_t ns_since_unix_epoch =
            (uts_timestamp -
             ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
             10000000ll) *
            100ll;
    ::ros::Time ros_time;
    ros_time.fromNSec(ns_since_unix_epoch);
    return ros_time;
}

std::unique_ptr<nav_msgs::OccupancyGrid>
sensor_ros::CreateOccupancyGridMsg(const cartographer::io::PaintSubmapSlicesResult &painted_slices,
                                const double resolution, const std::string &frame_id, const ros::Time &time)
{
    auto occupancy_grid = cartographer::common::make_unique<nav_msgs::OccupancyGrid>();

    const int width = cairo_image_surface_get_width(painted_slices.surface.get());
    const int height = cairo_image_surface_get_height(painted_slices.surface.get());
    const ros::Time now = ros::Time::now();

    occupancy_grid->header.stamp = time;
    occupancy_grid->header.frame_id = frame_id;
    occupancy_grid->info.map_load_time = time;
    occupancy_grid->info.resolution = resolution;
    occupancy_grid->info.width = width;
    occupancy_grid->info.height = height;
    occupancy_grid->info.origin.position.x =
            -painted_slices.origin.x() * resolution;
    occupancy_grid->info.origin.position.y =
            (-height + painted_slices.origin.y()) * resolution;
    occupancy_grid->info.origin.position.z = 0.;
    occupancy_grid->info.origin.orientation.w = 1.;
    occupancy_grid->info.origin.orientation.x = 0.;
    occupancy_grid->info.origin.orientation.y = 0.;
    occupancy_grid->info.origin.orientation.z = 0.;

    const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
            cairo_image_surface_get_data(painted_slices.surface.get()));
    occupancy_grid->data.reserve(width * height);
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const uint32_t packed = pixel_data[y * width + x];
            const unsigned char color = packed >> 16;
            const unsigned char observed = packed >> 8;
            const int value =
                    observed == 0
                    ? -1
                    : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
            CHECK_LE(-1, value);
            CHECK_GE(100, value);
            occupancy_grid->data.push_back(value);
        }
    }
    return occupancy_grid;
}

void sensor_ros::set_submaps()
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
    m_last_timestamp = ros::Time::now();
}


::std_msgs::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
    ::std_msgs::ColorRGBA result;
    result.r = color[0];
    result.g = color[1];
    result.b = color[2];
    result.a = 1.f;
    return result;
}

visualization_msgs::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.ns = "Trajectory " + std::to_string(trajectory_id);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.stamp = ::ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
    marker.scale.x = 0.07;
    marker.pose.orientation.w = 1.;
    marker.pose.position.z = 0.05;
    return marker;
}
void PushAndResetLineMarker(visualization_msgs::Marker* marker,
                            std::vector<visualization_msgs::Marker>* markers) {
    markers->push_back(*marker);
    ++marker->id;
    marker->points.clear();
}
geometry_msgs::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d)
{
    geometry_msgs::Point point;
    point.x = vector3d.x();
    point.y = vector3d.y();
    point.z = vector3d.z();
    return point;
}