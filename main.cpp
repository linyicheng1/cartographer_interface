#include "cartographer_interface.h"
#include <ros/ros.h>
#include "sensor_ros.h"

// demo 示例程序
int main(int argc, char** argv)
{
    // 构造 cartographer_interface 类，传入配置文件目录和文件名
    cartographer_interface localize("/home/lyc/Downloads/b2-2016-04-05-14-44-52.bag.pbstream",
                          "/home/lyc/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files",
                          "backpack_2d.lua");
    // ROS 初始化 
    ros::init(argc, argv, "localization");
    ros::start();
    ros::NodeHandle node_handle;

    // ROS 数据接口类初始化 
    sensor_ros msg(localize, 0.05);
    // 发布两条数据 
    // 轨迹数据
    ros::Publisher trajectory_node_list_publisher =
            node_handle.advertise<::visualization_msgs::MarkerArray>(
                    "Trajectory", 1);
    // 占用栅格地图
    ros::Publisher occupancy_grid_publisher = node_handle.advertise<::nav_msgs::OccupancyGrid>(
            "OccupancyGrid", 1,
            true /* latched */);
    // 订阅传感器数据
    // 激光雷达数据 + IMU 数据 
    // 调用回调函数 ros_msg::laser_callback  ros_msg::imu_callback
    ros::Subscriber m_laser_subscriber = node_handle.subscribe("echoes",1000,&sensor_ros::laser_callback,&msg);
    ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu",1000,&sensor_ros::imu_callback,&msg);
    //ros::Subscriber m_odometry_subscriber = node_handle.subscribe("odom",1000,&ros_msg::odometry_callback,&msg);
    ros::Rate loop_rate(100);// 指定循环频率
    // log 类，取消注释可以打印出cartographer算法内部的消息
    //ScopedLogSink ros_log_sink;
    while(ros::ok())
    {
        // 发布地图和轨迹数据 
        // publish map
        occupancy_grid_publisher.publish(*msg.DrawAndPublish());
        // publish traje
        trajectory_node_list_publisher.publish(msg.GetTrajectoryNodeList());
        //ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

