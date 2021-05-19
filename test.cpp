#include "localization.h"
#include <iostream>
#include <ros/ros.h>
#include "ros_msg.h"
#include <geometry_msgs/Pose2D.h>
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    cartographer_interface localize("/home/lyc/Downloads/b2-2016-04-05-14-44-52.bag.pbstream",
                          0.05,
                          "/home/lyc/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files",
                          "backpack_2d.lua");

    ros::init(argc, argv, "localization");
    ros::start();
    ros::NodeHandle node_handle;
    ros_msg msg(localize,0.05);
    tf2_ros::TransformBroadcaster tf_broadcaster;
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);

    ros::Publisher trajectory_node_list_publisher =
            node_handle.advertise<::visualization_msgs::MarkerArray>(
                    "Trajectory", 1);
    ros::Publisher occupancy_grid_publisher = node_handle.advertise<::nav_msgs::OccupancyGrid>(
            "OccupancyGrid", 1,
            true /* latched */);
    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("pose_nav", 10);

    ros::Publisher scan_matched_pub =
            node_handle.advertise<sensor_msgs::PointCloud2>("laser", 1);

    ros::Subscriber m_laser_subscriber = node_handle.subscribe("echoes",1000,&ros_msg::laser_callback,&msg);
    ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu",1000,&ros_msg::imu_callback,&msg);
    //ros::Subscriber m_odometry_subscriber = node_handle.subscribe("odom",1000,&ros_msg::odometry_callback,&msg);
    ros::Rate loop_rate(100);
    //ScopedLogSink ros_log_sink;
    while(ros::ok())
    {
        // publish map
        occupancy_grid_publisher.publish(*msg.DrawAndPublish());
        geometry_msgs::Pose2D pos_now;
        cartographer::common::Time time = cartographer::common::FromUniversal(
                (ros::Time::now().sec +
                 ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                10000000ll +
                (ros::Time::now().nsec + 50) / 100);

        // publish point cloud


        const auto tracking_to_map = localize.tracking_to_map(time);
        pose_pub.publish(pos_now);
        // publish traje
        trajectory_node_list_publisher.publish(msg.GetTrajectoryNodeList());
        tf_broadcaster.sendTransform(msg.TrajectoryStates(localize.get_map_builder()));
        //ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

