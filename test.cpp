#include "localization.h"
#include <ros/ros.h>
#include "ros_msg.h"


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

    ros::Publisher trajectory_node_list_publisher =
            node_handle.advertise<::visualization_msgs::MarkerArray>(
                    "Trajectory", 1);
    ros::Publisher occupancy_grid_publisher = node_handle.advertise<::nav_msgs::OccupancyGrid>(
            "OccupancyGrid", 1,
            true /* latched */);

    ros::Subscriber m_laser_subscriber = node_handle.subscribe("echoes",1000,&ros_msg::laser_callback,&msg);
    ros::Subscriber m_imu_subscriber = node_handle.subscribe("imu",1000,&ros_msg::imu_callback,&msg);
    //ros::Subscriber m_odometry_subscriber = node_handle.subscribe("odom",1000,&ros_msg::odometry_callback,&msg);
    ros::Rate loop_rate(100);
    //ScopedLogSink ros_log_sink;
    while(ros::ok())
    {
        // publish map
        occupancy_grid_publisher.publish(*msg.DrawAndPublish());
        // publish traje
        trajectory_node_list_publisher.publish(msg.GetTrajectoryNodeList());
        //ros::spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

