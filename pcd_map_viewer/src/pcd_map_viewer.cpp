#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "pcd_map_viewer");
    ros::NodeHandle nh;
    std::cout << "Using default map. To change map, replace the following file, maintaining the name and location:\n /home/grvc/catkin_ws/src/pcd_map_viewer/maps/scans_obstaculos.pcd" << std::endl;

    // Publisher for point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_topic", 1);

    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/grvc/catkin_ws/src/pcd_map_viewer/maps/scans_obstaculos.pcd", *cloud) == -1){
        ROS_ERROR("Couldn't read the PCD file");
        return -1;
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok()){
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
