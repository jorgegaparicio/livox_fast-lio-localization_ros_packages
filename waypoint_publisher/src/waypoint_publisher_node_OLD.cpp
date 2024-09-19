#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

// Callback for handling point messages
void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    static ros::Publisher pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("clicked_pose", 10);
    
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose.position = msg->point;
    pose_msg.pose.orientation.w = 1.0; // Default orientation
    
    pose_pub.publish(pose_msg);
}

// Callback for handling pose messages
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static ros::Publisher pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("clicked_pose", 10);
    pose_pub.publish(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    ros::Subscriber point_sub = nh.subscribe("/clicked_point", 10, pointCallback);
    ros::Subscriber pose_sub = nh.subscribe("/move_base_simple/goal", 10, poseCallback);

    ros::spin();
    return 0;
}

