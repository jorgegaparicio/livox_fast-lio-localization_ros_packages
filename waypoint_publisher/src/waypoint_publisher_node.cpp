#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

nav_msgs::Odometry current_odom, current_map;
/*
// Callback for handling point messages
void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    static ros::Publisher pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("clicked_point", 10);
    
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose.position = msg->point;
    pose_msg.pose.orientation.w = 1.0; // Default orientation
    
    pose_pub.publish(pose_msg);
}*/

// Callback for handling pose messages
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (current_odom.header.stamp == ros::Time(0) || current_map.header.stamp == ros::Time(0)) {
        // We don't have data yet
        return;
    }
    static ros::Publisher pose_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    geometry_msgs::PoseStamped pose_msg;
    // REVISAR HEADER. AHORA LA REFERENCIA ES LA DE RVIZ. QUIZA SEA MEJOR  EL HEADER DEL GLOBAL O LOCAL MAP
    pose_msg.header = current_odom.header;
    pose_msg.pose.position = msg->pose.position;

    // TRADUZCO LA NUEVA POSICION SELECCIONADA A UNA POSICION EN LA REFERENCIA DEL MAPA LOCAL.
    pose_msg.pose.position.x = pose_msg.pose.position.x - current_map.pose.pose.position.x;
    pose_msg.pose.position.y = pose_msg.pose.position.y - current_map.pose.pose.position.y;
    
    // MANTENGO LA ALTURA Y ORIENTACION QUE TIENE EL DRON
    pose_msg.pose.position.z = current_odom.pose.pose.position.z;

    pose_msg.pose.orientation = current_odom.pose.pose.orientation; // MANTENGO LA ORIENTACION QUE TENIA.
  
    pose_pub.publish(pose_msg);
}

void mapCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_map = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map_to_odom", 10, mapCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odometry", 10, odomCallback);
    ros::Subscriber pose_sub = nh.subscribe("/clicked_pose", 10, poseCallback);

    ros::spin();
    return 0;
}

