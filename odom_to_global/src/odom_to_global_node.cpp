#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

class PoseEstimator {
public:
    PoseEstimator() {
        // Initialize subscribers
        odom_sub = nh.subscribe("/Odometry", 10, &PoseEstimator::odomCallback, this);
        map_sub = nh.subscribe("/map_to_odom", 10, &PoseEstimator::mapCallback, this);

        // Initialize publisher
        global_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/global_pose", 10);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher global_pose_pub;

    nav_msgs::Odometry current_odom, current_map, global_odom;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom = *msg;
        publishGlobalPose();
    }

    void mapCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_map = *msg;
        publishGlobalPose();
    }

    void publishGlobalPose() {
        if (current_odom.header.stamp == ros::Time(0) || current_map.header.stamp == ros::Time(0)) {
            // We don't have data from both topics yet
            return;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        global_pose_msg.header = current_map.header;
        
        global_odom.pose.pose.position.x = current_odom.pose.pose.position.x + current_map.pose.pose.position.x;
        global_odom.pose.pose.position.y = current_odom.pose.pose.position.y + current_map.pose.pose.position.y;
        global_odom.pose.pose.position.z = current_odom.pose.pose.position.z + current_map.pose.pose.position.z;
        
        tf2::Quaternion q1, q2, q_combined;
        tf2::fromMsg(current_odom.pose.pose.orientation, q1);
        tf2::fromMsg(current_map.pose.pose.orientation, q2);
        q_combined = q1 * q2;  // Quaternion multiplication
        q_combined.normalize();  // Normalize the result

        global_odom.pose.pose.orientation = tf2::toMsg(q_combined);

        global_pose_msg.pose = global_odom.pose.pose;

        global_pose_pub.publish(global_pose_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_global");
    PoseEstimator pe;
    ros::spin();
    return 0;
}
