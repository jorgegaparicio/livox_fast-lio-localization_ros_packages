#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

class PoseEstimator {
public:
    PoseEstimator() {
        // Initialize subscribers
        map3d_sub = nh.subscribe("/global_pose", 10, &PoseEstimator::mapCallback, this);

        // Initialize publisher
        map2d_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/flat_map_pose", 10);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber map3d_sub;
    ros::Publisher map2d_pose_pub;

    //nav_msgs::Odometry map2d_pose;
    geometry_msgs::PoseStamped global_pose;

    void mapCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        global_pose = *msg;
        publish2Dpose();
    }

    void publish2Dpose() {

        geometry_msgs::PoseStamped map2d_pose_msg;
        map2d_pose_msg.header = global_pose.header;
        
        map2d_pose_msg.pose.position.x = global_pose.pose.position.x;
        map2d_pose_msg.pose.position.y = global_pose.pose.position.y;
        //map2d_pose_msg.pose = global_pose.pose.pose;

        map2d_pose_pub.publish(map2d_pose_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_pose_to_flat_map");
    PoseEstimator pe;
    ros::spin();
    return 0;
}
