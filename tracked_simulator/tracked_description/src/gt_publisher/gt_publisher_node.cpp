#include "ros/ros.h"
#include <string>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::TransformStamped odom_trans;
nav_msgs::Path path; // path

ros::Time current_time;
bool init_flag = false;
double bias_x = 0;
double bias_y = 0;
double bias_z = 0;

std::string header_id = "world";
std::string child_id = "ground_truth";

void chatterCallback(const nav_msgs::Odometry::ConstPtr &_odom)
{
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    path.header.stamp=current_time; // path
    
    if(init_flag == false){
        odom_trans.header.frame_id = header_id;
        odom_trans.child_frame_id = child_id;

        path.header.frame_id=header_id; // path

        bias_x = _odom->pose.pose.position.x;
        bias_y = _odom->pose.pose.position.y;
        bias_z = _odom->pose.pose.position.z;
        odom_trans.transform.translation.x = _odom->pose.pose.position.x - bias_x; 
        odom_trans.transform.translation.y = _odom->pose.pose.position.y - bias_y; 
        odom_trans.transform.translation.z = _odom->pose.pose.position.z - bias_z;  
        init_flag = true;
    }else{
        odom_trans.transform.translation.x = _odom->pose.pose.position.x - bias_x;
        odom_trans.transform.translation.y = _odom->pose.pose.position.y - bias_y;
        odom_trans.transform.translation.z = _odom->pose.pose.position.z - bias_z; 
    }
    odom_trans.transform.rotation = _odom->pose.pose.orientation;

    // this pose
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom_trans.transform.translation.x;
    this_pose_stamped.pose.position.y = odom_trans.transform.translation.y;
    this_pose_stamped.pose.position.z = odom_trans.transform.translation.z;
    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = child_id;

    path.poses.push_back(this_pose_stamped); // path
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gt_publisher");
    ros::NodeHandle n;
    ros::Rate r(10.0);
    tf::TransformBroadcaster odom_broadcaster;
    std::string sub_topic = "ground_truth/state";
    std::string pub_topic = "ground_truth/trajectory";
    if(argc!=5){
        ROS_WARN_STREAM("Usage：rosrun gt_publisher gt_publisher_node [world] [base_link]");
        return 0;
    }else{
        header_id = argv[1];
        child_id = argv[2];
        sub_topic = argv[3];
        pub_topic = argv[4];
    }
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>(pub_topic,100, true); // path
    ros::Subscriber sub = n.subscribe(sub_topic, 1000, chatterCallback);
    while(n.ok()){
        ros::spinOnce();
        odom_broadcaster.sendTransform(odom_trans);
        path_pub.publish(path); // path
        r.sleep();
    }
    return 0;
}