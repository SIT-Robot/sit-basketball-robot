#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::shared_ptr<tf2_ros::TransformListener> tfl_;
std::shared_ptr<tf2_ros::Buffer> tf_;

const std::string base_frame_id_ = "base_footprint";
const std::string odom_frame_id_ = "odom";
const std::string map_frame_id_ = "map";


tf2::Transform odom_in_map;
void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg){
    //map下的base位姿
    tf2::Transform base_in_map;
    tf2::convert(msg.pose.pose,base_in_map);

    //base下map的位姿
    tf2::Transform map_in_base = base_in_map.inverse();
    geometry_msgs::PoseStamped map_in_base_pose;
    map_in_base_pose.header.frame_id = base_frame_id_;
    map_in_base_pose.header.stamp = msg.header.stamp;
    tf2::toMsg(map_in_base,map_in_base_pose.pose);

    geometry_msgs::PoseStamped map_in_odom_pose = tf_->transform(map_in_base_pose,"odom",ros::Duration(15));

    tf2::Transform map_in_odom;
    tf2::convert(map_in_odom_pose.pose,map_in_odom);

    odom_in_map = map_in_odom.inverse();

}

void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("接收到新位姿");
    handleInitialPoseMessage(*msg);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "location");

    ros::NodeHandle nh;
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

    auto subber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10,initialPoseReceived);
    ros::Rate rate(10);
    tf2_ros::TransformBroadcaster broadcaster;
    while (ros::ok()){

        geometry_msgs::TransformStamped tfs;
        tfs.header.frame_id = map_frame_id_;
        tfs.header.stamp = ros::Time::now();
        tfs.child_frame_id = odom_frame_id_;
        tfs.transform = tf2::toMsg(odom_in_map);
        broadcaster.sendTransform(tfs);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}