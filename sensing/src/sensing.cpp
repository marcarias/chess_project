#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <sensing/getPiecePose.h>
#include <string>
#include <iostream>

tf2_ros::Buffer tfBuffer;

bool getPiecePose(
       sensing::getPiecePose::Request &req,
       sensing::getPiecePose::Response &res){
         std::string piece = std::to_string(req.piece);
         std::string aruco = "aruco_frame_";
         for(int i=0; i<3; i++){
           try{
             res.transformStamped = tfBuffer.lookupTransform("world", aruco + piece, ros::Time(0));//, ros::Duration(0.1));
           }
           catch (tf2::TransformException ex ){
             ROS_ERROR("%d - %s",i,ex.what());
             //exception due to not yet available tf. Publish joint_states and retry.
             continue;
           }
         }

       ROS_INFO("The pose of piece %s is:", piece.c_str());
       ROS_INFO("x: %f", res.transformStamped.transform.translation.x);
       ROS_INFO("y: %f", res.transformStamped.transform.translation.y);
       ROS_INFO("z: %f", res.transformStamped.transform.translation.z);
       ROS_INFO("qx: %f", res.transformStamped.transform.rotation.x);
       ROS_INFO("qy: %f", res.transformStamped.transform.rotation.y);
       ROS_INFO("qz: %f", res.transformStamped.transform.rotation.z);
       ROS_INFO("qw: %f", res.transformStamped.transform.rotation.w);
       return true;
}

int main(int argc, char **argv){
       ros::init(argc,argv,"sensing");
       ros::NodeHandle nh;

       ros::ServiceServer server =
               nh.advertiseService("get_piece_pose", getPiecePose);

       tf2_ros::TransformListener tfListener(tfBuffer);

       ros::spin();
       return 0;
}
