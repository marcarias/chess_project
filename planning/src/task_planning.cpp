//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <planning/planPick.h>
#include <planning/planPlace.h>
#include <planning/planMove.h>
#include <tuple>
#include <string>
#include <iostream>


int main(int argc, char **argv){
  ros::init(argc,argv,"task_planning");
  ros::NodeHandle nh;
  ros::ServiceClient spawnClient
          = nh.serviceClient<planning::planPick>("plan_pick");
  ros::ServiceClient spawnClient2
          = nh.serviceClient<planning::planPlace>("plan_place");




       ros::spin();
       return 0;
}
