#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <planning/planPick.h>
#include <planning/planPlace.h>
#include <planning/planMove.h>
#include <planning/planKill.h>
#include <planning/planCastle.h>
#include <tuple>
#include <string>
#include <iostream>

tf2_ros::Buffer tfBuffer;
std::map<int,double> lot;
int pickedPiece;
int picked_45;

// Function to pass from x y coordinates to Cell
std::string coordToCell(double x, double y){
  std::string cell;
  if (y <= 0.20 && y > 0.15 ){
    cell = 'A';
  } else if (y <= 0.15 && y > 0.10){
    cell = 'B';
  } else if (y <= 0.10 && y > 0.05){
    cell = 'C';
  } else if (y <= 0.05 && y > 0){
    cell = 'D';
  } else if (y <= 0.0 && y > -0.05){
    cell = 'E';
  } else if (y <= -0.05 && y > -0.10){
    cell = 'F';
  } else if (y <= -0.10 && y > -0.15){
    cell = 'G';
  } else if (y <= -0.15 && y > -0.20){
    cell = 'H';
  }

  if (x <= 0.20 && x > 0.15 ){
    cell = cell + '8';
  } else if (x <= 0.15 && x > 0.10){
    cell = cell + '7';
  } else if (x <= 0.10 && x > 0.05){
    cell = cell + '6';
  } else if (x <= 0.05 && x > 0.0){
    cell = cell + '5';
  } else if (x <= 0.0 && x > -0.05){
    cell = cell + '4';
  } else if (x <= -0.05 && x > -0.10){
    cell = cell + '3';
  } else if (x <= -0.10 && x > -0.15){
    cell = cell + '2';
  } else if (x <= -0.15 && x >= -0.20){
    cell = cell + '1';
  }
  return cell;
}

// Function to pass from CELL to x,y coordinates
std::tuple<double, double> cellToCoord(std::string cell){
  double x;
  double y;

  if (cell[0] == 'A'){
    y = 0.175;
  } else if (cell[0] == 'B'){
    y = 0.125;
  } else if (cell[0] == 'C'){
    y = 0.075;
  } else if (cell[0] == 'D'){
    y = 0.025;
  } else if (cell[0] == 'E'){
    y = -0.025;
  } else if (cell[0] == 'F'){
    y = -0.075;
  } else if (cell[0] == 'G'){
    y = -0.125;
  } else if (cell[0] == 'H'){
    y = -0.175;
  }

  if (cell[1] == '8'){
    x = 0.175;
  } else if (cell[1] == '7'){
    x = 0.125;
  } else if (cell[1] == '6'){
    x = 0.075;
  } else if (cell[1] == '5'){
    x = 0.025;
  } else if (cell[1] == '4'){
    x = -0.025;
  } else if (cell[1] == '3'){
    x = -0.075;
  } else if (cell[1] == '2'){
    x = -0.125;
  } else if (cell[1] == '1'){
    x = -0.175;
  }

  return std::make_tuple(x, y);
}



bool planPlace(
       planning::planPlace::Request &req,
       planning::planPlace::Response &res){
         std::tuple<double, double> coord;
         coord = cellToCoord(req.cell);
         if(picked_45 == 1){
         res.p1.transform.translation.x = std::get<1>(coord);
         res.p1.transform.translation.y = -std::get<0>(coord)+0.37-0.12;
         res.p1.transform.translation.z = lot[pickedPiece] + 0.265; // To change when simulation/real
         res.p1.transform.rotation.x= -0.924;
         res.p1.transform.rotation.w= 0.383;
         res.p2 = res.p1;
         res.p2.transform.translation.z = res.p2.transform.translation.z - 0.14;
       } else {
         res.p1.transform.translation.x = std::get<1>(coord);
         res.p1.transform.translation.y = -std::get<0>(coord)+0.37;
         res.p1.transform.translation.z = lot[pickedPiece] + 0.265; // To change when simulation/real
         res.p1.transform.rotation.x= 1;
         res.p2 = res.p1;
         res.p2.transform.translation.z = res.p2.transform.translation.z - 0.085; // To change when simulation/real
       }

       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y,
                            res.p1.transform.translation.z,
                            res.p1.transform.rotation.x,
                            res.p1.transform.rotation.y,
                            res.p1.transform.rotation.z,
                            res.p1.transform.rotation.w);
       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p2.transform.translation.x,
                            res.p2.transform.translation.y,
                            res.p2.transform.translation.z,
                            res.p2.transform.rotation.x,
                            res.p2.transform.rotation.y,
                            res.p2.transform.rotation.z,
                            res.p2.transform.rotation.w);
       ROS_INFO("PLACE IN CELL: %s", req.cell.c_str());
       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y,
                            res.p1.transform.translation.z,
                            res.p1.transform.rotation.x,
                            res.p1.transform.rotation.y,
                            res.p1.transform.rotation.z,
                            res.p1.transform.rotation.w);
       ROS_INFO("MOVE TO HOME");
       pickedPiece = 0;
       return true;
}

bool planPick(
       planning::planPick::Request &req,
       planning::planPick::Response &res){
         pickedPiece = req.piece;
         std::string cell;
         std::string piece = std::to_string(req.piece);
         std::string aruco = "aruco_frame_";
       for(int i=0; i<3; i++){
           try{
             res.p1 = tfBuffer.lookupTransform("world", aruco + piece, ros::Time(0));//, ros::Duration(0.1));
             res.cell = coordToCell(res.p1.transform.translation.x, res.p1.transform.translation.y);
             //res.p1 = tfBuffer.lookupTransform("team_A_base_link", aruco + piece, ros::Time(0));
             std::tuple<double, double> real_coord = cellToCoord(res.cell);
             if(res.cell[1] == '1' || res.cell[1] == '2'){
               res.p1.transform.translation.x = std::get<1>(real_coord);
               res.p1.transform.translation.y = -std::get<0>(real_coord)+0.37-0.12;
               res.p1.transform.translation.z = lot[pickedPiece] + 0.265; // To change when simulation/real
               res.p1.transform.rotation.x = -0.924;
               res.p1.transform.rotation.y = 0;
               res.p1.transform.rotation.z = 0;
               res.p1.transform.rotation.w = 0.383;
               res.p2 = res.p1;
               res.p2.transform.translation.z = res.p2.transform.translation.z - 0.145; // To change when simulation/real
               picked_45 = 1;
             } else {
               res.p1.transform.translation.x = std::get<1>(real_coord);
               res.p1.transform.translation.y = -std::get<0>(real_coord)+0.37;
               res.p1.transform.translation.z = lot[pickedPiece] + 0.265; // To change when simulation/real
               res.p1.transform.rotation.x = 1;
               res.p1.transform.rotation.y = 0;
               res.p1.transform.rotation.z = 0;
               res.p1.transform.rotation.w = 0;
               res.p2 = res.p1;
               res.p2.transform.translation.z = res.p2.transform.translation.z - 0.09; // To change when simulation/real
               picked_45 = 0;
           }
           }
           catch (tf2::TransformException ex ){
             ROS_ERROR("%d - %s",i,ex.what());
             //exception due to not yet available tf. Publish joint_states and retry.
             continue;
           }
         }

       ROS_INFO("OPEN GRIPPER");
       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y,
                            res.p1.transform.translation.z,
                            res.p1.transform.rotation.x,
                            res.p1.transform.rotation.y,
                            res.p1.transform.rotation.z,
                            res.p1.transform.rotation.w);
       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p2.transform.translation.x,
                            res.p2.transform.translation.y,
                            res.p2.transform.translation.z,
                            res.p2.transform.rotation.x,
                            res.p2.transform.rotation.y,
                            res.p2.transform.rotation.z,
                            res.p2.transform.rotation.w);
       ROS_INFO("PICK: %s FROM CELL: %s", piece.c_str(), res.cell.c_str());
       ROS_INFO("MOVE TO: x=%f y=%f z=%f qx=%f qy=%f qz=%f qw=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y,
                            res.p1.transform.translation.z,
                            res.p1.transform.rotation.x,
                            res.p1.transform.rotation.y,
                            res.p1.transform.rotation.z,
                            res.p1.transform.rotation.w);
       ROS_INFO("MOVE TO HOME");
       return true;
}

bool planMove(
       planning::planMove::Request &req,
       planning::planMove::Response &res){

       planning::planPick::Request req_pick;
       planning::planPick::Response resp_pick;
       req_pick.piece = req.piece;
       bool success = planPick(req_pick,resp_pick);

       res.p1 = resp_pick.p1;
       res.p2 = resp_pick.p2;

       planning::planPlace::Request req_place;
       planning::planPlace::Response resp_place;
       req_place.cell = req.cell;
       bool success2 = planPlace(req_place,resp_place);

       res.p3 = resp_place.p1;
       res.p4 = resp_place.p2;

       ROS_INFO("PICK FROM: x=%f y=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y);
       ROS_INFO("PLACE IN: x=%f y=%f",
                            res.p3.transform.translation.x,
                            res.p3.transform.translation.y);
       return true;
}

bool planKill(
       planning::planKill::Request &req,
       planning::planKill::Response &res){

       planning::planPick::Request req_pick1;
       planning::planPick::Response resp_pick1;
       req_pick1.piece = req.piece1;
       bool success = planPick(req_pick1,resp_pick1);

       res.p1 = resp_pick1.p1;
       res.p2 = resp_pick1.p2;

       planning::planPick::Request req_pick2;
       planning::planPick::Response resp_pick2;
       req_pick2.piece = req.piece2;
       bool success1 = planPick(req_pick2,resp_pick2);

       res.p3 = resp_pick2.p1;
       res.p4 = resp_pick2.p2;


       ROS_INFO("PICK FROM: x=%f y=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y);
       ROS_INFO("TRHOWAWAY");
       ROS_INFO("PICK FROM: x=%f y=%f",
                            res.p3.transform.translation.x,
                            res.p3.transform.translation.y);
       ROS_INFO("PLACE IN: x=%f y=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y);

       return true;
}

bool planCastle(
       planning::planCastle::Request &req,
       planning::planCastle::Response &res){

       planning::planPick::Request req_pick1;
       planning::planPick::Response resp_pick1;
       req_pick1.piece = req.piece1;
       bool success = planPick(req_pick1,resp_pick1);

       res.p1 = resp_pick1.p1;
       res.p2 = resp_pick1.p2;

       planning::planPick::Request req_pick2;
       planning::planPick::Response resp_pick2;
       req_pick2.piece = req.piece2;
       bool success1 = planPick(req_pick2,resp_pick2);

       res.p5 = resp_pick2.p1;
       res.p6 = resp_pick2.p2;


      std::string cell;
      cell = resp_pick1.cell;
      if (cell == "A1"){
        res.p3.transform.translation.x = 0.025;
        res.p3.transform.translation.y = -0.175;
        res.p7.transform.translation.x = 0.075;
        res.p7.transform.translation.y = -0.175;
      } else if (cell == "H1"){
        res.p3.transform.translation.x = -0.075;
        res.p3.transform.translation.y = -0.175;
        res.p7.transform.translation.x = -0.125;
        res.p7.transform.translation.y = -0.175;
      } else if (cell == "A8"){
        res.p3.transform.translation.x = 0.025;
        res.p3.transform.translation.y = 0.175;
        res.p7.transform.translation.x = 0.075;
        res.p7.transform.translation.y = 0.175;
      } else if (cell == "H8"){
        res.p3.transform.translation.x = -0.075;
        res.p3.transform.translation.y = 0.175;
        res.p7.transform.translation.x = -0.125;
        res.p7.transform.translation.y = 0.175;
      }
       res.p3.transform.translation.z = 0.2;
       res.p7.transform.translation.z = 0.2;
       res.p4 = res.p3;
       res.p8 = res.p7;
       res.p4.transform.translation.z = res.p4.transform.translation.z - 0.12;
       res.p8.transform.translation.z = res.p8.transform.translation.z - 0.12;

       ROS_INFO("PICK FROM: x=%f y=%f",
                            res.p1.transform.translation.x,
                            res.p1.transform.translation.y);
      ROS_INFO("PLACE IN: x=%f y=%f",
                           res.p3.transform.translation.x,
                           res.p3.transform.translation.y);
       ROS_INFO("PICK FROM: x=%f y=%f",
                            res.p5.transform.translation.x,
                            res.p5.transform.translation.y);
       ROS_INFO("PLACE IN: x=%f y=%f",
                            res.p7.transform.translation.x,
                            res.p7.transform.translation.y);

       return true;
}

int main(int argc, char **argv){
        lot[201] = 0.04;
        lot[202] = 0.04;
        lot[203] = 0.04;
        lot[204] = 0.04;
        lot[205] = 0.04;
        lot[206] = 0.04;
        lot[207] = 0.04;
        lot[208] = 0.04;
        lot[209] = 0.06;
        lot[210] = 0.06;
        lot[211] = 0.06;
        lot[212] = 0.06;
        lot[213] = 0.06;
        lot[214] = 0.06;
        lot[215] = 0.08;
        lot[216] = 0.08;
        lot[301] = 0.04;
        lot[302] = 0.04;
        lot[303] = 0.04;
        lot[304] = 0.04;
        lot[305] = 0.04;
        lot[306] = 0.04;
        lot[307] = 0.04;
        lot[308] = 0.04;
        lot[309] = 0.06;
        lot[310] = 0.06;
        lot[311] = 0.06;
        lot[312] = 0.06;
        lot[313] = 0.06;
        lot[314] = 0.06;
        lot[315] = 0.08;
        lot[316] = 0.08;
       ros::init(argc,argv,"planning");
       ros::NodeHandle nh;

       ros::ServiceServer server =
               nh.advertiseService("plan_pick",&planPick);

       ros::ServiceServer server2 =
               nh.advertiseService("plan_place",&planPlace);

       ros::ServiceServer server3 =
               nh.advertiseService("plan_move",&planMove);

       ros::ServiceServer server4 =
               nh.advertiseService("plan_kill",&planKill);

       ros::ServiceServer server5 =
               nh.advertiseService("plan_castle",&planCastle);
       tf2_ros::TransformListener tfListener(tfBuffer);

       ros::spin();
       return 0;
}
