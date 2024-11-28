#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <action/actionMove.h>
#include <action/actionKill.h>
#include <action/actionCastle.h>
#include <action/actionPick.h>
#include <action/actionPlace.h>
#include <action/planPlace.h>
#include <action/planPick.h>
#include <action/planMove.h>
#include <action/planKill.h>
#include <action/planCastle.h>
#include <action/gotoJoints.h>
#include <tuple>
#include <string>
#include <iostream>
#include <action/Attach.h>
#include <map>

std::map<int,std::string> lot;
int pickedPiece;
int killed;
int moved_45;

double x_home = 0;
double y_home = 0 + 0.37;
double z_home = 0.4;
double qx_home = 1;
double qy_home = 0;
double qz_home = 0;
double qw_home = 0;

// 52.6 -111.62 77.96 -87.75 -75.46 335.15

std::string coordToCell(double x, double y){
  std::string cell;
  if (x <= 0.20 && x > 0.15 ){
    cell = 'A';
  } else if (x <= 0.15 && x > 0.10){
    cell = 'B';
  } else if (x <= 0.10 && x > 0.05){
    cell = 'C';
  } else if (x <= 0.05 && x > 0){
    cell = 'D';
  } else if (x <= 0.0 && x > -0.05){
    cell = 'E';
  } else if (x <= -0.05 && x > -0.10){
    cell = 'F';
  } else if (x <= -0.10 && x > -0.15){
    cell = 'G';
  } else if (x <= -0.15 && x > -0.20){
    cell = 'H';
  }

  if (y <= 0.20 && y > 0.15 ){
    cell = cell + '8';
  } else if (y <= 0.15 && y > 0.10){
    cell = cell + '7';
  } else if (y <= 0.10 && y > 0.05){
    cell = cell + '6';
  } else if (y <= 0.05 && y > 0.0){
    cell = cell + '5';
  } else if (y <= 0.0 && y > -0.05){
    cell = cell + '4';
  } else if (y <= -0.05 && y > -0.10){
    cell = cell + '3';
  } else if (y <= -0.10 && y > -0.15){
    cell = cell + '2';
  } else if (y <= -0.15 && y > -0.20){
    cell = cell + '1';
  }
  return cell;
}

bool actionPick(
       action::actionPick::Request &req,
       action::actionPick::Response &res){
         //get points from planning
         action::planPick::Request req_plan;
         action::planPick::Response resp_plan;
         req_plan.piece = req.piece;
         pickedPiece = req.piece;
         bool success = ros::service::call("plan_pick", req_plan, resp_plan);

         //ensure gripper is opened
         std_srvs::Empty::Request req_open1;
         std_srvs::Empty::Response resp_open1;

         success = ros::service::call("/team_A_arm/gripper_open", req_open1, resp_open1);

         //move to first point
         action::gotoJoints::Request req_p1;
         action::gotoJoints::Response resp_p1;

         req_p1.px = resp_plan.p1.transform.translation.x;
         req_p1.py = resp_plan.p1.transform.translation.y;
         req_p1.pz = resp_plan.p1.transform.translation.z;
         req_p1.qx = resp_plan.p1.transform.rotation.x;
         req_p1.qy = resp_plan.p1.transform.rotation.y;
         req_p1.qz = resp_plan.p1.transform.rotation.z;
         req_p1.qw = resp_plan.p1.transform.rotation.w;
         req_p1.trajduration = 3;

         if(moved_45 == 1){
           req_p1.py = req_p1.py-0.12;
           req_p1.qx = -0.924;
           req_p1.qw = 0.383;
         }

         success = ros::service::call("move_ur3", req_p1, resp_p1);
         if(success == false){
           return false;
         }
         //move to second point
         action::gotoJoints::Request req_p2;
         action::gotoJoints::Response resp_p2;

         req_p2.px = resp_plan.p2.transform.translation.x;
         req_p2.py = resp_plan.p2.transform.translation.y;
         req_p2.pz = resp_plan.p2.transform.translation.z;
         req_p2.qx = resp_plan.p2.transform.rotation.x;
         req_p2.qy = resp_plan.p2.transform.rotation.y;
         req_p2.qz = resp_plan.p2.transform.rotation.z;
         req_p2.qw = resp_plan.p2.transform.rotation.w;
         req_p2.trajduration = 3;

         if(moved_45 == 1){
           req_p2.py = req_p2.py-0.12;
           req_p2.pz = req_p2.pz-0.055;
           req_p2.qx = -0.924;
           req_p2.qw = 0.383;
         }

         success = ros::service::call("move_ur3", req_p2, resp_p2);
         if(success == false){
           return false;
         }
         //close gripper
         action::Attach::Request a_req;
         action::Attach::Response a_resp;
         a_req.model_name_1 = lot[req.piece];
         a_req.link_name_1 = "link";
         a_req.model_name_2 = "team_A_arm";
         a_req.link_name_2 = "team_A_wrist_3_link";

         std_srvs::Empty::Request req_close;
         std_srvs::Empty::Response resp_close;

         success = ros::service::call("/team_A_arm/gripper_close", req_close, resp_close);
         ros::Duration(1).sleep();
         success = ros::service::call("/link_attacher_node/attach",a_req,a_resp);
         //move to first point
         success = ros::service::call("move_ur3", req_p1, resp_p1);
         if(success == false){
           return false;
         }
      return true;
}

bool actionHome(
       std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res){

      action::gotoJoints::Request req_home;
      action::gotoJoints::Response resp_home;

      req_home.px = x_home;
      req_home.py = y_home;
      req_home.pz = z_home;
      req_home.qx = qx_home;
      req_home.qy = qy_home;
      req_home.qz = qz_home;
      req_home.qw = qw_home;
      req_home.trajduration = 3;

      bool success = ros::service::call("move_ur3", req_home, resp_home);
      if(success == false){
        return false;
      }
      return true;
      }

bool actionPlace(
       action::actionPlace::Request &req,
       action::actionPlace::Response &res){
         //get points from planning
         action::planPlace::Request req_plan;
         action::planPlace::Response resp_plan;
         req_plan.cell = req.cell;
         bool success = ros::service::call("plan_place", req_plan, resp_plan);

         //move to first point
         action::gotoJoints::Request req_p1;
         action::gotoJoints::Response resp_p1;

         req_p1.px = resp_plan.p1.transform.translation.x;
         req_p1.py = resp_plan.p1.transform.translation.y;
         req_p1.pz = resp_plan.p1.transform.translation.z;
         req_p1.qx = resp_plan.p1.transform.rotation.x;
         req_p1.qy = resp_plan.p1.transform.rotation.y;
         req_p1.qz = resp_plan.p1.transform.rotation.z;
         req_p1.qw = resp_plan.p1.transform.rotation.w;
         req_p1.trajduration = 3;

         if(moved_45 == 1){
           req_p1.py = req_p1.py-0.12;
           req_p1.qx = -0.924;
           req_p1.qw = 0.383;
         }

         success = ros::service::call("move_ur3", req_p1, resp_p1);
         if(success == false){
           return false;
         }
         //move to second point
         action::gotoJoints::Request req_p2;
         action::gotoJoints::Response resp_p2;

         req_p2.px = resp_plan.p2.transform.translation.x;
         req_p2.py = resp_plan.p2.transform.translation.y;
         req_p2.pz = resp_plan.p2.transform.translation.z;
         req_p2.qx = resp_plan.p2.transform.rotation.x;
         req_p2.qy = resp_plan.p2.transform.rotation.y;
         req_p2.qz = resp_plan.p2.transform.rotation.z;
         req_p2.qw = resp_plan.p2.transform.rotation.w;
         req_p2.trajduration = 3;

         if(moved_45 == 1){
           req_p2.py = req_p2.py-0.12;
           req_p2.pz = req_p2.pz-0.055;
           req_p2.qx = -0.924;
           req_p2.qw = 0.383;
           moved_45 = 0;
         }

         success = ros::service::call("move_ur3", req_p2, resp_p2);
         if(success == false){
           return false;
         }
         //open gripper
         action::Attach::Request a_req;
         action::Attach::Response a_resp;
         a_req.model_name_1 = lot[pickedPiece];
         a_req.link_name_1 = "link";
         a_req.model_name_2 = "team_A_arm";
         a_req.link_name_2 = "team_A_wrist_3_link";
         std_srvs::Empty::Request req_open1;
         std_srvs::Empty::Response resp_open1;
         success = ros::service::call("/link_attacher_node/detach",a_req,a_resp);
         ros::Duration(1).sleep();
         success = ros::service::call("/team_A_arm/gripper_open", req_open1, resp_open1);
         pickedPiece = 0;


         //move to first point
         success = ros::service::call("move_ur3", req_p1, resp_p1);
         if(success == false){
           return false;
         }
      return true;
}

bool actionMove(
       action::actionMove::Request &req,
       action::actionMove::Response &res){
      //get points from planning
      action::planMove::Request req_plan;
      action::planMove::Response resp_plan;
      req_plan.piece = req.piece;
      req_plan.cell = req.cell;
      bool success = ros::service::call("plan_move", req_plan, resp_plan);

      if (req.cell[1] == '1' || req.cell[1] == '2'){
        moved_45 = 1;
      } else{
        moved_45 = 0;
      }
      // Pick
      action::actionPick::Request req_pick;
      action::actionPick::Response resp_pick;

      req_pick.piece = req.piece;

      success = actionPick(req_pick,resp_pick);

      // Place
      action::actionPlace::Request req_place;
      action::actionPlace::Response resp_place;

      req_place.cell = req.cell;

      success = actionPlace(req_place,resp_place);

      //move to HOME
      std_srvs::Empty::Request req_home;
      std_srvs::Empty::Response resp_home;

      success = actionHome(req_home, resp_home);

      return true;
}

bool actionKill(
       action::actionKill::Request &req,
       action::actionKill::Response &res){
      //get points from planning
      action::planKill::Request req_plan;
      action::planKill::Response resp_plan;
      req_plan.piece1 = req.piece1;
      req_plan.piece2 = req.piece2;
      bool success = ros::service::call("plan_kill", req_plan, resp_plan);

      // Pick killed
      action::actionPick::Request req_pick;
      action::actionPick::Response resp_pick;

      req_pick.piece = req.piece1;

      success = actionPick(req_pick,resp_pick);

      //move to trash
      action::gotoJoints::Request req_trash;
      action::gotoJoints::Response resp_trash;

      req_trash.px = -0.25;
      req_trash.py = -0.25+killed*0.05+0.37;
      req_trash.pz = 0.265+0.08;
      req_trash.qx = 1;
      req_trash.trajduration = 3;

      success = ros::service::call("move_ur3", req_trash, resp_trash);
      if(success == false){
        return false;
      }

      req_trash.pz = 0.26;

      success = ros::service::call("move_ur3", req_trash, resp_trash);
      if(success == false){
        return false;
      }

      //open gripper
      action::Attach::Request a_req;
      action::Attach::Response a_resp;
      a_req.model_name_1 = lot[pickedPiece];
      a_req.link_name_1 = "link";
      a_req.model_name_2 = "team_A_arm";
      a_req.link_name_2 = "team_A_wrist_3_link";
      std_srvs::Empty::Request req_open1;
      std_srvs::Empty::Response resp_open1;
      pickedPiece = 0;
      success = ros::service::call("/link_attacher_node/detach",a_req,a_resp);
      ros::Duration(1).sleep();
      success = ros::service::call("/team_A_arm/gripper_open", req_open1, resp_open1);

      req_trash.pz = 0.265+0.08;

      killed = killed + 1;

      success = ros::service::call("move_ur3", req_trash, resp_trash);
      if(success == false){
        return false;
      }

      // Pick killer
      req_pick.piece = req.piece2;

      success = actionPick(req_pick,resp_pick);

      // Place killer
      action::actionPlace::Request req_place;
      action::actionPlace::Response resp_place;

      req_place.cell = coordToCell(resp_plan.p2.transform.translation.x, -resp_plan.p2.transform.translation.y+0.37);

      success = actionPlace(req_place,resp_place);

      //move to HOME
      std_srvs::Empty::Request req_home;
      std_srvs::Empty::Response resp_home;

      success = actionHome(req_home, resp_home);

      return true;
}

bool actionCastle(
       action::actionCastle::Request &req,
       action::actionCastle::Response &res){
      //get points from planning
      action::planCastle::Request req_plan;
      action::planCastle::Response resp_plan;
      req_plan.piece1 = req.piece1;
      req_plan.piece2 = req.piece2;
      bool success = ros::service::call("plan_castle", req_plan, resp_plan);

      //action_move for rook (p1, p2, p3, p4)
      action::actionMove::Request req_rook;
      action::actionMove::Response resp_rook;
      req_rook.piece = req.piece1;
      req_rook.cell = coordToCell(resp_plan.p3.transform.translation.x,-resp_plan.p3.transform.translation.y+0.37);
      success = actionMove(req_rook, resp_rook);

      //action_move for rook (p1, p2, p3, p4)
      action::actionMove::Request req_king;
      action::actionMove::Response resp_king;
      req_king.piece = req.piece2;
      req_king.cell = coordToCell(resp_plan.p7.transform.translation.x,-resp_plan.p7.transform.translation.y+0.37);
      success = actionMove(req_king, resp_king);

      return true;
}

bool testMove(
       std_srvs::Empty::Request &req,
       std_srvs::Empty::Response &res){

      action::actionMove::Request req_m;
      action::actionMove::Response resp_m;

      req_m.piece = 205;
      req_m.cell = "A5";
      bool success = actionMove(req_m, resp_m);
      req_m.piece = 216;
      req_m.cell = "B5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 213;
      req_m.cell = "C5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 204;
      req_m.cell = "D5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 206;
      req_m.cell = "E5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 212;
      req_m.cell = "F5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 215;
      req_m.cell = "G5";
      success = actionMove(req_m, resp_m);
      req_m.piece = 208;
      req_m.cell = "H5";
      success = actionMove(req_m, resp_m);

      return true;
}

bool testKill(
       std_srvs::Empty::Request &req,
       std_srvs::Empty::Response &res){

      action::actionKill::Request req_m;
      action::actionKill::Response resp_m;

      req_m.piece1 = 205;
      req_m.piece2 = 216;
      bool success = actionKill(req_m, resp_m);
      req_m.piece1 = 213;
      req_m.piece2 = 204;
      success = actionKill(req_m, resp_m);
      req_m.piece1 = 206;
      req_m.piece2 = 212;
      success = actionKill(req_m, resp_m);
      req_m.piece1 = 215;
      req_m.piece2 = 208;
      success = actionKill(req_m, resp_m);

      return true;
}

bool testCastle(
       std_srvs::Empty::Request &req,
       std_srvs::Empty::Response &res){

      action::actionMove::Request req_m;
      action::actionMove::Response resp_m;
      action::actionCastle::Request req_c;
      action::actionCastle::Response resp_c;

      req_c.piece1 = 209;
      req_c.piece2 = 216;
      bool success = actionCastle(req_c, resp_c);
      req_m.piece = 216;
      req_m.cell = "E8";
      success = actionMove(req_m, resp_m);
      req_c.piece1 = 210;
      req_c.piece2 = 216;
      success = actionCastle(req_c, resp_c);


      return true;
}

int main(int argc, char **argv){
       lot[201] = "pawnB1";
       lot[202] = "pawnB2";
       lot[203] = "pawnB3";
       lot[204] = "pawnB4";
       lot[205] = "pawnB5";
       lot[206] = "pawnB6";
       lot[207] = "pawnB7";
       lot[208] = "pawnB8";
       lot[209] = "rookB1";
       lot[210] = "rookB2";
       lot[211] = "knightB1";
       lot[212] = "knightB2";
       lot[213] = "bishopB1";
       lot[214] = "bishopB2";
       lot[215] = "queenB";
       lot[216] = "kingB";
       lot[301] = "pawnW1";
       lot[302] = "pawnW2";
       lot[303] = "pawnW3";
       lot[304] = "pawnW4";
       lot[305] = "pawnW5";
       lot[306] = "pawnW6";
       lot[307] = "pawnW7";
       lot[308] = "pawnW8";
       lot[309] = "rookW1";
       lot[310] = "rookW2";
       lot[311] = "knightW1";
       lot[312] = "knightW2";
       lot[313] = "bishopW1";
       lot[314] = "bishopW2";
       lot[315] = "queenW";
       lot[316] = "kingW";
       killed = 0;

       ros::init(argc,argv,"action");
       ros::NodeHandle nh;

       ros::ServiceServer server =
               nh.advertiseService("action_move", actionMove);
       ros::ServiceServer server2 =
               nh.advertiseService("action_kill", actionKill);
       ros::ServiceServer server3 =
               nh.advertiseService("action_castle", actionCastle);
       ros::ServiceServer server4 =
               nh.advertiseService("action_pick", actionPick);
       ros::ServiceServer server5 =
               nh.advertiseService("action_place", actionPlace);
       ros::ServiceServer server6 =
               nh.advertiseService("action_home", actionHome);
       ros::ServiceServer server7 =
               nh.advertiseService("test_move", testMove);
       ros::ServiceServer server8 =
               nh.advertiseService("test_kill", testKill);
       ros::ServiceServer server9 =
               nh.advertiseService("test_castle", testCastle);

       ros::spin();

       return 0;
}
